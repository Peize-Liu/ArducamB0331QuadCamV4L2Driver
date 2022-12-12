#include "arducam_driver_v4l2.hpp"
cv::Mat& convert(cv::Mat& raw_imag, int& rows);
void setExposureGain(int exp, int gain);
double clearness(cv::Mat & img) ;

ArduCamV4L2Driver::ArduCamV4L2Driver(){
  cv::setNumThreads(1);
  // buffer_free_list_.clear();
  // buffer_available_list_.clear();
}

ArduCamV4L2Driver::~ArduCamV4L2Driver(){
  printf("Arducam destructor\n");
  if(!buffer_vect_.empty()){
    for(auto iter = buffer_vect_.begin(); iter != buffer_vect_.end(); iter++){
      munmap(iter->image_buffer_addr,iter->buffer_size);
    }
  }
  int32_t type =  V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if(ioctl(cam_fd_,VIDIOC_STREAMOFF,&type) < 0){
    printf("Image caputure stop failed!\n");
  }
  close(cam_fd_);
}

int32_t ArduCamV4L2Driver::Init(ros::NodeHandle & nh) {
//Load parameters
  nh.param<bool>("raw8", config_.raw8);
  nh.param<bool>("publish_splited", config_.publish_splited);
  nh.param<bool>("print_clearness", config_.print_clearness);
  nh.param<bool>("show", config_.show);
  nh.param<bool>("sync", config_.is_sync);
  nh.param<int>("fps", config_.fps);
  nh.param<int>("width", config_.width);
  nh.param<int>("height", config_.height);
  nh.param<int>("cap_device", config_.cap_device);
  nh.param<int>("camera_num", config_.camera_num);
  nh.param<int>("exposure", config_.exposure);
  nh.param<int>("gain", config_.gain);
  nh.param("buffer_frame_number",config_.buffer_frame_num);
  printf("Set fps:%d , set: buffer_frame_number:%d \n",config_.fps, config_.buffer_frame_num);

//Open camera file 
  cam_fd_ = open("/dev/video0",O_RDWR|O_NONBLOCK,0);
  if(cam_fd_ < 0){
    printf("Failed to open /dev/video0\n");
    return -1;
  }
  printf("camera_fd =%d\n",cam_fd_);
  struct v4l2_capability cam_cap;
  if(ioctl(cam_fd_,VIDIOC_QUERYCAP,&cam_cap) == -1){
    printf("Unable to query devic \n");
    return -2;
  }
  if((cam_cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0){
    printf("Camera video unable to capture\n");
    return -3;
  }
//Set camera parameters
  struct v4l2_format camera_fmt;
  memset(&camera_fmt,0,sizeof(camera_fmt));
  camera_fmt.type = V4L2_CAP_VIDEO_CAPTURE;
  camera_fmt.fmt.pix.width = config_.width;
  camera_fmt.fmt.pix.height =  config_.height;
  camera_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_SRGGB8;
  camera_fmt.fmt.pix.field = V4L2_FIELD_ANY;
  if(ioctl(cam_fd_,VIDIOC_S_FMT,&camera_fmt) == -1){
    printf("Set camera format failed -4\n");
    return -4;
  }

// Check camera parameters
  struct v4l2_format camera_fmt_req;
  camera_fmt_req.type = V4L2_CAP_VIDEO_CAPTURE;
  if(ioctl(cam_fd_, VIDIOC_G_FMT, &camera_fmt_req) == -1){
    printf("require camera parameter failed\n");
    return -5;
  }
  if(camera_fmt_req.fmt.pix.pixelformat =! V4L2_PIX_FMT_SRGGB8){
    printf("Set camera format failed -6\n");
    return -6;
  }

// request buffer to store image date
  struct v4l2_requestbuffers buff_req;
  buff_req.count = config_.buffer_frame_num;
  buff_req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buff_req.memory = V4L2_MEMORY_MMAP;
  if(ioctl(cam_fd_, VIDIOC_REQBUFS,&buff_req) == -1){
    printf("Request buffer failed\n");
    return -7;
  }
  printf("request %d frame in kernel\n",buff_req.count);

//Init buffer mmap, a frame a buffer, 
  struct v4l2_buffer camera_buffer;
  camera_buffer.length = camera_fmt.fmt.pix.width*camera_fmt.fmt.pix.height;
  camera_buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  camera_buffer.memory = V4L2_MEMORY_MMAP;
  //mmap kernel memory and put memory back to kernel to let v4l2 to use
  for(int32_t i=0; i<buff_req.count; i++){
    camera_buffer.index = i;
    if(ioctl(cam_fd_,VIDIOC_QUERYBUF,&camera_buffer) < 0){
        printf("Unable to query buffer.\n");
        return -7;
      };
      //mmap
    uint8_t* frame_addr = (uint8_t*)mmap(NULL,camera_buffer.length, //record addr to release in destruct
      PROT_READ|PROT_WRITE,MAP_SHARED,cam_fd_,camera_buffer.m.offset);
    if(frame_addr == MAP_FAILED){
      printf("MMAP error -1\n");
      return -8;
    }
    //put buffer back to kernel
    if(ioctl(cam_fd_,VIDIOC_QBUF,&camera_buffer) == -1){ 
      printf("return buffer to kernel failed\n");
      return -10;
    }
    ImageBufferNode new_node(frame_addr, camera_buffer.length, (uint32_t)0, i);
    buffer_vect_.push_back(new_node);
  }

// Open Stream input
  int32_t type =  V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if(ioctl(cam_fd_,VIDIOC_STREAMON,&type) < 0){
    printf("Image caputure start failed!\n");
    return -9; 
  }

  //Create publishers
  image_transport_ = new image_transport::ImageTransport(nh);
  pub_raw_ = image_transport_->advertise("/arducam/image", 1);

  if (config_.publish_splited) {
      for (int i = 0; i < config_.camera_num; i++) {
          std::stringstream ss;
          ss << "/arducam/image_" << i;
          pub_splited_.push_back(image_transport_->advertise(ss.str(), 1));
      }
  }

  //Start grab thread if success
  if (config_.is_sync) {
      grab_thread_ = std::thread(&ArduCamV4L2Driver::grabThread, this);
  } else {
      //Software trigger
      grab_timer_ = nh.createTimer(ros::Duration(1.0/config_.fps), &ArduCamV4L2Driver::grabRos, this);
  }
  printf("Image caputure start\n");
  init_flag_ = 0;
  return 0;
}


void ArduCamV4L2Driver::grabRos(const ros::TimerEvent & event) {
    grab();
}

void ArduCamV4L2Driver::grab() {
  static struct v4l2_buffer new_image;
  new_image.length = config_.width*config_.height;
  new_image.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  new_image.memory = V4L2_MEMORY_MMAP;
  auto ret = ioctl(cam_fd_,VIDIOC_DQBUF,&new_image);//考虑用select 优化这个位置 
  if(ret < 0){
    return;
  }
  if (frame_count_ == 0) {
      tstart_ = ros::Time::now();
  }
  auto ts = ros::Time::now();
  cv::Mat frame(config_.width,config_.height,CV_8U,buffer_vect_[new_image.index].image_buffer_addr);

  frame = convert(frame, config_.height); //占用及其高
  if(ioctl(cam_fd_,VIDIOC_QBUF,&new_image) < 0){
    printf("return buffer failed\n");
  }
  return;
  if (!frame.empty()) {
    cv_bridge::CvImage cv_img;
    cv_img.header.stamp = ts;
    cv_img.header.frame_id = "arducam";
    cv_img.encoding = "bgr8";
    cv_img.image = frame;
    cv::Mat show_image;
    if (config_.show && cam_shown_ == -1) {
      cv::resize(frame, show_image, 
          cv::Size(frame.cols / config_.camera_num, frame.rows / config_.camera_num));
    }
    //publish
    pub_raw_.publish(cv_img.toImageMsg());
    if (config_.publish_splited || config_.print_clearness || (cam_shown_ >= 0 && config_.show)) {
      if (config_.print_clearness) {
          printf("[ArduCam] clearness:");
      }
      for (int i = 0; i < config_.camera_num; i++) {
        if (config_.publish_splited || cam_shown_ == i || config_.print_clearness) {
            cv_img.image = frame(cv::Rect(i * frame.cols / config_.camera_num, 0, 
                    frame.cols / config_.camera_num, frame.rows));
            if (config_.publish_splited) {
                pub_splited_[i].publish(cv_img.toImageMsg());
            }
            if (config_.print_clearness) {
                printf("%d: %.1f%%\t", i, clearness(cv_img.image)*100);
            }
        }
        if (config_.show && cam_shown_ == i) {
            show_image = cv_img.image;
        }
      }
      if (config_.print_clearness) {
          printf("\n");
      }
    }

    if (config_.show) {
        printf("Show image\n");
        showImage(show_image);
    }

    if (frame_count_ == 0) {
        setExposureGain(config_.exposure, config_.gain);
    }
    frame_count_ ++;
    double tgrab = (ros::Time::now() - tstart_).toSec();
    ROS_INFO_THROTTLE(1.0, "[ArduCam] Total %d freq:%.1ffps", 
      frame_count_, frame_count_/tgrab);
  } else {
    ROS_WARN("[ArduCam] Failed to grab a frame");
  }

  //returb buffer to kernel
  if(ioctl(cam_fd_,VIDIOC_QBUF,&new_image) < 0){
    printf("return buffer failed\n");
  }
}

void ArduCamV4L2Driver::showImage(cv::Mat & show_image) {

  char title[64] = {0};
  if (cam_shown_ != -1) {
      //Show clearness on image
      double clear = clearness(show_image);
      sprintf(title, "Clearness %.1f%%", clear*100);
      cv::putText(show_image, title, cv::Point(10, 30), 
              cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 255, 0));
  }
  sprintf(title, "Cam %d +/- to switch", cam_shown_);
  cv::putText(show_image, title, cv::Point(10, 10), 
              cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 255));

  cv::imshow("ArduCam", show_image);
  int key = cv::waitKey(1);
  if (key==61) {
    cam_shown_ = (cam_shown_ + 2)%(config_.camera_num + 1) - 1;
  } else {
    if (key==45){
      cam_shown_ = cam_shown_%(config_.camera_num + 1) - 1;
    }
  } 
}

void ArduCamV4L2Driver::grabThread() {
  ROS_INFO("[ArduCam] Start to grab\n");
  tstart_ = ros::Time::now();
  while (ros::ok()) {
    grab();
    ros::Duration(0.1 / config_.fps).sleep();
  }
  return;
}

cv::Mat& convert(cv::Mat& raw_imag, int& rows) {
  cv::Mat img = raw_imag.reshape(0, rows);
  static cv::Mat encode_image;
  struct timeval start_time, end_time;
  gettimeofday(&start_time,NULL);
  cv::cvtColor(img, encode_image, cv::COLOR_BayerRG2BGR);
  gettimeofday(&end_time,NULL);
  printf("use time: sec:%ld usec:%ld\n",(end_time.tv_sec-start_time.tv_sec), (end_time.tv_usec-start_time.tv_usec));
  return encode_image;
}

void setExposureGain(int exp, int gain) {
  char cmd[64] = {0};
  printf("Setting exposure to %d gain to %d by v4l2-ctrl", exp, gain);
  sprintf(cmd, "/usr/bin/v4l2-ctl -c exposure=%d", exp);
  auto ret = system(cmd);
  sprintf(cmd, "/usr/bin/v4l2-ctl -c gain=%d", gain);
  ret = system(cmd);
}

double clearness(cv::Mat & img) {
  //Clearness for focus
  cv::Mat gray, imgSobel;
  cv::Rect2d roi(img.cols/3, img.rows/3, img.cols/3, img.rows/3);
  cv::rectangle(img, roi, cv::Scalar(255, 0, 0), 1);
  cv::cvtColor(img(roi), gray, cv::COLOR_BGR2GRAY);
  cv::Sobel(gray, imgSobel, CV_16U, 1, 1);
  return cv::mean(imgSobel)[0];
}