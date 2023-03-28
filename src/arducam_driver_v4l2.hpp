#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <list>
#include <vector>
#include <linux/videodev2.h>
#include <memory.h>
#include <unistd.h>
#include <time.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

#define COLOR_RAW_8 1111967570
#define COLOR_Y16 540422489

struct ArduCamConfig {
  bool raw8 = true;
  bool show = false;
  bool publish_splited = false;
  bool print_clearness = false;
  bool is_sync = false;

  int32_t buffer_frame_num = 2;
  int32_t fps =20;
  int32_t width = 5120;
  int32_t height = 800;
  int32_t cap_device = 0;
  int32_t camera_num = 4;
  int32_t exposure = 300;
  int32_t gain = 1;
};

class ArduCamV4L2Driver {
 public:
  ArduCamV4L2Driver();
  ~ArduCamV4L2Driver();
  int32_t Init(ros::NodeHandle & nh);
 private:  
  void grabThread();
  void grab();
  void grabRos(const ros::TimerEvent & event);
  void showImage(cv::Mat & show);

  struct ImageBufferNode{
    uint8_t* image_buffer_addr;
    int32_t buffer_size;
    int32_t used_size;
    int32_t index;
    ImageBufferNode(uint8_t* buffer_addr, int32_t buffer_size, 
    int32_t used_size, int32_t index) : image_buffer_addr(buffer_addr),buffer_size(buffer_size),
    used_size(used_size),index(index){}
  };

  std::vector<ImageBufferNode> buffer_vect_; 

  int32_t init_flag_= -1;
  int32_t cam_fd_ = -1;
  ArduCamConfig config_;
  std::thread grab_thread_;
  cv::VideoCapture video_cap_;
  image_transport::Publisher pub_raw_;
  std::vector<image_transport::Publisher> pub_splited_;
  int frame_count_ = 0;
  int cam_shown_ = -1;
  ros::Time tstart_;
  ros::Timer grab_timer_;  
  image_transport::ImageTransport * image_transport_ = nullptr;
  uint8_t* local_frame_addr = nullptr; //OpenCV has problem when dealing with mmap memory
  size_t local_frame_size = -1;
};
