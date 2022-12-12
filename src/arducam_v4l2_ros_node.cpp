#include "arducam_driver_v4l2.hpp"

//main function for arducam_ros_node
int main(int argc, char **argv) {
  ros::init(argc, argv, "arducam_v4l2_ros");
  ros::NodeHandle nh("arducam_v4l2_ros");
  ArduCamV4L2Driver driver;
  int32_t ret = driver.Init(nh);
  if(ret < 0){
    printf("Init failed error:%d\n",ret);
    return -1;
  }
  ros::spin();
  return 0;
}