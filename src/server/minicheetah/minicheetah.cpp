#include <ros_controller.h>


int main(int argc, char* argv[]) {
  ros::init(argc, argv, "minicheetah_sim");
  MPCControllerRos controller(1000);
  controller.preWork();
  while (ros::ok())
  {
    controller.spin();
  }

}
