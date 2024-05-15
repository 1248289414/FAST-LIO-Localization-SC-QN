#include "fast_lio_localization_sc_qn/main.h"

void SigHandle(int sig)
{
    std::cout << "You pressed Ctrl + C, exiting" << std::endl;
    exit(1);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nh_private = rclcpp::Node::make_shared("fast_lio_localization_sc_qn_node");
  FastLioLocalizationScQnClass FastLioLocalizationScQn_(nh_private);

  signal(SIGINT, SigHandle);
  
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(nh_private);
  executor.spin();
 
  return 0;
}
