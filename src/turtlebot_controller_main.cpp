#include "rclcpp/rclcpp.hpp"
#include "turtlebot_controller.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlebotController>());
  rclcpp::shutdown();
  return 0;
}