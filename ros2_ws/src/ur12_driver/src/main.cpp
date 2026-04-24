#include "ur12_driver/ur_driver_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ur12_driver::UrDriverNode>());
  rclcpp::shutdown();
  return 0;
}
