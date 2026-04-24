#include "ur12_driver/ur_driver_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // SingleThreadedExecutor works for this node; the action execute functions
  // spawn their own threads so they don't block the executor.
  rclcpp::executors::SingleThreadedExecutor exe;
  auto node = std::make_shared<ur12_driver::UrDriverNode>();
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
