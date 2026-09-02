#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "graph_planner/graph_planner_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<graph_planner::GraphPlannerNode>(options);
  
  // MultiThreadedExecutor keeps execution smooth during long actions/TF frame polling
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}