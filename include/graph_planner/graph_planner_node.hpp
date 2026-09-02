#ifndef GRAPH_PLANNER__GRAPH_PLANNER_NODE_HPP_
#define GRAPH_PLANNER__GRAPH_PLANNER_NODE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp> // Added for RViz markers
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

namespace graph_planner
{

struct Vertex
{
  int id;
  double x;
  double y;
  double yaw;
};

struct Edge
{
  int target_id;
  double weight;
};

class GraphPlannerNode : public rclcpp::Node
{
public:
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleNavThrough = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  explicit GraphPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~GraphPlannerNode() = default;

private:
  void load_graph_file(const std::string & file_path);
  void publish_graph_markers(); // Helper to publish RViz markers
  std::optional<geometry_msgs::msg::PoseStamped> get_current_robot_pose();
  int find_nearest_vertex(const geometry_msgs::msg::PoseStamped & pose);
  std::vector<int> compute_dijkstra_path(int start_id, int target_id);
  
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal);
  
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);
  
  void handle_accepted(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

  void execute_plan(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

  void send_nav2_goal(
    const std::vector<geometry_msgs::msg::PoseStamped> & waypoints,
    std::shared_ptr<GoalHandleNavigateToPose> graph_goal_handle);

  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav2_action_client_;
  
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_; // Marker Publisher

  rclcpp::TimerBase::SharedPtr marker_timer_;

  std::unordered_map<int, Vertex> vertices_;
  std::unordered_map<int, std::vector<Edge>> adjacency_list_;

  std::string global_frame_;
  std::string robot_base_frame_;
};

}  // namespace graph_planner

#endif  // GRAPH_PLANNER__GRAPH_PLANNER_NODE_HPP_