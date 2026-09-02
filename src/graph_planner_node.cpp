#include "graph_planner/graph_planner_node.hpp"

#include <fstream>
#include <cmath>
#include <queue>
#include <limits>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace graph_planner
{

using json = nlohmann::json;

GraphPlannerNode::GraphPlannerNode(const rclcpp::NodeOptions & options)
: Node("graph_planner_node", options)
{
  this->declare_parameter<std::string>("graph_file_path", "");
  this->declare_parameter<std::string>("global_frame", "map");
  this->declare_parameter<std::string>("robot_base_frame", "base_link");

  global_frame_ = this->get_parameter("global_frame").as_string();
  robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
  std::string graph_path = this->get_parameter("graph_file_path").as_string();

  if (graph_path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Parameter 'graph_file_path' is required!");
    throw std::runtime_error("Missing parameter: graph_file_path");
  }

  // Publisher setup with transient local QoS (latched topic) so RViz gets markers on connect
  rclcpp::QoS marker_qos(10);
  marker_qos.transient_local();
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("graph_planner/markers", marker_qos);

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("graph_planner/path", 10);

  load_graph_file(graph_path);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Publish markers every 1 second so Foxglove and late-connecting clients always see the graph
  marker_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&GraphPlannerNode::publish_graph_markers, this)
  );

  nav2_action_client_ = rclcpp_action::create_client<NavigateThroughPoses>(
    this, "navigate_through_poses");

  action_server_ = rclcpp_action::create_server<NavigateToPose>(
    this,
    "plan_graph_path",
    std::bind(&GraphPlannerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GraphPlannerNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&GraphPlannerNode::handle_accepted, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "Graph Planner Node cleanly initialized!");
}

void GraphPlannerNode::load_graph_file(const std::string & file_path)
{
  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open graph JSON file: %s", file_path.c_str());
    throw std::runtime_error("File opening failure");
  }

  json j;
  file >> j;

  for (const auto & v : j["vertices"]) {
    int id = v["id"];
    double x = v["x"];
    double y = v["y"];
    double yaw = v["yaw"];
    vertices_[id] = Vertex{id, x, y, yaw};
  }

  size_t total_edges = 0;
  for (const auto & e : j["edges"]) {
    int src_id = e["source"];
    int tgt_id = e["target"];
    double weight = e["weight"];

    if (vertices_.count(src_id) && vertices_.count(tgt_id)) {
      adjacency_list_[src_id].push_back(Edge{tgt_id, weight});
      total_edges++;
    }
  }

  RCLCPP_INFO(
    this->get_logger(), "Graph loaded with %zu vertices and %zu edges.",
    vertices_.size(), total_edges);

  publish_graph_markers();
}

void GraphPlannerNode::publish_graph_markers()
{
  visualization_msgs::msg::MarkerArray marker_array;
  int id_counter = 0;

  // 1. Vertices (Spheres)
  for (const auto & [id, v] : vertices_) {
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = global_frame_;
    sphere.header.stamp = this->now();
    sphere.ns = "vertices";
    sphere.id = id_counter++;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;

    sphere.pose.position.x = v.x;
    sphere.pose.position.y = v.y;
    sphere.pose.position.z = 0.1;

    sphere.scale.x = 0.2;
    sphere.scale.y = 0.2;
    sphere.scale.z = 0.2;

    sphere.color.r = 0.0f;
    sphere.color.g = 0.8f;
    sphere.color.b = 1.0f;
    sphere.color.a = 1.0f; // Cyan color

    marker_array.markers.push_back(sphere);

    // Vertex ID Text
    visualization_msgs::msg::Marker text;
    text.header.frame_id = global_frame_;
    text.header.stamp = this->now();
    text.ns = "vertex_ids";
    text.id = id_counter++;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;

    text.pose.position.x = v.x;
    text.pose.position.y = v.y;
    text.pose.position.z = 0.5;

    text.scale.z = 0.3; // Text height
    text.color.r = 0.0f;
    text.color.g = 0.0f;
    text.color.b = 0.0f;
    text.color.a = 1.0f;

    text.text = "Node " + std::to_string(id);
    marker_array.markers.push_back(text);
  }

  // 2. Edges (Directed Arrows)
  for (const auto & [src_id, edges] : adjacency_list_) {
    const auto & src_v = vertices_[src_id];

    for (const auto & edge : edges) {
      const auto & tgt_v = vertices_[edge.target_id];

      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = global_frame_;
      arrow.header.stamp = this->now();
      arrow.ns = "edges";
      arrow.id = id_counter++;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point p_start, p_end;
      p_start.x = src_v.x;
      p_start.y = src_v.y;
      p_start.z = 0.1;

      p_end.x = tgt_v.x;
      p_end.y = tgt_v.y;
      p_end.z = 0.1;

      arrow.points.push_back(p_start);
      arrow.points.push_back(p_end);

      arrow.scale.x = 0.03; // Shaft diameter
      arrow.scale.y = 0.07; // Head diameter
      arrow.scale.z = 0.40; // Head length      

      arrow.color.r = 0.0f;
      arrow.color.g = 1.0f;
      arrow.color.b = 0.0f;
      arrow.color.a = 0.7f; // Green transparent arrows

      marker_array.markers.push_back(arrow);
    }
  }

  marker_pub_->publish(marker_array);
}

std::optional<geometry_msgs::msg::PoseStamped> GraphPlannerNode::get_current_robot_pose()
{
  try {
    geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform(
      global_frame_, robot_base_frame_, tf2::TimePointZero, std::chrono::seconds(2));

    geometry_msgs::msg::PoseStamped pose;
    pose.header = tf_stamped.header;
    pose.pose.position.x = tf_stamped.transform.translation.x;
    pose.pose.position.y = tf_stamped.transform.translation.y;
    pose.pose.position.z = tf_stamped.transform.translation.z;
    pose.pose.orientation = tf_stamped.transform.rotation;
    return pose;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "TF2 Lookup failure: %s", ex.what());
    return std::nullopt;
  }
}

int GraphPlannerNode::find_nearest_vertex(const geometry_msgs::msg::PoseStamped & pose)
{
  int nearest_id = -1;
  double min_dist = std::numeric_limits<double>::max();

  for (const auto & [id, vertex] : vertices_) {
    double dx = vertex.x - pose.pose.position.x;
    double dy = vertex.y - pose.pose.position.y;
    double dist = std::hypot(dx, dy);

    if (dist < min_dist) {
      min_dist = dist;
      nearest_id = id;
    }
  }

  return nearest_id;
}

std::vector<int> GraphPlannerNode::compute_dijkstra_path(int start_id, int target_id)
{
  std::unordered_map<int, double> dist;
  std::unordered_map<int, int> parent;

  for (const auto & [id, _] : vertices_) {
    dist[id] = std::numeric_limits<double>::max();
  }

  // Priority Queue: pair<distance, vertex_id>
  using Element = std::pair<double, int>;
  std::priority_queue<Element, std::vector<Element>, std::greater<Element>> pq;

  dist[start_id] = 0.0;
  pq.push({0.0, start_id});

  while (!pq.empty()) {
    auto [current_dist, u] = pq.top();
    pq.pop();

    if (u == target_id) break;
    if (current_dist > dist[u]) continue;

    if (!adjacency_list_.count(u)) continue;

    for (const auto & edge : adjacency_list_[u]) {
      double new_dist = current_dist + edge.weight;
      if (new_dist < dist[edge.target_id]) {
        dist[edge.target_id] = new_dist;
        parent[edge.target_id] = u;
        pq.push({new_dist, edge.target_id});
      }
    }
  }

  std::vector<int> path;
  if (dist[target_id] == std::numeric_limits<double>::max()) {
    RCLCPP_WARN(this->get_logger(), "Target is unreachable.");
    return path; // Target is unreachable
  }

  for (int curr = target_id; parent.count(curr) || curr == start_id; curr = parent[curr]) {
    path.push_back(curr);
    if (curr == start_id) break;
  }

  std::reverse(path.begin(), path.end());
  return path;
}

// rclcpp_action::GoalResponse GraphPlannerNode::handle_goal(
//   const rclcpp_action::GoalUUID &,
//   std::shared_ptr<const NavigateToPose::Goal>)
// {
//   RCLCPP_INFO(this->get_logger(), "Received incoming graph execution goal.");
//   return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
// }

rclcpp_action::GoalResponse GraphPlannerNode::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const NavigateToPose::Goal> goal)
{
  std::string frame = goal->pose.header.frame_id;
  int target_id = -1;

  // Check if target is specified as a node ID string (e.g., "node_5" or "5")
  if (frame.rfind("node_", 0) == 0) {
    target_id = std::stoi(frame.substr(5));
  } else {
    try {
      target_id = std::stoi(frame);
    } catch (...) {
      // Fallback: If frame_id is "map", find vertex nearest to goal.pose.position
      auto pose_stamped = goal->pose;
      target_id = find_nearest_vertex(pose_stamped);
    }
  }

  if (vertices_.find(target_id) == vertices_.end()) {
    RCLCPP_ERROR(get_logger(), "Target node ID %d does not exist in graph!", target_id);
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "Accepted goal for Target Node ID: %d", target_id);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GraphPlannerNode::handle_cancel(
  const std::shared_ptr<GoalHandleNavigateToPose>)
{
  RCLCPP_INFO(this->get_logger(), "Goal cancellation requested.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GraphPlannerNode::handle_accepted(
  const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  std::thread{std::bind(&GraphPlannerNode::execute_plan, this, std::placeholders::_1), goal_handle}.detach();
}

/*
void GraphPlannerNode::execute_plan(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  auto result = std::make_shared<NavigateToPose::Result>();

  auto pose_target = goal_handle->get_goal()->pose;

  auto robot_pose = get_current_robot_pose();
  if (!robot_pose) {
    RCLCPP_ERROR(this->get_logger(), "Could not determine robot position!");
    goal_handle->abort(result);
    return;
  }

  int start_vertex = find_nearest_vertex(*robot_pose);
  int target_vertex = find_nearest_vertex(pose_target);

  if (start_vertex == -1 || target_vertex == -1) {
    RCLCPP_ERROR(this->get_logger(), "Invalid start or target vertex ID.");
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Planning path: Start node %d -> Target node %d",
    start_vertex, target_vertex);

  auto vertex_path = compute_dijkstra_path(start_vertex, target_vertex);
  if (vertex_path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Unreachable goal or path calculation failed!");
    goal_handle->abort(result);
    return;
  }

  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = global_frame_;
  path_msg.header.stamp = this->now();

  for (int id : vertex_path) {
    const auto & vp = vertices_[id];

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = this->now();
    pose.pose.position.x = vp.x;
    pose.pose.position.y = vp.y;

    tf2::Quaternion q;
    q.setRPY(0, 0, vp.yaw);
    pose.pose.orientation = tf2::toMsg(q);

    waypoints.push_back(pose);
    path_msg.poses.push_back(pose);
  }

  path_pub_->publish(path_msg);
  send_nav2_goal(waypoints, goal_handle);
}
*/

void GraphPlannerNode::execute_plan(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  std::string frame = goal->pose.header.frame_id;
  int target_id = -1;

  if (frame.rfind("node_", 0) == 0) {
    target_id = std::stoi(frame.substr(5));
  } else {
    try {
      target_id = std::stoi(frame);
    } catch (...) {
      target_id = find_nearest_vertex(goal->pose);
    }
  }

  // 1. Get current robot pose from TF
  auto robot_pose_opt = get_current_robot_pose();
  if (!robot_pose_opt.has_value()) {
    RCLCPP_ERROR(get_logger(), "Could not get current robot pose from TF!");
    auto result = std::make_shared<NavigateToPose::Result>();
    goal_handle->abort(result);
    return;
  }

  // 2. Find nearest starting node
  int start_id = find_nearest_vertex(robot_pose_opt.value());
  RCLCPP_INFO(get_logger(), "Planning path from Start Node %d to Target Node %d", start_id, target_id);

  // 3. Compute shortest path via Dijkstra
  std::vector<int> path_node_ids = compute_dijkstra_path(start_id, target_id);

  if (path_node_ids.empty()) {
    RCLCPP_ERROR(get_logger(), "No path found between node %d and node %d!", start_id, target_id);
    auto result = std::make_shared<NavigateToPose::Result>();
    goal_handle->abort(result);
    return;
  }

  // 4. Convert graph path nodes to PoseStamped waypoints for Nav2
  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = global_frame_;
  path_msg.header.stamp = now();

  for (int node_id : path_node_ids) {
    const auto & v = vertices_[node_id];

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = now();
    pose.pose.position.x = v.x;
    pose.pose.position.y = v.y;
    pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, v.yaw);
    pose.pose.orientation = tf2::toMsg(q);

    waypoints.push_back(pose);
    path_msg.poses.push_back(pose);
  }

  // Publish visualization path on /graph_planner/path
  path_pub_->publish(path_msg);

  // 5. Forward waypoints to Nav2 NavigateThroughPoses
  send_nav2_goal(waypoints, goal_handle);
}

void GraphPlannerNode::send_nav2_goal(
  const std::vector<geometry_msgs::msg::PoseStamped> & waypoints,
  std::shared_ptr<GoalHandleNavigateToPose> graph_goal_handle)
{
  if (!nav2_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Nav2 Action Server unreachable!");
    graph_goal_handle->abort(std::make_shared<NavigateToPose::Result>());
    return;
  }

  auto nav2_goal = NavigateThroughPoses::Goal();
  nav2_goal.poses = waypoints;

  auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();

  send_goal_options.result_callback =
    [this, graph_goal_handle](const GoalHandleNavThrough::WrappedResult & result) {
      auto action_res = std::make_shared<NavigateToPose::Result>();
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Nav2 successfully traversed graph poses.");
          graph_goal_handle->succeed(action_res);
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Nav2 aborted traversal execution.");
          graph_goal_handle->abort(action_res);
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(this->get_logger(), "Nav2 execution canceled.");
          graph_goal_handle->canceled(action_res);
          break;
        default:
          graph_goal_handle->abort(action_res);
          break;
      }
    };

  nav2_action_client_->async_send_goal(nav2_goal, send_goal_options);
}

}  // namespace graph_planner