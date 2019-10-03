// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2018 Simbe Robotics
// Copyright (c) 2019 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Navigation Strategy based on:
// Brock, O. and Oussama K. (1999). High-Speed Navigation Using
// the Global Dynamic Window Approach. IEEE.
// https://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf

#include "nav2_navfn_planner/navfn_planner.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_navfn_planner/navfn.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using namespace std::chrono_literals;
using nav2_util::declare_parameter_if_not_declared;

namespace nav2_navfn_planner
{

NavfnPlanner::NavfnPlanner()
: tf_(nullptr), costmap_(nullptr)
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare this node's parameters
  declare_parameter("tolerance", rclcpp::ParameterValue(0.0));
  declare_parameter("use_astar", rclcpp::ParameterValue(false));

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
}

NavfnPlanner::~NavfnPlanner()
{
  RCLCPP_INFO(node_->get_logger(), "Destroying plugin %s", name_.c_str());
}

void
NavfnPlanner::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  tf_ = tf;
  name_ = name;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  RCLCPP_INFO(node_->get_logger(), "Configuring plugin %s", name_.c_str());

  // Initialize parameters
  // Declare this plugin's parameters
  declare_parameter_if_not_declared(node_, "tolerance", rclcpp::ParameterValue(0.0));
  node_->get_parameter("tolerance", tolerance_);
  declare_parameter_if_not_declared(node_, "use_astar", rclcpp::ParameterValue(false));
  node_->get_parameter("use_astar", use_astar_);
  declare_parameter_if_not_declared(node_, "allow_unknown", rclcpp::ParameterValue(true));
  node_->get_parameter("allow_unknown", allow_unknown_);

  // Create a planner based on the new costmap size
  if (isPlannerOutOfDate()) {
    current_costmap_size_[0] = costmap_.metadata.size_x;
    current_costmap_size_[1] = costmap_.metadata.size_y;
    planner_ = std::make_unique<NavFn>(costmap_.metadata.size_x, costmap_.metadata.size_y);
  }

  // Initialize pubs & subs
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);

  auto node = shared_from_this();

  // Create the action server that we implement with our navigateToPose method
  action_server_ = std::make_unique<ActionServer>(rclcpp_node_, "ComputePathToPose",
      std::bind(&NavfnPlanner::computePathToPose, this));

  return nav2_util::CallbackReturn::SUCCESS;
}

void
NavfnPlanner::activate()
{
  RCLCPP_INFO(get_logger(), "Activating");

  plan_publisher_->on_activate();
  action_server_->activate();

  return nav2_util::CallbackReturn::SUCCESS;
}

void
NavfnPlanner::deactivate()
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();
  plan_publisher_->on_deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

void
NavfnPlanner::cleanup()
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_.reset();
  plan_publisher_.reset();
  planner_.reset();
  tf_listener_.reset();
  tf_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav_msgs::msg::Path NavfnPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node entered error state");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
NavfnPlanner::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void
NavfnPlanner::computePathToPose()
{
  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_->get_current_goal();
  auto result = std::make_shared<nav2_msgs::action::ComputePathToPose::Result>();

  try {
    if (action_server_ == nullptr) {
      RCLCPP_DEBUG(get_logger(), "Action server unavailable. Stopping.");
      return;
    }

    if (!action_server_->is_server_active()) {
      RCLCPP_DEBUG(get_logger(), "Action server is inactive. Stopping.");
      return;
    }

    if (action_server_->is_cancel_requested()) {
      RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
      action_server_->terminate_goals();
      return;
    }

    // Get the current costmap
    getCostmap(costmap_);
    RCLCPP_DEBUG(get_logger(), "Costmap size: %d,%d",
      costmap_.metadata.size_x, costmap_.metadata.size_y);

    geometry_msgs::msg::PoseStamped start;
    if (!nav2_util::getCurrentPose(start, *tf_)) {
      return;
    }

    // Update planner based on the new costmap size
    if (isPlannerOutOfDate()) {
      current_costmap_size_[0] = costmap_.metadata.size_x;
      current_costmap_size_[1] = costmap_.metadata.size_y;
      planner_->setNavArr(costmap_.metadata.size_x, costmap_.metadata.size_y);
    }

    if (action_server_->is_preempt_requested()) {
      RCLCPP_INFO(get_logger(), "Preempting the goal pose.");
      goal = action_server_->accept_pending_goal();
    }

    RCLCPP_DEBUG(get_logger(), "Attempting to a find path from (%.2f, %.2f) to "
      "(%.2f, %.2f).", start.pose.position.x, start.pose.position.y,
      goal->pose.pose.position.x, goal->pose.pose.position.y);

    // Make the plan for the provided goal pose
    bool foundPath = makePlan(start.pose, goal->pose.pose, tolerance_, result->path);

  nav_msgs::msg::Path path;

    RCLCPP_DEBUG(get_logger(), "Found valid path of size %u", result->path.poses.size());

    // Publish the plan for visualization purposes
    RCLCPP_DEBUG(get_logger(), "Publishing the valid path");
    publishPlan(result->path);

    // TODO(orduno): Enable potential visualization

    RCLCPP_DEBUG(get_logger(),
      "Successfully computed a path to (%.2f, %.2f) with tolerance %.2f",
      goal->pose.pose.position.x, goal->pose.pose.position.y, tolerance_);
    action_server_->succeeded_current(result);
    return;
  } catch (std::exception & ex) {
    RCLCPP_WARN(get_logger(), "Plan calculation to (%.2f, %.2f) failed: \"%s\"",
      goal->pose.pose.position.x, goal->pose.pose.position.y, ex.what());

    // TODO(orduno): provide information about fail error to parent task,
    //               for example: couldn't get costmap update
    action_server_->terminate_goals();
    return;
  } catch (...) {
    RCLCPP_WARN(get_logger(), "Plan calculation failed");

    // TODO(orduno): provide information about the failure to the parent task,
    //               for example: couldn't get costmap update
    action_server_->terminate_goals();
    return;
  }
  return path;
}

bool
NavfnPlanner::isPlannerOutOfDate()
{
  if (!planner_.get() ||
    planner_->nx != static_cast<int>(costmap_->getSizeInCellsX()) ||
    planner_->ny != static_cast<int>(costmap_->getSizeInCellsY()))
  {
    return true;
  }
  return false;
}

bool
NavfnPlanner::makePlan(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal, double tolerance,
  nav_msgs::msg::Path & plan)
{
  // clear the plan, just in case
  plan.poses.clear();

  // TODO(orduno): add checks for start and goal reference frame -- should be in global frame

  double wx = start.position.x;
  double wy = start.position.y;

  RCLCPP_DEBUG(node_->get_logger(), "Making plan from (%.2f,%.2f) to (%.2f,%.2f)",
    start.position.x, start.position.y, goal.position.x, goal.position.y);

  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Cannot create a plan: the robot's start position is off the global"
      " costmap. Planning will always fail, are you sure"
      " the robot has been properly localized?");
    return false;
  }

  // clear the starting cell within the costmap because we know it can't be an obstacle
  clearRobotCell(mx, my);

  // make sure to resize the underlying array that Navfn uses
  planner_->setNavArr(costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY());

  planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

  int map_start[2];
  map_start[0] = mx;
  map_start[1] = my;

  wx = goal.position.x;
  wy = goal.position.y;

  if (!worldToMap(wx, wy, mx, my)) {
    RCLCPP_WARN(node_->get_logger(),
      "The goal sent to the planner is off the global costmap."
      " Planning will always fail to this goal.");
    return false;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  // TODO(orduno): Explain why we are providing 'map_goal' to setStart().
  //               Same for setGoal, seems reversed. Computing backwards?

  planner_->setStart(map_goal);
  planner_->setGoal(map_start);
  if (use_astar_) {
    planner_->calcNavFnAstar();
  } else {
    planner_->calcNavFnDijkstra(true);
  }

  double resolution = costmap_->getResolution();
  geometry_msgs::msg::Pose p, best_pose;
  p = goal;

  bool found_legal = false;
  double best_sdist = std::numeric_limits<double>::max();

  p.position.y = goal.position.y - tolerance;

  while (p.position.y <= goal.position.y + tolerance) {
    p.position.x = goal.position.x - tolerance;
    while (p.position.x <= goal.position.x + tolerance) {
      double potential = getPointPotential(p.position);
      double sdist = squared_distance(p, goal);
      if (potential < POT_HIGH && sdist < best_sdist) {
        best_sdist = sdist;
        best_pose = p;
        found_legal = true;
      }
      p.position.x += resolution;
    }
    p.position.y += resolution;
  }

  if (found_legal) {
    // extract the plan
    if (getPlanFromPotential(best_pose, plan)) {
      smoothApproachToGoal(best_pose, plan);
    } else {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to create a plan from potential when a legal"
        " potential was found. This shouldn't happen.");
    }
  }

  return !plan.poses.empty();
}

void
NavfnPlanner::smoothApproachToGoal(
  const geometry_msgs::msg::Pose & goal,
  nav_msgs::msg::Path & plan)
{
  // Replace the last pose of the computed path if it's actually further away
  // to the second to last pose than the goal pose.

  auto second_to_last_pose = plan.poses.end()[-2];
  auto last_pose = plan.poses.back();
  if (
    squared_distance(last_pose.pose, second_to_last_pose.pose) >
    squared_distance(goal, second_to_last_pose.pose))
  {
    plan.poses.back().pose = goal;
  } else {
    geometry_msgs::msg::PoseStamped goal_copy;
    goal_copy.pose = goal;
    plan.poses.push_back(goal_copy);
  }
}

bool
NavfnPlanner::computePotential(const geometry_msgs::msg::Point & world_point)
{
  // make sure to resize the underlying array that Navfn uses
  planner_->setNavArr(costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY());

  planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

  unsigned int mx, my;
  if (!worldToMap(world_point.x, world_point.y, mx, my)) {
    return false;
  }

  int map_start[2];
  map_start[0] = 0;
  map_start[1] = 0;

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_start);
  planner_->setGoal(map_goal);

  if (use_astar_) {
    return planner_->calcNavFnAstar();
  }

  return planner_->calcNavFnDijkstra();
}

bool
NavfnPlanner::getPlanFromPotential(
  const geometry_msgs::msg::Pose & goal,
  nav_msgs::msg::Path & plan)
{
  // clear the plan, just in case
  plan.poses.clear();

  // Goal should be in global frame
  double wx = goal.position.x;
  double wy = goal.position.y;

  // the potential has already been computed, so we won't update our copy of the costmap
  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "The goal sent to the navfn planner is off the global costmap."
      " Planning will always fail to this goal.");
    return false;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_goal);

  planner_->calcPath(costmap_->getSizeInCellsX() * 4);

  // extract the plan
  float * x = planner_->getPathX();
  float * y = planner_->getPathY();
  int len = planner_->getPathLen();

  plan.header.stamp = node_->now();
  plan.header.frame_id = global_frame_;

  for (int i = len - 1; i >= 0; --i) {
    // convert the plan to world coordinates
    double world_x, world_y;
    mapToWorld(x[i], y[i], world_x, world_y);

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.poses.push_back(pose);
  }

  return !plan.poses.empty();
}

double
NavfnPlanner::getPointPotential(const geometry_msgs::msg::Point & world_point)
{
  unsigned int mx, my;
  if (!worldToMap(world_point.x, world_point.y, mx, my)) {
    return std::numeric_limits<double>::max();
  }

  unsigned int index = my * planner_->nx + mx;
  return planner_->potarr[index];
}

bool
NavfnPlanner::validPointPotential(const geometry_msgs::msg::Point & world_point)
{
  return validPointPotential(world_point, tolerance_);
}

bool
NavfnPlanner::validPointPotential(
  const geometry_msgs::msg::Point & world_point, double tolerance)
{
  const double resolution = costmap_->getResolution();

  geometry_msgs::msg::Point p = world_point;
  p.y = world_point.y - tolerance;

  while (p.y <= world_point.y + tolerance) {
    p.x = world_point.x - tolerance;
    while (p.x <= world_point.x + tolerance) {
      double potential = getPointPotential(p);
      if (potential < POT_HIGH) {
        return true;
      }
      p.x += resolution;
    }
    p.y += resolution;
  }

  return false;
}

bool
NavfnPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
  if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) {
    RCLCPP_ERROR(node_->get_logger(), "worldToMap failed: wx,wy: %f,%f, "
      "size_x,size_y: %d,%d", wx, wy,
      costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    return false;
  }

  mx = static_cast<int>(
    std::round((wx - costmap_->getOriginX()) / costmap_->getResolution()));
  my = static_cast<int>(
    std::round((wy - costmap_->getOriginY()) / costmap_->getResolution()));

  if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) {
    return true;
  }

  RCLCPP_ERROR(node_->get_logger(), "worldToMap failed: mx,my: %d,%d, size_x,size_y: %d,%d", mx, my,
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  return false;
}

void
NavfnPlanner::mapToWorld(double mx, double my, double & wx, double & wy)
{
  wx = costmap_->getOriginX() + mx * costmap_->getResolution();
  wy = costmap_->getOriginY() + my * costmap_->getResolution();
}

void
NavfnPlanner::clearRobotCell(unsigned int mx, unsigned int my)
{
  // TODO(orduno): check usage of this function, might instead be a request to
  //               world_model / map server
  unsigned int index = my * costmap_.metadata.size_x + mx;
  costmap_.data[index] = nav2_util::Costmap::free_space;
}

void
NavfnPlanner::getCostmap(
  nav2_msgs::msg::Costmap & costmap,
  const std::string /*layer*/)
{
  // TODO(orduno): explicitly provide specifications for costmap using the costmap on the request,
  //               including master (aggregate) layer

  auto request = std::make_shared<nav2_util::CostmapServiceClient::CostmapServiceRequest>();
  request->specs.resolution = 1.0;

  auto result = costmap_client_.invoke(request, 5s);
  costmap = result.get()->map;
}

void
NavfnPlanner::printCostmap(const nav2_msgs::msg::Costmap & costmap)
{
  std::cout << "Costmap" << std::endl;
  std::cout << "  size:       " <<
    costmap.metadata.size_x << "," << costmap.metadata.size_x << std::endl;
  std::cout << "  origin:     " <<
    costmap.metadata.origin.position.x << "," << costmap.metadata.origin.position.y << std::endl;
  std::cout << "  resolution: " << costmap.metadata.resolution << std::endl;
  std::cout << "  data:       " <<
    "(" << costmap.data.size() << " cells)" << std::endl << "    ";

  const char separator = ' ';
  const int valueWidth = 4;

  unsigned int index = 0;
  for (unsigned int h = 0; h < costmap.metadata.size_y; ++h) {
    for (unsigned int w = 0; w < costmap.metadata.size_x; ++w) {
      std::cout << std::left << std::setw(valueWidth) << std::setfill(separator) <<
        static_cast<unsigned int>(costmap.data[index]);
      index++;
    }
    std::cout << std::endl << "    ";
  }
  std::cout << std::endl;
}

void
NavfnPlanner::publishPlan(const nav2_msgs::msg::Path & path)
{
  // Publish as a nav1 path msg
  nav_msgs::msg::Path rviz_path;

  rviz_path.header = path.header;
  rviz_path.poses.resize(path.poses.size());

  // Assuming path is already provided in world coordinates
  for (unsigned int i = 0; i < path.poses.size(); i++) {
    rviz_path.poses[i].header = path.header;
    rviz_path.poses[i].pose = path.poses[i];
  }

  plan_publisher_->publish(rviz_path);
}

}  // namespace nav2_navfn_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_navfn_planner::NavfnPlanner, nav2_core::GlobalPlanner)
