#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <iostream>
#include <nav2_msgs/msg/behavior_tree_log.hpp>
#include <nav2_msgs/msg/behavior_tree_status_change.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/int32.hpp>
#include <thread>
#include <vector>

#include "lifecycle_msgs/srv/change_state.hpp"
#define MAX 99999

#include "JunctionMonitor.h"

TCAS::JunctionMonitor obj;
std::vector<std::future<void>> results;

rclcpp::Node::SharedPtr node = nullptr;
std::array<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr, MAX>
    goal_pub;

std::vector<nav_msgs::msg::Path> initial_path(MAX);
std::vector<geometry_msgs::msg::PoseStamped> initial_goal(MAX);

std::vector<geometry_msgs::msg::PoseStamped> current_pose(MAX);
std::vector<geometry_msgs::msg::PoseStamped> previous_pose(MAX);

std::vector<std::string> goal_topics_;

float offset[MAX];
bool first_goal_path[MAX];
bool first_goal[MAX];

bool deactivateRobot[MAX];
bool activateRobot[MAX];

float front_dist[MAX], bottom_dist[MAX], left_dist[MAX], right_dist[MAX];

void sendOffset(int robot_id) {
  int i[MAX];
  i[robot_id] = 0;
  previous_pose[robot_id] = initial_path[robot_id].poses[0];

  std::string goal_topic = goal_topics_[robot_id];

  goal_pub[robot_id] =
      node->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);

  std::cout << initial_path[robot_id].poses.size() << std::endl;

  for (const auto& pose_stamped : initial_path[robot_id].poses) {
    i[robot_id] += 1;
    if (i[robot_id] == 7) {
      std::this_thread::sleep_for(std::chrono::seconds(1));

      std::vector<geometry_msgs::msg::PoseStamped> modified_pose(MAX);
      modified_pose[robot_id] = pose_stamped;
      current_pose[robot_id] = modified_pose[robot_id];

      float dx[MAX];
      dx[robot_id] = previous_pose[robot_id].pose.position.x -
                     current_pose[robot_id].pose.position.x;
      float dy[MAX];
      dy[robot_id] = previous_pose[robot_id].pose.position.y -
                     current_pose[robot_id].pose.position.y;

      if (dy[robot_id] <= 0.1 && dy[robot_id] >= -0.1) {
        modified_pose[robot_id].pose.position.y += offset[robot_id];
        if (dx[robot_id] > 0) {
          offset[robot_id] = obj.robot_radius;
          if (offset[robot_id] > left_dist[robot_id]) {
            offset[robot_id] = -1 * (left_dist[robot_id] - 0.1);
          }

          else {
            offset[robot_id] = -1 * offset[robot_id];
          }

          modified_pose[robot_id].pose.orientation.z = 1.0;
        }

        else if (dx[robot_id] < 0) {
          offset[robot_id] = obj.robot_radius;
          if (offset[robot_id] > left_dist[robot_id]) {
            offset[robot_id] = (left_dist[robot_id] - 0.1);
          }
          modified_pose[robot_id].pose.orientation.z = 0.0;
        }
      }

      else if (dx[robot_id] <= 0.1 && dx[robot_id] >= -0.1) {
        modified_pose[robot_id].pose.position.x += offset[robot_id];
        if (dy[robot_id] > 0) {
          offset[robot_id] = obj.robot_radius;
          if (offset[robot_id] > left_dist[robot_id]) {
            offset[robot_id] = (left_dist[robot_id] - 0.1);
          }
          // modified_pose[robot_id].pose.orientation.z = 0.0;
        }

        else if (dy[robot_id] < 0) {
          offset[robot_id] = obj.robot_radius;
          if (offset[robot_id] > left_dist[robot_id]) {
            offset[robot_id] = -1 * (left_dist[robot_id] - 0.1);
          }

          else {
            offset[robot_id] = -1 * offset[robot_id];
          }
          // modified_pose[robot_id].pose.orientation.z = 1.0;
        }
      }

      modified_pose[robot_id].header = initial_goal[robot_id].header;
      goal_pub[robot_id]->publish(modified_pose[robot_id]);

      i[robot_id] = 0;
      previous_pose[robot_id] = current_pose[robot_id];
    }
  }

  std::cout << "Done" << std::endl;
  goal_pub[robot_id]->publish(initial_goal[robot_id]);
}

void getInitialPath(int robot_id, const nav_msgs::msg::Path::SharedPtr plan) {
  if (!first_goal_path[robot_id]) {
    initial_path[robot_id] = *plan;
    first_goal_path[robot_id] = true;

    results.push_back(std::async(std::launch::async, sendOffset, robot_id));
  }
}

void getInitialGoal(int robot_id,
                    const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
  if (!first_goal[robot_id]) {
    initial_goal[robot_id] = *pose;
    first_goal[robot_id] = true;
  }
}

void robotDeactivated(const std_msgs::msg::Int32::SharedPtr msg) {
  int robot_id = msg->data;
  deactivateRobot[robot_id] = true;
}

void robotActivated(const std_msgs::msg::Int32::SharedPtr msg) {
  int robot_id = msg->data;
  activateRobot[robot_id] = true;

  if (deactivateRobot[robot_id]) {
    if (activateRobot[robot_id]) {
      std::cout << "Activating..." << std::endl;
      deactivateRobot[robot_id] = false;
      activateRobot[robot_id] = false;

      std::string goal_topic = goal_topics_[robot_id];

      goal_pub[robot_id] =
          node->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic,
                                                                  10);

      goal_pub[robot_id]->publish(initial_goal[robot_id]);
    }
  }
}

void getGoalRecoveryStatus(
    int robot_id, const nav2_msgs::msg::BehaviorTreeLog::SharedPtr status) {
  for (const nav2_msgs::msg::BehaviorTreeStatusChange each_status :
       status->event_log) {
    if (each_status.node_name == "NavigateRecovery") {
      if (each_status.current_status == "SUCCESS") {
        first_goal[robot_id] = false;
        first_goal_path[robot_id] = false;
        offset[robot_id] = 0.0;
      }
    }
  }
}

void getScanData(int robot_id,
                 const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  if (!first_goal_path[robot_id]) {
    if (!std::isinf(scan->ranges[0])) {
      front_dist[robot_id] = scan->ranges[0];
    }
    if (!std::isinf(scan->ranges[90])) {
      left_dist[robot_id] = scan->ranges[90];
    }
    if (!std::isinf(scan->ranges[180])) {
      bottom_dist[robot_id] = scan->ranges[180];
    }
    if (!std::isinf(scan->ranges[270])) {
      right_dist[robot_id] = scan->ranges[270];
    }
  }
}

void JunctionMonitorActivate(const std_msgs::msg::Int32::SharedPtr msg) {
  int robot_id = msg->data;

  std::string goal_topic = goal_topics_[robot_id];

  goal_pub[robot_id] =
      node->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);

  first_goal[robot_id] = false;
  first_goal_path[robot_id] = false;
  offset[robot_id] = 0.0;
  goal_pub[robot_id]->publish(initial_goal[robot_id]);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("lane_modifier");

  node->declare_parameter("no_of_robots", 0);
  obj.no_of_robots = node->get_parameter("no_of_robots").as_int();
  std::cout << "No of robots: " << obj.no_of_robots << std::endl;

  std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> path_sub;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>
      goal_sub;
  std::vector<rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr>
      status_sub;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr>
      scan_sub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr activity_sub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr deactivity_sub;

  std::string radius_param = "robot_radius";

  node->declare_parameter(radius_param, 0.4);
  obj.robot_radius = node->get_parameter(radius_param).as_double();

  activity_sub = node->create_subscription<std_msgs::msg::Int32>(
      "/activate_robot", 10, robotActivated);

  deactivity_sub = node->create_subscription<std_msgs::msg::Int32>(
      "/deactivate_robot", 10, robotDeactivated);

  for (int i = 0; i < obj.no_of_robots; i++) {
    std::string robot_namespace = "robot" + std::to_string(i);

    node->declare_parameter(robot_namespace, "");
    auto robot_name = node->get_parameter(robot_namespace).as_string();

    std::string path_topic = robot_name + "/plan";
    std::string scan_topic = robot_name + "/scan";
    std::string status_topic = robot_name + "/behavior_tree_log";
    std::string goal_topic = robot_name + "/goal_pose";

    goal_topics_.push_back(goal_topic);

    auto callback = [node, i](const nav_msgs::msg::Path::SharedPtr msg) {
      getInitialPath(i, msg);
    };

    path_sub.push_back(node->create_subscription<nav_msgs::msg::Path>(
        path_topic, 10, callback));

    auto callback2 = [node,
                      i](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      getScanData(i, msg);
    };

    scan_sub.push_back(node->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, 10, callback2));

    auto callback3 = [node,
                      i](const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg) {
      getGoalRecoveryStatus(i, msg);
    };

    status_sub.push_back(
        node->create_subscription<nav2_msgs::msg::BehaviorTreeLog>(
            status_topic, 10, callback3));

    auto callback4 = [node,
                      i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      getInitialGoal(i, msg);
    };

    goal_sub.push_back(
        node->create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_topic, 10, callback4));
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}