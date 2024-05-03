// Junction Monitoring Node for the Traffic Management System
// multiple junctions
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/msg/behavior_tree_log.hpp>
#include <nav2_msgs/msg/behavior_tree_status_change.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/int32.hpp>

#include "JunctionMonitor.h"
#include "lifecycle_msgs/srv/change_state.hpp"

rclcpp::Node::SharedPtr node = nullptr;
TCAS::JunctionMonitor obj;

void publishStopCommand(int*, int);

void robotPositionCallback(
    int robot_id,
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  float robot_x = msg->pose.pose.position.x;
  float robot_y = msg->pose.pose.position.y;
  float robot_z = msg->pose.pose.position.z;

  TCAS::Robot robot_info;
  int state[MAX];

  robot_info.id = robot_id;
  robot_info.position(0) = robot_x;
  robot_info.position(1) = robot_y;
  robot_info.position(2) = robot_z;

  for (size_t j = 0; j < obj.junctions_.size(); j++) {
    state[j] = obj.junctionMonitor(robot_info, j);
  }

  publishStopCommand(state, robot_id);
}

void publishStopCommand(int state[], int robot_id) {
  geometry_msgs::msg::Twist stop_cmd_vel;
  stop_cmd_vel.linear.x = 0.0;
  stop_cmd_vel.angular.z = 0.0;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr activity_pub;
  activity_pub =
      node->create_publisher<std_msgs::msg::Int32>("/activate_robot", 10);

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr deactivate_pub;
  deactivate_pub =
      node->create_publisher<std_msgs::msg::Int32>("/deactivate_robot", 10);

  std_msgs::msg::Int32 message;

  for (size_t j = 0; j < obj.junctions_.size(); j++) {
    if (state[j] == 2) {
      if (!obj.nav_srv_.empty()) {
        auto srv_topic = obj.nav_srv_[robot_id];

        auto client =
            node->create_client<lifecycle_msgs::srv::ChangeState>(srv_topic);

        if (!client->wait_for_service(std::chrono::seconds(5))) {
          RCLCPP_ERROR(node->get_logger(), "Service not available");
        }

        auto request =
            std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();

        request->transition.id = 4;
        auto result = client->async_send_request(request);

        message.data = robot_id;
        deactivate_pub->publish(message);
      }
    }

    if (state[j] == 3) {
      if (!obj.robot_queue.empty()) {
        std::cout << "Robots in the queue are: ";

        for (int id : obj.robot_queue) {
          std::cout << id << " ";
        }

        std::cout << std::endl;

        int resume_robot = obj.robot_queue.front();
        obj.robot_queue.erase(obj.robot_queue.begin());

        std::cout << "Next to resume -> Robot-" << resume_robot << std::endl;

        auto srv_topic = obj.nav_srv_[resume_robot];

        auto client =
            node->create_client<lifecycle_msgs::srv::ChangeState>(srv_topic);

        if (!client->wait_for_service(std::chrono::seconds(5))) {
          RCLCPP_ERROR(node->get_logger(), "Service not available");
        }

        auto request =
            std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();

        request->transition.id = 3;
        auto result = client->async_send_request(request);

        message.data = resume_robot;
        activity_pub->publish(message);

        for (int queue_robot_id : obj.robot_queue) {
          if (!obj.nav_srv_.empty()) {
            auto srv_topic = obj.nav_srv_[queue_robot_id];

            auto client = node->create_client<lifecycle_msgs::srv::ChangeState>(
                srv_topic);

            if (!client->wait_for_service(std::chrono::seconds(5))) {
              RCLCPP_ERROR(node->get_logger(), "Service not available");
            }

            auto request =
                std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();

            request->transition.id = 4;
            auto result = client->async_send_request(request);

            message.data = robot_id;
            deactivate_pub->publish(message);
          }
        }
      }
    }
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("junction_monitor");

  node->declare_parameter("no_of_robots", 0);
  auto no_of_robots = node->get_parameter("no_of_robots").as_int();
  std::cout << "No of robots: " << no_of_robots << std::endl;

  std::vector<rclcpp::Subscription<
      geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr>
      robot_position_sub;

  for (int i = 0; i < no_of_robots; i++) {
    std::string robot_namespace = "robot" + std::to_string(i);

    node->declare_parameter(robot_namespace, "");
    auto robot_name = node->get_parameter(robot_namespace).as_string();

    std::string position_topic = robot_name + "/amcl_pose";
    std::string srv_topic = robot_name + "/bt_navigator/change_state";

    obj.nav_srv_.push_back(srv_topic);

    auto callback =
        [node, i](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
                      msg) { robotPositionCallback(i, msg); };

    robot_position_sub.push_back(node->create_subscription<
                                 geometry_msgs::msg::PoseWithCovarianceStamped>(
        position_topic, 10, callback));
  }

  node->declare_parameter("no_of_junctions", 0);
  auto no_of_junctions = node->get_parameter("no_of_junctions").as_int();

  Eigen::VectorXf junction_coordinates(8);

  for (int i = 0; i < no_of_junctions; i++) {
    std::string param_name = "junction_" + std::to_string(i + 1);
    node->declare_parameter(param_name, std::vector<double>(8, -1));
    auto junction = node->get_parameter(param_name).as_double_array();

    std::cout << param_name << std::endl;

    for (int j = 0; j < 8; j++) {
      junction_coordinates(j) = static_cast<double>(junction[j]);
    }
    obj.junctions_.push_back(junction_coordinates);
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
