// Junction Monitoring Node for the Traffic Management System

#include <bits/stdc++.h>

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traffic_management_system/msg/robot_details.hpp>
#include <vector>
// multiple junctions
struct Point {
  float x, y;
};

struct Line {
  Point p1, p2;
};

struct JunctionData {
  std::string junction_name;
  int robot_id = -1;
};

class JunctionMonitor : public rclcpp::Node {
 public:
  JunctionMonitor() : Node("junction_monitor") {
    declare_parameter("no_of_robots", 0);
    auto no_of_robots = get_parameter("no_of_robots").as_int();
    std::cout << no_of_robots << std::endl;

    for (int i = 0; i < no_of_robots; i++) {
      std::string sub_topic = "tb3_" + std::to_string(i) + "/odom";

      auto callback = [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
        robotPositionCallback(i, msg);
      };

      robot_position_sub.push_back(
          this->create_subscription<nav_msgs::msg::Odometry>(sub_topic, 10,
                                                             callback));
    }

    robot_details_pub_ =
        this->create_publisher<traffic_management_system::msg::RobotDetails>(
            "/robot_details", 10);

    declare_parameter("no_of_junctions", 0);
    auto no_of_junctions = get_parameter("no_of_junctions").as_int();

    Eigen::VectorXf junction_coordinates(8);

    for (int i = 0; i < no_of_junctions; i++) {
      std::string param_name = "junction_" + std::to_string(i + 1);
      declare_parameter(param_name, std::vector<double>(8, -1));
      auto junction = get_parameter(param_name).as_double_array();

      for (int j = 0; j < 8; j++) {
        junction_coordinates(j) = static_cast<double>(junction[j]);
      }
      junctions_.push_back(junction_coordinates);
    }
  }

 private:
  void robotPositionCallback(int robot_id,
                             const nav_msgs::msg::Odometry::SharedPtr msg) {
    float robot_x = msg->pose.pose.position.x;
    float robot_y = msg->pose.pose.position.y;

    junctionMonitor(robot_id, robot_x, robot_y);
  }

  void junctionMonitor(int robot_id, float robot_x, float robot_y) {
    auto robot_info =
        std::make_shared<traffic_management_system::msg::RobotDetails>();

    int no_of_junctions = junctions_.size();
    junction_data_list.resize(no_of_junctions);

    for (size_t j = 0; j < junctions_.size(); j++) {
      if (isInsideJunctionRegion(robot_x, robot_y, junctions_[j])) {
        if (junction_data_list[j].robot_id == -1) {
          junction_data_list[j].robot_id = robot_id;
          std::cout << "Robot-" << robot_id << " has entered Junction-" << j + 1
                    << std::endl;
        }

        else if (junction_data_list[j].robot_id != -1 &&
                 junction_data_list[j].robot_id != robot_id) {
          int existing_robot_id = junction_data_list[j].robot_id;
          std::cout << "Already a robot-" << existing_robot_id
                    << " is present in Junction-" << j + 1
                    << ". Stopping the current robot-" << robot_id << "."
                    << std::endl;

          manageVelocity(robot_id);
        }
      }

      // Robot is not in a given junction
      else {
        if (junction_data_list[j].robot_id == robot_id) {
          // Robot is not in the junction, so we need to empty the array.
          std::cout << "Robot-" << robot_id << " has exited Junction-" << j + 1
                    << std::endl;
          junction_data_list[j].robot_id = -1;
        }
      }
    }
  }

  void manageVelocity(int robot_id) {
    std::string pub_topic = "tb3_" + std::to_string(robot_id) + "/cmd_vel";

    robot_cmd_vel_subscriber_ =
        this->create_subscription<geometry_msgs::msg::Twist>(
            pub_topic, 10,
            std::bind(&JunctionMonitor::cmdVelCallback, this,
                      std::placeholders::_1));

    robot_cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>(pub_topic, 10);
  }

  void cmdVelCallback(geometry_msgs::msg::Twist::SharedPtr msg) {
    float robot_velocity_linear = msg->linear.x;
    float robot_velocity_angular = msg->angular.z;

    publishStopCommand(robot_velocity_linear, robot_velocity_angular);
  }

  void publishStopCommand(float rv_linear, float rv_angular) {
    // Can be defined in a better way
    float step_size = 0.1;

    float current_linear = rv_linear;
    float current_angular = rv_angular;

    if (current_linear > 0.0) {
      current_linear -= step_size;
      if (current_linear < 0.0) {
        current_linear = 0.0;
      }
    }

    if (current_angular > 0.0) {
      current_angular -= step_size;
      if (current_angular < 0.0) {
        current_angular = 0.0;
      }
    }

    geometry_msgs::msg::Twist stop_cmd_vel;
    stop_cmd_vel.linear.x = current_linear;
    stop_cmd_vel.angular.z = current_angular;

    robot_cmd_vel_publisher_->publish(stop_cmd_vel);
  }

  bool isInsideJunctionRegion(float robot_x, float robot_y,
                              const Eigen::VectorXf& region) {
    int n = region.size() / 2;

    Point p = {robot_x, robot_y};

    Point polygon[] = {{region(0), region(1)},
                       {region(2), region(3)},
                       {region(4), region(5)},
                       {region(6), region(7)}};

    if (checkInside(polygon, n, p)) {
      return true;
    }

    return false;
  }

  bool checkInside(Point polygon[], int n, Point p) {
    if (n < 3) {
      std::cout << "Error, not a polygon!" << std::endl;
      return 0;
    }

    Line exline = {p, {9999, p.y}};
    int count = 0;
    int i = 0;
    do {
      Line side = {polygon[i], polygon[(i + 1) % n]};
      if (isIntersect(side, exline)) {
        if (direction(side.p1, p, side.p2) == 0) {
          return onLine(side, p);
        }
        count++;
      }
      i = (i + 1) % n;
    } while (i != 0);

    return count & 1;
  }

  bool onLine(Line l1, Point p) {
    if (p.x <= std::max(l1.p1.x, l1.p2.x) &&
        p.x <= std::min(l1.p1.x, l1.p2.x) &&
        (p.y <= std::max(l1.p1.y, l1.p2.y) &&
         p.y <= std::min(l1.p1.y, l1.p2.y)))
      return true;

    return false;
  }

  bool isIntersect(Line l1, Line l2) {
    float dir1 = direction(l1.p1, l1.p2, l2.p1);
    float dir2 = direction(l1.p1, l1.p2, l2.p2);
    float dir3 = direction(l2.p1, l2.p2, l1.p1);
    float dir4 = direction(l2.p1, l2.p2, l1.p2);

    if (dir1 != dir2 && dir3 != dir4) return true;

    if (dir1 == 0 && onLine(l1, l2.p1)) return true;

    if (dir2 == 0 && onLine(l1, l2.p2)) return true;

    if (dir3 == 0 && onLine(l2, l1.p1)) return true;

    if (dir4 == 0 && onLine(l2, l1.p2)) return true;

    return false;
  }

  float direction(Point a, Point b, Point c) {
    float val = (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y);

    if (val == 0)

      return 0;

    else if (val < 0)

      return 2;

    return 1;
  }

  std::vector<Eigen::VectorXf> junctions_;
  std::vector<JunctionData> junction_data_list;

  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>
      robot_position_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      junction_coords_sub_;
  rclcpp::Publisher<traffic_management_system::msg::RobotDetails>::SharedPtr
      robot_details_pub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      robot_cmd_vel_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      robot_cmd_vel_subscriber_;
  geometry_msgs::msg::Twist robot_cmd_vel_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JunctionMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
