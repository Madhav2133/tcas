#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traffic_management_system/msg/robot_details.hpp>

class RobotPlanner : public rclcpp::Node {
 public:
  RobotPlanner() : Node("robot_planner") {
    subscription_ =
        this->create_subscription<traffic_management_system::msg::RobotDetails>(
            "/robot_details", 10,
            std::bind(&RobotPlanner::robotDetailsCallback, this,
                      std::placeholders::_1));

    robot1_cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/tb3_0/cmd_vel", 10);

    robot2_cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/tb3_1/cmd_vel", 10);
  }

 private:
  void robotDetailsCallback(
      const traffic_management_system::msg::RobotDetails::SharedPtr msg) {
    std::string robot_name = msg->robot_name;
    bool robot_in_junction = msg->in_junction;

    bool prev_robot_in_junction = robots_status_[robot_name];

    robots_status_[robot_name] = robot_in_junction;

    if (robots_status_["robot_1"] && robots_status_["robot_2"]) {
      if (robot_in_junction != prev_robot_in_junction) {
        if (robot_name == "robot_1") {
          stopRobot("robot_1");
        }

        else if (robot_name == "robot_2") {
          stopRobot("robot_2");
        }
      }
    }
  }

  void stopRobot(const std::string& robot_name) {
    geometry_msgs::msg::Twist stop_cmd_vel;
    stop_cmd_vel.linear.x = 0.0;
    stop_cmd_vel.angular.z = 0.0;

    RCLCPP_INFO(this->get_logger(), "%s cmd_vel: linear = %f, angular = %f",
                robot_name.c_str(), stop_cmd_vel.linear.x,
                stop_cmd_vel.angular.z);

    if (robot_name == "robot_1") {
      robot1_cmd_vel_publisher_->publish(stop_cmd_vel);
    }

    else if (robot_name == "robot_2") {
      robot2_cmd_vel_publisher_->publish(stop_cmd_vel);
    }
  }

  rclcpp::Subscription<traffic_management_system::msg::RobotDetails>::SharedPtr
      subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      robot1_cmd_vel_publisher_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      robot2_cmd_vel_publisher_;

  std::unordered_map<std::string, bool> robots_status_;

  geometry_msgs::msg::Twist robot_1_cmd_vel_;
  geometry_msgs::msg::Twist robot_2_cmd_vel_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// One case: If both are entering at the same time?