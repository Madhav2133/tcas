#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/msg/behavior_tree_log.hpp>
#include <nav2_msgs/msg/behavior_tree_status_change.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <thread>

bool first_goal_path = false;
nav_msgs::msg::Path initial_path;

bool first_goal = false;
geometry_msgs::msg::PoseStamped initial_goal;

geometry_msgs::msg::PoseStamped current_pose;
geometry_msgs::msg::PoseStamped previous_pose;

float offset = 0.0;
float front_dist, bottom_dist, left_dist, right_dist;

class PlanModifierNode : public rclcpp::Node {
 public:
  PlanModifierNode() : Node("plan_modifier_node") {
    path_sub = this->create_subscription<nav_msgs::msg::Path>(
        "/tb3_0/plan", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) {
          getInitialPath(msg);
        });

    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/tb3_0/scan", 10,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          getScanData(msg);
        });

    status_sub = this->create_subscription<nav2_msgs::msg::BehaviorTreeLog>(
        "/tb3_0/behavior_tree_log", 10,
        [this](const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg) {
          getGoalRecoveryStatus(msg);
        });

    goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/tb3_0/goal_pose", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          getInitialGoal(msg);
        });

    goal_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/tb3_0/goal_pose", 10);
  }

 private:
  void sendOffset() {
    int i = 0;

    previous_pose = initial_path.poses[0];

    for (const auto& pose_stamped : initial_path.poses) {
      i += 1;
      if (i == 7) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "here" << std::endl;

        geometry_msgs::msg::PoseStamped modified_pose = pose_stamped;
        current_pose = modified_pose;

        float dx = previous_pose.pose.position.x - current_pose.pose.position.x;
        float dy = previous_pose.pose.position.y - current_pose.pose.position.y;

        if (dy <= 0.02 && dy >= -0.02) {
          modified_pose.pose.position.y += offset;
          if (dx > 0) {
            offset = 0.7;
            if (offset > left_dist) {
              std::cout << "yes: " << left_dist << std::endl;
              offset = -1 * (left_dist - 0.1);
            }

            else {
              offset = -1 * offset;
            }

            modified_pose.pose.orientation.z = 1.0;
            std::cout << "Moving up" << std::endl;
          }

          else if (dx < 0) {
            offset = 0.7;
            if (offset > left_dist) {
              std::cout << "yes: " << left_dist << std::endl;
              offset = (left_dist - 0.1);
            }
            modified_pose.pose.orientation.z = 0.0;
            std::cout << "Moving down" << std::endl;
          }
        }

        else if (dx <= 0.02 && dx >= -0.02) {
          modified_pose.pose.position.x += offset;
          if (dy > 0) {
            offset = 0.7;
            if (offset > left_dist) {
              std::cout << "yes: " << left_dist << std::endl;
              offset = (left_dist - 0.1);
            }
            std::cout << "Moving down" << std::endl;
          }

          else if (dy < 0) {
            offset = 0.7;
            if (offset > left_dist) {
              std::cout << "yes: " << left_dist << std::endl;
              offset = -1 * (left_dist - 0.1);
            }

            else {
              offset = -1 * offset;
            }

            std::cout << "Moving right" << std::endl;
          }
        }

        std::cout << "Previous - x: " << previous_pose.pose.position.x
                  << " y: " << previous_pose.pose.position.y;

        std::cout << "   Current - x: " << current_pose.pose.position.x
                  << " y: " << current_pose.pose.position.y << std::endl;

        goal_pub->publish(modified_pose);

        i = 0;
        previous_pose = current_pose;
      }
    }

    std::cout << "done" << std::endl;

    goal_pub->publish(initial_goal);
  }

  void getInitialPath(const nav_msgs::msg::Path::SharedPtr plan) {
    if (!first_goal_path) {
      std::cout << "hi" << std::endl;
      initial_path = *plan;
      first_goal_path = true;

      sendOffset();
    }
  }

  void getInitialGoal(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
    if (!first_goal) {
      std::cout << "hello" << std::endl;
      initial_goal = *pose;
      first_goal = true;
    }
  }

  void getGoalRecoveryStatus(
      const nav2_msgs::msg::BehaviorTreeLog::SharedPtr status) {
    for (const nav2_msgs::msg::BehaviorTreeStatusChange each_status :
         status->event_log) {
      if (each_status.node_name == "NavigateRecovery") {
        if (each_status.current_status == "SUCCESS") {
          first_goal = false;
          first_goal_path = false;
          offset = 0.0;
        }
      }
    }
  }

  void getScanData(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    if (!std::isinf(scan->ranges[0])) {
      front_dist = scan->ranges[0];
    }
    if (!std::isinf(scan->ranges[90])) {
      left_dist = scan->ranges[90];
    }
    if (!std::isinf(scan->ranges[180])) {
      bottom_dist = scan->ranges[180];
    }
    if (!std::isinf(scan->ranges[270])) {
      right_dist = scan->ranges[270];
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
  rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr status_sub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanModifierNode>());
  rclcpp::shutdown();
  return 0;
}
