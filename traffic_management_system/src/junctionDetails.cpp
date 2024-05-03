// @TODO: Need to publish these junction details as a topic (custom message) or
// create a param file with the junction details.

#include <rclcpp/rclcpp.hpp>

class JunctionDetails : public rclcpp::Node {
 public:
  JunctionDetails() : rclcpp::Node("junction_details", rclcpp::NodeOptions()) {
    declare_parameter("junction_1", std::vector<int64_t>(8, -1));
    declare_parameter("junction_2", std::vector<int64_t>(8, -1));

    auto junction1 = get_parameter("junction_1").as_integer_array();
    auto junction2 = get_parameter("junction_2").as_integer_array();

    // std::cout << static_cast<int64_t>(junction1[4]) << std::endl;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JunctionDetails>());
  rclcpp::shutdown();
  return 0;
}
