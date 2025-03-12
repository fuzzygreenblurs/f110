#include "aeb/safety_node.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace aeb {
	SafetyNode::SafetyNode() : rclcpp::Node("aeb_safety_node") {
		scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
				"/scan", 10, 
				std::bind(&SafetyNode::ingest_scan, this, _1)
		);
	}

	void SafetyNode::ingest_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
		RCLCPP_INFO(this->get_logger(), "scan msg of angle_min: %fi", msg->angle_min);
	}
}

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<aeb::SafetyNode>());
	rclcpp::shutdown();

	return 0;
}
