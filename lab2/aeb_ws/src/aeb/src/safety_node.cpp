#include "aeb/safety_node.hpp"

using std::placeholders::_1;

namespace aeb {
	SafetyNode::SafetyNode() : rclcpp::Node("aeb_safety_node") {
		scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
			"/scan", 10, 
			std::bind(&SafetyNode::ingest_scan, this, _1)
		);

		odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
			"/ego_racecar/odom", 10,
			std::bind(&SafetyNode::ingest_odom, this, _1)
		);
	}

	void SafetyNode::ingest_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
		RCLCPP_INFO(this->get_logger(), "scan.angle_min: %fi", msg->angle_min);
	}

	void SafetyNode::ingest_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
		RCLCPP_INFO(this->get_logger(), "odom.longitudinal_velocty: %f", msg->twist.twist.linear.x);
	}
}

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<aeb::SafetyNode>());
	rclcpp::shutdown();

	return 0;
}
