#include <iostream>
#include <functional>
#include "aeb/safety_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class SafetyNode : public rclcpp::Node {

public:
	SafetyNode() : Node("safety_node") {
		auto scan_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
			"scan",10, std::bind(&SafetyNode::ingest_scan, this, _1));
	};

	~SafetyNode() {};


private:
	sensor_msgs::msg::LaserScan scan_;

	void ingest_scan(const sensor_msgs::msg::LaserScan msg) {
		scan_ = msg;
	};
};

int main() {
	SafetyNode();
	return 0;
}
