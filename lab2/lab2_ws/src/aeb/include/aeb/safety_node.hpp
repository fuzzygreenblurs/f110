#ifndef SAFETY_NODE
#define SAFETY_NODE

#include "rclcpp/rclcpp.hpp"

class SafetyNode : public rclcpp::Node() {
public:
	SafetyNode();
	~SafetyNode();

private:
	void ingest_scan(const sensor_msgs::msg::LaserScan msg);
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>SharedPtr scan_subscription_;
}


#endif
