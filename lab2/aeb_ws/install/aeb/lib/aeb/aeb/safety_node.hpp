#ifndef SAFETY_NODE_HPP
#define SAFETY_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace aeb {

class SafetyNode : public rclcpp::Node {
public:
	SafetyNode();
	
private:
	void ingest_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
};

}

#endif

