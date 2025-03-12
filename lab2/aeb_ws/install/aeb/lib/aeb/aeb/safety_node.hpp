#ifndef SAFETY_NODE_HPP
#define SAFETY_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace aeb {

class SafetyNode : public rclcpp::Node {
public:
	SafetyNode();
	
private:
	void ingest_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
	void ingest_odom(const nav_msgs::msg::Odometry::SharedPtr msg);

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
};

}

#endif

