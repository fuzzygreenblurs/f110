#ifndef SAFETY_NODE_HPP
#define SAFETY_NODE_HPP

#include <vector>
#include <Eigen/Dense> 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

namespace AEB {

class SafetyNode : public rclcpp::Node {
public:
	SafetyNode();
	
private:
	void ingest_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
	void ingest_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
	void brake();
	void is_within_threshold();
	float convert_to_yaw(const nav_msgs::msg::Odometry::SharedPtr msg);

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
	rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;

	static constexpr double EPSILON = 0.0001f;	
	static constexpr size_t BEAM_COUNT = 1080;
	static constexpr float AEB_MIN_RADIUS = 5;
	Eigen::ArrayXf ttc_map_;	

	float speed_;
	float heading_;
	
	static constexpr char* SCAN_TOPIC = "/scan";
	static constexpr char* ODOMETRY_TOPIC = "/ego_racecar/odom";
	static constexpr char* FUNNEL_TOPIC = "/drive_funnel";
};

}

#endif










