#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

// _1 is used as a placeholder for the first argument to be dynamically passed in
// as part of a binding call.
using std::placeholders::_1;

class SafetyNode : public rclcpp::Node {
public:
	SafetyNode() : Node("safety_node") {
		publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("safety_drive", 10);
		
		/*
		 * params:
		 * 	topic name
		 * 	queue size: specifies the queue size for the subscription's message queue 
		 *		- if more msgs arrive than can be stored, older msgs are dropped
		 *	callback: bind callback to this (current instance) and use placeholder to represent incoming msg
		 *	
		 */
		lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10,
			       	std::bind(&SafetyNode::ingest_scan, this, _1));

		odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("ego_racecar/odom", 10,
				std::bind(&SafetyNode::ingest_odometry, this, _1));
	};

	~SafetyNode();

	void ingest_odometry() {}

	void ingest_scan() {}

	void update() {
		// read in data from /scan topic and store locally
		
	 //	auto cloud = sensor_msgs::msg::LaserScan();
		
		
		// read in data from /odom topic and store locally
	};
	void calc_ttc() {
		// float ego_long_velocity = odom.velocity
		// float ego_heading_angle = odom.steering_angle
		// range_rates = ego_long_velocity * math.cos(ego_heading_angle) * cloud.ranges
		
		// ttc_map = cloud.ranges / max(0, -1 * range_rates)
	
	};

	bool is_within_ttc_threshold {
		// return any element in ttc_map <= ttc_threshold:
	
		return true;
	};

	void brake() {
		// publish AckermannDriveStamped(velocity = 0.0);
	};

private:
	int beam_count_ = 1080;
	rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>SharedPtr lidar_subscriber_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>SharedPtr odom_subscriber_;

	sensor_msgs::msg::LaserScan cloud_ = sensor_msgs::msg::LaserScan();
	nav_msgs::msg::Odometry odom_ = nav_msgs::msg::Odometry();
	Matrix<float, 1, beam_count> ttc_map_;
	Matrix<float, 1, beam_count> range_rates_;
	
	boolean override_teleop_;

}


int main() {
	safety_node = SafetyNode();
	

	// note: ROS2 doesnt have a built-in notion of ISRs
	// can we use ISRs through RTOS on top of ROS2?
//	while(1) {
//		safety_node.update();
//		safety_node.calc_ttc()i;
//		safety_node.trigger();
//	}	

	return 0;
}
