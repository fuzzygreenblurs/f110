#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

class SafetyNode : public rclcpp::Node {
public:
	SafetyNode();
	~SafetyNode();

	void update() {
		// read in data from /scan topic and store locally
		// read in data from /odom topic and store locally
	};
	void calc_ttc() {
		// float ego_long_velocity = odom.velocity
		// float ego_heading_angle = odom.steering_angle
		// range_rates = ego_long_velocity * math.cos(ego_heading_angle) * cloud.ranges
		
		// ttc_map = cloud.ranges / max(0, -1 * range_rates)
	
	}

	boolean is_within_ttc_threshold {
		// return any element in ttc_map <= ttc_threshold:
	}

	void brake() {
		// publish AckermannDriveStamped(velocity = 0.0);
	}

private:
	int beam_count = 1080;
	rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
	rclcpp::Subscriber<sensor_msgs::msg::LaserScan>SharedPtr lidar_subscriber_;
	rclcpp::Subscriber<nav_msgs::msg::Odometry> odom_subscriber_;

	sensor_msgs::msg::LaserScan cloud = sensor_msgs::msg::LaserScan();
	nav_msgs::msg::Odometry odom = nav_msgs::msg::Odometry();
	Matrix<float, 1, beam_count> ttc_map;
	Matrix<float, 1, beam_count> range_rates;
	
	boolean override_teleop;

}




int main() {
	safety_node = SafetyNode();
	

	// note: ROS2 doesnt have a built-in notion of ISRs
	// can we use ISRs through RTOS on top of ROS2?
	while(1) {
		safety_node.update();
		safety_node.calc_ttc()i;
		safety_node.trigger();
	}	

	return 0;
}
