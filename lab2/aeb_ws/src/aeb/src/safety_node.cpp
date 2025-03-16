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

		publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_funnel", 10);
	}

	void SafetyNode::ingest_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
		// c++ vectors can be cleared without reallocating/resizing
		// we want to clear the old projection and handle the incoming one
		
		projected_range_rates_.clear();
		projected_range_rates_.resize(msg->ranges.size());

		for(size_t i=0; i < msg->ranges.size(); i++) {
			double beam_angle = msg->angle_min + ((msg->angle_increment) * i);
			double relative_angle = beam_angle - heading_;
			double range_rate = speed_ * std::cos(relative_angle);

			projected_range_rates_[i] = range_rate;
		}


		RCLCPP_INFO(this->get_logger(), "scan.angle_min: %fi", msg->angle_min);
	}

	void SafetyNode::ingest_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
		speed_ = msg->twist.twist.linear.x;
		double heading_ = convert_to_yaw(msg);	
		
		RCLCPP_INFO(this->get_logger(), "odom.longitudinal_velocty: %f", msg->twist.twist.linear.x);
	}

	double convert_to_yaw(const nav_msgs::msg::Odometry::SharedPtr msg) {
		// quaternion to yaw
		double qw = msg->pose.pose.orientation.w;
		double qx = msg->pose.pose.orientation.x;
		double qy = msg->pose.pose.orientation.y;
		double qz = msg->pose.pose.orientation.z;

		double yaw = std::atan2(
			2.0 * ((qw * qx) + (qy * qz)),
			1.0 - (2.0 * (qx * qx) + (qy * qy))
		);

		return yaw;
	}

	void is_within_threshold() {
		// calculate ttc along each beam
		


		// if any(within_threshhold), return true
	}

	void SafetyNode::brake() {
		// publish AckermannDriveStamped().velocity = 0.0 to mux at "drive_funnel"

		auto msg = ackermann_msgs::msg::AckermannDriveStamped();
		msg.drive.speed = 0.0;
		publisher_->publish(msg);

		RCLCPP_INFO(this->get_logger(), "AEB::CriticalThresholdDetected : trigger brake");
	}
}

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<aeb::SafetyNode>());
	rclcpp::shutdown();

	return 0;
}
