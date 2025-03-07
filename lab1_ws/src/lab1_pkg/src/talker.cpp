#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
	public:
		MinimalPublisher() : Node("talker"), count_(0) {
			this->declare_parameter("v", 1.0);
			this->declare_parameter("d", 0.0);
			
			publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
			
		}

		
		void publish() {

			auto msg = ackermann_msgs::msg::AckermannDriveStamped();
			msg.drive.speed = this->get_parameter("v").as_double();
			msg.drive.steering_angle = this->get_parameter("d").as_double();
			RCLCPP_INFO(this->get_logger(), "publish: speed=%f, angle=%f", msg.drive.speed, msg.drive.steering_angle);
		
			publisher_->publish(msg);
		}

	private:
		size_t count_;
		rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
		rclcpp::TimerBase::SharedPtr timer_;
	
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<MinimalPublisher>();
	
	while(rclcpp::ok()) {
		node->publish();
		rclcpp::spin_some(node);
	}

	rclcpp::shutdown();
	return 0;
}


