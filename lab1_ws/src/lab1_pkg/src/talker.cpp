#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
	public:
		MinimalPublisher() : Node("talker"), count_(0) {
			publisher_ = this->create_publisher<std_msgs::msg::String>("drive", 10);
		}

		
		void publish() {

			auto msg = std_msgs::msg::String();
			msg.data = "hello" + std::to_string(count_++);
			RCLCPP_INFO(this->get_logger(), "publishing: '%s'", msg.data.c_str());
			publisher_->publish(msg);
		}

	private:
		size_t count_;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
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


