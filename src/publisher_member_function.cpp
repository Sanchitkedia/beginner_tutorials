#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "beginner_tutorials/srv/string_change.hpp"  

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
    server_ = this->create_service<beginner_tutorials::srv::StringChange>("string_change", std::bind(&MinimalPublisher::string_change_callback, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = msg + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::StringChange>::SharedPtr server_;
  size_t count_;
  std::string msg = "Hey, Terrapins! Count with Me ";

  bool string_change_callback(const beginner_tutorials::srv::StringChange::Request::SharedPtr request, const beginner_tutorials::srv::StringChange::Response::SharedPtr response) {
    msg = request->input;
    response->output = msg;
    RCLCPP_INFO(this->get_logger(), "Incoming request\nOutput String is: %s", msg.c_str());
    RCLCPP_INFO(this->get_logger(), "Sending response to talker: %s", msg.c_str());
    return true;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}