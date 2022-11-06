/**
 * @file publisher_member_function.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-05
 *
 * @copyright MIT Copyright (c) 2022
 *
 */

#include "beginner_tutorials/srv/string_change.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

using namespace std::chrono_literals;
/**
 * @brief The Class definition for the node minimal_publisher
 *
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Publisher object
   *
   */
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    this->declare_parameter("Publisher_Frequency",
                            1.0);  // Declare the parameter Publisher_Frequency
    publisher_ = this->create_publisher<std_msgs::msg::String>(
        "topic", 10);  // 10 is the queue size

    double talker_freq = 1.0;  // Default frequency

    if (this->get_parameter("Publisher_Frequency")
            .get_parameter_value()
            .get<double>() > 0.0) {  // Check if the frequency is greater
                                    // than 0, set the frequency to
                                   // the parameter value
      talker_freq = this->get_parameter("Publisher_Frequency")
                        .get_parameter_value()
                        .get<double>();
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Talker publishing at "
                                                  << talker_freq << " Hz"
                                                  << std::endl);
    } else if (this->get_parameter("Publisher_Frequency")
                   .get_parameter_value()
                   .get<double>() < 0.0) {  // If the frequency is negative, set
                                           // it to the default value
      RCLCPP_FATAL_STREAM(this->get_logger(),
                          "Positive value of frequency expected" << std::endl);
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "Talker frequency set to default value of 1.0Hz"
                             << std::endl);
      talker_freq = 1.0;
    } else if (this->get_parameter("Publisher_Frequency")
                   .get_parameter_value()
                   .get<double>() ==
               0.0) {  // If the frequency is zero, set it to the default value
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Frequency cant be zero" << std::endl);
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "Talker frequency set to default value of 1.0Hz"
                             << std::endl);
      talker_freq = 1.0;
    }

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(talker_freq),
        std::bind(&MinimalPublisher::timer_callback, this));
    server_ = this->create_service<beginner_tutorials::srv::StringChange>(
        "string_change",
        std::bind(&MinimalPublisher::string_change_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

 private:
  /**
   * @brief Callback function for the timer which will loop the publisher and
   * publish the message
   *
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = msg + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::StringChange>::SharedPtr
      server_;
  size_t count_;
  std::string msg = "Hey, Terrapins! Count with Me ";  // Default message
  /**
   * @brief service callback function to change the message being published
   *
   * @param request  The service request
   * @param response  The service response
   */
  void string_change_callback(
      const beginner_tutorials::srv::StringChange::Request::SharedPtr request,
      const beginner_tutorials::srv::StringChange::Response::SharedPtr
          response) {
    if (request->input == "reset") {  // If the input is reset, set the message
                                     // to the default message
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "Resetting the string" << std::endl);
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "The new string is:" << msg << std::endl);
      msg = "Hey, Terrapins! Count with Me ";
      response->output = msg;
    } else if (request->input.empty()) {  // If the input is empty, set the
                                         // message to the default message
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Empty string received" << std::endl);
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "No change will be made to the original string"
                             << std::endl);
    } else {  // If the input is not empty or reset,
             // set the message to the input
      msg = request->input;
      RCLCPP_INFO_STREAM(this->get_logger(), "New string received" << msg);
      response->output = msg;
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Incoming request\nThe New String is: %s"
                             << msg.c_str() << std::endl);
      RCLCPP_INFO_STREAM(this->get_logger(), "Sending response to talker: %s"
                                                 << msg.c_str() << std::endl);
    }
  }
};
/**
 * @brief Main function for the node which initializes the node and spins it
 *
 * @param argc  The number of arguments
 * @param argv  The arguments
 * @return int  The return value
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
