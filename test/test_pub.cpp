/******************************************************************************
  *MIT License

  *Copyright (c) 2022 Sanchit Kedia

  *Permission is hereby granted, free of charge, to any person obtaining a copy
  *of this software and associated documentation files (the "Software"), to deal
  *in the Software without restriction, including without limitation the rights
  *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  *copies of the Software, and to permit persons to whom the Software is
  *furnished to do so, subject to the following conditions:

  *The above copyright notice and this permission notice shall be included in all
  *copies or substantial portions of the Software.

  *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  *SOFTWARE.
  ******************************************************************************/
/**
 * @file test_pub.cpp
 * @author Sanchit Kedia (sanchit@terpmail.umd.edu)
 * @brief  Test file for the publisher node
 * @version 0.1
 * @date 2022-11-15
 *
 * @copyright MIT Copyright (c) 2022
 *
 */
#include "beginner_tutorials/srv/string_change.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "gtest/gtest.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
/**
 * @brief The Class definition for the node minimal_publisher
 *
 */
class TestPub : public ::testing::Test {
 protected:
 /**
  * @brief Set the Up object
  * 
  */
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_pub");
    client_ = node_->create_client<beginner_tutorials::srv::StringChange>(
        "string_change");
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_FATAL_STREAM(
            node_->get_logger(),
            "Interrupted while waiting for the service. Exiting.");
        exit(1);
      }
      RCLCPP_INFO_STREAM(node_->get_logger(),
                         "service not available, waiting again...");
    }
  }
/**
 * @brief Destroy the Test Pub object
 * 
 */
  void TearDown() override { rclcpp::shutdown(); }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<beginner_tutorials::srv::StringChange>::SharedPtr client_;
};
/**
 * @brief Test case to check if the service is initialized
 * 
 */
TEST_F(TestPub, testServiceInitialization) {
  ASSERT_TRUE(client_->service_is_ready());
}

/**
 * @brief Test case to check if the service responds with the correct string
 * 
 */
TEST_F(TestPub, testServiceResponse) {
  auto request =
      std::make_shared<beginner_tutorials::srv::StringChange::Request>();
  request->input = "Hello World ";
  auto result = client_->async_send_request(request);
  RCLCPP_INFO_STREAM(node_->get_logger(), "Waiting for response...");
  auto ret = rclcpp::spin_until_future_complete(node_, result,
                                                5s);
  std::string answer = result.get()->output;
  RCLCPP_INFO_STREAM(node_->get_logger(), "Response received " + answer);
  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(answer, request->input);
}

/**
 * @brief Test case to check if the tf is brodcasted correctly with world as the parent frame and talk as the child frame
 * 
 */
TEST_F(TestPub, testTF) {
  auto tfBuffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped =
        tfBuffer->lookupTransform("world", "talk", rclcpp::Time(0), 5s);
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(node_->get_logger(), "Failure %s ", ex.what());
  }
  ASSERT_EQ(transformStamped.header.frame_id, "world");
  ASSERT_EQ(transformStamped.child_frame_id, "talk");
}
/**
 * @brief Main function to run the test cases
 * 
 * @param argc  Number of arguments
 * @param argv  Arguments
 * @return int  Returns 0 if the test cases pass
 */
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
