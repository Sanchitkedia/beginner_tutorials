#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "beginner_tutorials/srv/string_change.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

class TestPub : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_pub");
    client_ = node_->create_client<beginner_tutorials::srv::StringChange>("string_change");
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_FATAL_STREAM(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        exit(1);
      }
      RCLCPP_INFO_STREAM(node_->get_logger(), "service not available, waiting again...");
    }
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<beginner_tutorials::srv::StringChange>::SharedPtr client_;
};

TEST_F(TestPub, testServiceInitialization){
  ASSERT_TRUE(client_->service_is_ready());
}

TEST_F(TestPub, testServiceResponse){
  auto request = std::make_shared<beginner_tutorials::srv::StringChange::Request>();
  request->input = "Hello World ";
  auto result = client_->async_send_request(request);
  RCLCPP_INFO_STREAM(node_->get_logger(), "Waiting for response...");
  auto ret = rclcpp::spin_until_future_complete(node_, result, 5s);  // Wait for the result.
  std::string answer = result.get()->output;
  RCLCPP_INFO_STREAM(node_->get_logger(), "Response received " + answer);
  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(answer, request->input);
}
TEST_F(TestPub, testTF){
  auto tfBuffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
  geometry_msgs::msg::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer->lookupTransform("world", "talk", rclcpp::Time(0), 5s);
  }
  catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(node_->get_logger(), "Failure %s ", ex.what());
  }
  ASSERT_EQ(transformStamped.header.frame_id, "world");
  ASSERT_EQ(transformStamped.child_frame_id, "talk");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
