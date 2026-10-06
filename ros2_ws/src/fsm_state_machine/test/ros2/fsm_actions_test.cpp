#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "fsm_state_machine/fsm_actions.hpp"

using namespace fsm_state_machine;
using namespace std::chrono_literals;

namespace
{
constexpr float kApproachRate = 0.3f;
constexpr float kLandRate = 0.4f;
constexpr float kLandEntryHeight = 0.5f;

SensorInput make_input(
  bool marker = true, bool geometry = true, float align = 0.1f, float delta_h = 1.0f)
{
  SensorInput s;
  s.valid = true;
  s.marker_detected = marker;
  s.geometry_valid = geometry;
  s.align_error = align;
  s.altitude = 2.0f;
  s.delta_h = delta_h;
  s.d_horiz = 0.5f;
  s.yaw_rate = 0.0f;
  return s;
}
}  // namespace

class ActuatorsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("fsm_actuators_test_node");
    probe_ = std::make_shared<rclcpp::Node>("fsm_actuators_probe");
    actuators_ = std::make_unique<FsmActuators>(node_.get());

    gimbal_sub_ = probe_->create_subscription<std_msgs::msg::UInt8>(
      "gimbal/state_request", 10, [this](std_msgs::msg::UInt8::SharedPtr m) {gimbal_.push_back(m->data);});
    mode_sub_ = probe_->create_subscription<std_msgs::msg::UInt8>(
      "planner/mode", 10, [this](std_msgs::msg::UInt8::SharedPtr m) {mode_.push_back(m->data);});
    gain_sub_ = probe_->create_subscription<std_msgs::msg::Float32>(
      "planner/apf_gain", 10, [this](std_msgs::msg::Float32::SharedPtr m) {gain_.push_back(m->data);});
    align_sub_ = probe_->create_subscription<std_msgs::msg::Float32>(
      "gimbal/align_error_cmd", 10,
      [this](std_msgs::msg::Float32::SharedPtr m) {align_.push_back(m->data);});
    descent_sub_ = probe_->create_subscription<std_msgs::msg::Float32>(
      "cmd/vertical_descent_rate", 10,
      [this](std_msgs::msg::Float32::SharedPtr m) {descent_.push_back(m->data);});
    disarm_sub_ = probe_->create_subscription<std_msgs::msg::Bool>(
      "cmd/disarm_request", 10, [this](std_msgs::msg::Bool::SharedPtr m) {disarm_.push_back(m->data);});

    exec_.add_node(node_);
    exec_.add_node(probe_);
    wait_for_discovery();
  }

  void TearDown() override
  {
    exec_.remove_node(node_);
    exec_.remove_node(probe_);
  }

  void spin_for(std::chrono::milliseconds d)
  {
    auto end = std::chrono::steady_clock::now() + d;
    while (std::chrono::steady_clock::now() < end) {
      exec_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  void wait_for_discovery()
  {
    auto end = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < end) {
      exec_.spin_some();
      if (gimbal_sub_->get_publisher_count() > 0 && mode_sub_->get_publisher_count() > 0 &&
        gain_sub_->get_publisher_count() > 0 && align_sub_->get_publisher_count() > 0 &&
        descent_sub_->get_publisher_count() > 0 && disarm_sub_->get_publisher_count() > 0)
      {
        break;
      }
      std::this_thread::sleep_for(10ms);
    }
    spin_for(100ms);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> probe_;
  std::unique_ptr<FsmActuators> actuators_;
  rclcpp::executors::SingleThreadedExecutor exec_;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr gimbal_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gain_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr align_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr descent_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr disarm_sub_;

  std::vector<uint8_t> gimbal_;
  std::vector<uint8_t> mode_;
  std::vector<float> gain_;
  std::vector<float> align_;
  std::vector<float> descent_;
  std::vector<bool> disarm_;
};

TEST_F(ActuatorsTest, SearchPublishesModeGimbalAndZeroDescent)
{
  actuators_->action_search();
  spin_for(300ms);
  ASSERT_EQ(mode_.size(), 1u);
  ASSERT_EQ(gimbal_.size(), 1u);
  EXPECT_EQ(mode_.back(), static_cast<uint8_t>(State::SEARCH));
  EXPECT_EQ(gimbal_.back(), static_cast<uint8_t>(State::SEARCH));
  ASSERT_EQ(descent_.size(), 1u);
  EXPECT_FLOAT_EQ(descent_.back(), 0.0f);
  EXPECT_TRUE(gain_.empty());
  EXPECT_TRUE(align_.empty());
  EXPECT_TRUE(disarm_.empty());
}

TEST_F(ActuatorsTest, FollowPublishesGainOneAndZeroDescent)
{
  actuators_->action_follow(make_input());
  spin_for(300ms);
  ASSERT_EQ(mode_.size(), 1u);
  EXPECT_EQ(mode_.back(), static_cast<uint8_t>(State::FOLLOW));
  EXPECT_EQ(gimbal_.back(), static_cast<uint8_t>(State::FOLLOW));
  ASSERT_EQ(gain_.size(), 1u);
  EXPECT_FLOAT_EQ(gain_.back(), 1.0f);
  ASSERT_EQ(descent_.size(), 1u);
  EXPECT_FLOAT_EQ(descent_.back(), 0.0f);
  EXPECT_TRUE(align_.empty());
  EXPECT_TRUE(disarm_.empty());
}

TEST_F(ActuatorsTest, ApproachPublishesGainHalfAlignErrorAndDescent)
{
  actuators_->action_approach(make_input(true, true, 0.12f, 1.0f), kLandEntryHeight);
  spin_for(300ms);
  ASSERT_EQ(mode_.size(), 1u);
  EXPECT_EQ(mode_.back(), static_cast<uint8_t>(State::APPROACH));
  EXPECT_EQ(gimbal_.back(), static_cast<uint8_t>(State::APPROACH));
  ASSERT_EQ(gain_.size(), 1u);
  EXPECT_FLOAT_EQ(gain_.back(), 0.5f);
  ASSERT_EQ(align_.size(), 1u);
  EXPECT_FLOAT_EQ(align_.back(), 0.12f);
  ASSERT_EQ(descent_.size(), 1u);
  EXPECT_NEAR(descent_.back(), kApproachRate, 1e-6f);
  EXPECT_TRUE(disarm_.empty());
}

TEST_F(ActuatorsTest, ApproachDescendsAtExactlyLandEntryHeight)
{
  actuators_->action_approach(make_input(true, true, 0.1f, kLandEntryHeight), kLandEntryHeight);
  spin_for(300ms);
  ASSERT_EQ(descent_.size(), 1u);
  EXPECT_NEAR(descent_.back(), kApproachRate, 1e-6f);
}

TEST_F(ActuatorsTest, ApproachStopsDescentBelowLandEntryHeight)
{
  actuators_->action_approach(make_input(true, true, 0.1f, 0.3f), kLandEntryHeight);
  spin_for(300ms);
  ASSERT_EQ(descent_.size(), 1u);
  EXPECT_FLOAT_EQ(descent_.back(), 0.0f);
}

TEST_F(ActuatorsTest, ApproachNoDescentWithoutMarker)
{
  actuators_->action_approach(make_input(false, true, 0.1f, 1.0f), kLandEntryHeight);
  spin_for(300ms);
  ASSERT_EQ(descent_.size(), 1u);
  EXPECT_FLOAT_EQ(descent_.back(), 0.0f);
}

TEST_F(ActuatorsTest, ApproachNoDescentWithInvalidGeometry)
{
  actuators_->action_approach(make_input(true, false, 0.1f, 1.0f), kLandEntryHeight);
  spin_for(300ms);
  ASSERT_EQ(descent_.size(), 1u);
  EXPECT_FLOAT_EQ(descent_.back(), 0.0f);
}

TEST_F(ActuatorsTest, LandPublishesGainZeroAndLandDescent)
{
  actuators_->action_land(make_input());
  spin_for(300ms);
  ASSERT_EQ(mode_.size(), 1u);
  EXPECT_EQ(mode_.back(), static_cast<uint8_t>(State::LAND));
  EXPECT_EQ(gimbal_.back(), static_cast<uint8_t>(State::LAND));
  ASSERT_EQ(gain_.size(), 1u);
  EXPECT_FLOAT_EQ(gain_.back(), 0.0f);
  ASSERT_EQ(descent_.size(), 1u);
  EXPECT_NEAR(descent_.back(), kLandRate, 1e-6f);
  EXPECT_TRUE(align_.empty());
  EXPECT_TRUE(disarm_.empty());
}

TEST_F(ActuatorsTest, CompletePublishesOnlyDisarm)
{
  actuators_->action_complete();
  spin_for(300ms);
  ASSERT_EQ(disarm_.size(), 1u);
  EXPECT_TRUE(disarm_.back());
  EXPECT_TRUE(mode_.empty());
  EXPECT_TRUE(gimbal_.empty());
  EXPECT_TRUE(gain_.empty());
  EXPECT_TRUE(align_.empty());
  EXPECT_TRUE(descent_.empty());
}

TEST_F(ActuatorsTest, CompletePublishesEveryCall)
{
  for (int i = 0; i < 3; ++i) {
    actuators_->action_complete();
  }
  spin_for(300ms);
  EXPECT_EQ(disarm_.size(), 3u);
}

TEST_F(ActuatorsTest, SequenceKeepsOrder)
{
  actuators_->action_search();
  actuators_->action_follow(make_input());
  actuators_->action_approach(make_input(), kLandEntryHeight);
  actuators_->action_land(make_input());
  spin_for(400ms);
  std::vector<uint8_t> expected = {0, 1, 2, 3};
  EXPECT_EQ(mode_, expected);
  EXPECT_EQ(gimbal_, expected);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}