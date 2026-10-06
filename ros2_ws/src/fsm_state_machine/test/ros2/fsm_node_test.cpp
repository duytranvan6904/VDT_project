#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "fsm_state_machine/fsm_node.hpp"
#include "vdt_msgs/msg/input_snapshot.hpp"
#include "vdt_msgs/msg/rc_fsm_input.hpp"

using namespace fsm_state_machine;
using namespace std::chrono_literals;

namespace
{
using Snapshot = vdt_msgs::msg::InputSnapshot;
using Rc = vdt_msgs::msg::RcFsmInput;
using BoolMsg = std_msgs::msg::Bool;

constexpr float kNan = std::numeric_limits<float>::quiet_NaN();

Snapshot snap(
  bool marker = true, float align = 0.1f, float alt = 2.0f, float delta_h = 1.0f,
  float d_horiz = 0.5f, float yaw_rate = 0.0f, bool touchdown = false)
{
  Snapshot s;
  s.valid = true;
  s.marker_detected = marker;
  s.align_error = align;
  s.altitude = alt;
  s.delta_h = delta_h;
  s.d_horiz = d_horiz;
  s.yaw_rate = yaw_rate;
  s.touchdown = touchdown;
  s.planner_timeout = false;
  return s;
}

Snapshot land_snap() {return snap(true, 0.15f, 0.4f, 0.3f, 0.1f);}
Snapshot touchdown_snap() {return snap(true, 0.0f, 0.05f, 0.0f, 0.0f, 0.0f, true);}

Rc rc(bool land_switch)
{
  Rc r;
  r.land_switch = land_switch;
  r.kill_switch = false;
  return r;
}
}  // namespace

class FsmNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    fsm_ = std::make_shared<FsmNode>();
    probe_ = std::make_shared<rclcpp::Node>("fsm_node_probe");

    snapshot_pub_ = probe_->create_publisher<Snapshot>("input_cache/snapshot", 10);
    rc_pub_ = probe_->create_publisher<Rc>("rc/fsm_input", 10);
    force_pub_ = probe_->create_publisher<BoolMsg>("safety/force_land", 10);
    killed_pub_ = probe_->create_publisher<BoolMsg>("system/killed", rclcpp::QoS(1).transient_local());

    state_sub_ = probe_->create_subscription<std_msgs::msg::UInt8>(
      "fsm/state", 10, [this](std_msgs::msg::UInt8::SharedPtr m) {states_.push_back(m->data);});
    mode_sub_ = probe_->create_subscription<std_msgs::msg::UInt8>(
      "planner/mode", 10, [this](std_msgs::msg::UInt8::SharedPtr m) {modes_.push_back(m->data);});
    gimbal_sub_ = probe_->create_subscription<std_msgs::msg::UInt8>(
      "gimbal/state_request", 10,
      [this](std_msgs::msg::UInt8::SharedPtr m) {gimbal_.push_back(m->data);});
    gain_sub_ = probe_->create_subscription<std_msgs::msg::Float32>(
      "planner/apf_gain", 10, [this](std_msgs::msg::Float32::SharedPtr m) {gains_.push_back(m->data);});
    align_sub_ = probe_->create_subscription<std_msgs::msg::Float32>(
      "gimbal/align_error_cmd", 10,
      [this](std_msgs::msg::Float32::SharedPtr m) {aligns_.push_back(m->data);});
    descent_sub_ = probe_->create_subscription<std_msgs::msg::Float32>(
      "cmd/vertical_descent_rate", 10,
      [this](std_msgs::msg::Float32::SharedPtr m) {descents_.push_back(m->data);});
    disarm_sub_ = probe_->create_subscription<BoolMsg>(
      "cmd/disarm_request", 10, [this](BoolMsg::SharedPtr m) {disarms_.push_back(m->data);});

    exec_.add_node(fsm_);
    exec_.add_node(probe_);
    wait_for_discovery();
  }

  void TearDown() override
  {
    exec_.remove_node(fsm_);
    exec_.remove_node(probe_);
  }

  void wait_for_discovery()
  {
    auto end = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < end) {
      exec_.spin_some();
      if (snapshot_pub_->get_subscription_count() > 0 && rc_pub_->get_subscription_count() > 0 &&
        force_pub_->get_subscription_count() > 0 && killed_pub_->get_subscription_count() > 0 &&
        state_sub_->get_publisher_count() > 0)
      {
        break;
      }
      std::this_thread::sleep_for(10ms);
    }
    std::this_thread::sleep_for(100ms);
  }

  void drive(double seconds, const Snapshot & s, const Rc & r, bool force = false)
  {
    auto start = std::chrono::steady_clock::now();
    auto end = start + std::chrono::milliseconds(static_cast<int>(seconds * 1000));
    auto next_pub = start;
    while (std::chrono::steady_clock::now() < end) {
      if (std::chrono::steady_clock::now() >= next_pub) {
        snapshot_pub_->publish(s);
        rc_pub_->publish(r);
        BoolMsg f;
        f.data = force;
        force_pub_->publish(f);
        next_pub += 50ms;
      }
      exec_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  void silence(double seconds)
  {
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<int>(seconds * 1000));
    while (std::chrono::steady_clock::now() < end) {
      exec_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  void kill()
  {
    BoolMsg k;
    k.data = true;
    killed_pub_->publish(k);
  }

  int last_state() const {return states_.empty() ? -1 : states_.back();}

  void to_follow() {drive(2.0, snap(), rc(false));}

  void to_approach()
  {
    to_follow();
    drive(0.8, snap(), rc(true));
  }

  std::shared_ptr<FsmNode> fsm_;
  std::shared_ptr<rclcpp::Node> probe_;
  rclcpp::executors::SingleThreadedExecutor exec_;

  rclcpp::Publisher<Snapshot>::SharedPtr snapshot_pub_;
  rclcpp::Publisher<Rc>::SharedPtr rc_pub_;
  rclcpp::Publisher<BoolMsg>::SharedPtr force_pub_;
  rclcpp::Publisher<BoolMsg>::SharedPtr killed_pub_;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr gimbal_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gain_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr align_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr descent_sub_;
  rclcpp::Subscription<BoolMsg>::SharedPtr disarm_sub_;

  std::vector<uint8_t> states_;
  std::vector<uint8_t> modes_;
  std::vector<uint8_t> gimbal_;
  std::vector<float> gains_;
  std::vector<float> aligns_;
  std::vector<float> descents_;
  std::vector<bool> disarms_;
};

TEST_F(FsmNodeTest, PublishesStateEveryCycle)
{
  drive(1.0, snap(true, 0.1f, 2.0f, 1.0f, 0.5f, 0.3f), rc(false));
  EXPECT_GE(states_.size(), 5u);
  EXPECT_EQ(last_state(), 0);
}

TEST_F(FsmNodeTest, SearchPublishesSearchCommands)
{
  drive(1.0, snap(true, 0.1f, 2.0f, 1.0f, 0.5f, 0.3f), rc(false));
  ASSERT_FALSE(modes_.empty());
  EXPECT_EQ(modes_.back(), 0);
  EXPECT_EQ(gimbal_.back(), 0);
  EXPECT_TRUE(gains_.empty());
  EXPECT_TRUE(aligns_.empty());
  ASSERT_FALSE(descents_.empty());
  EXPECT_FLOAT_EQ(descents_.back(), 0.0f);
}

TEST_F(FsmNodeTest, SearchToFollowWithStableMarker)
{
  drive(2.0, snap(), rc(false));
  EXPECT_EQ(last_state(), 1);
  EXPECT_EQ(modes_.back(), 1);
  EXPECT_EQ(gimbal_.back(), 1);
  ASSERT_FALSE(gains_.empty());
  EXPECT_FLOAT_EQ(gains_.back(), 1.0f);
  EXPECT_FLOAT_EQ(descents_.back(), 0.0f);
}

TEST_F(FsmNodeTest, HighYawRateBlocksFollow)
{
  drive(2.0, snap(true, 0.1f, 2.0f, 1.0f, 0.5f, 0.3f), rc(false));
  EXPECT_EQ(last_state(), 0);
}

TEST_F(FsmNodeTest, LowYawRateAllowsFollow)
{
  drive(2.0, snap(true, 0.1f, 2.0f, 1.0f, 0.5f, 0.02f), rc(false));
  EXPECT_EQ(last_state(), 1);
}

TEST_F(FsmNodeTest, NanYawRateBlocksFollow)
{
  drive(2.0, snap(true, 0.1f, 2.0f, 1.0f, 0.5f, kNan), rc(false));
  EXPECT_EQ(last_state(), 0);
}

TEST_F(FsmNodeTest, NanDeltaHBlocksFollow)
{
  drive(2.0, snap(true, 0.1f, 2.0f, kNan), rc(false));
  EXPECT_EQ(last_state(), 0);
}

TEST_F(FsmNodeTest, NanAlignErrorBlocksFollow)
{
  drive(2.0, snap(true, kNan), rc(false));
  EXPECT_EQ(last_state(), 0);
}

TEST_F(FsmNodeTest, FollowToApproachOnLandSwitch)
{
  to_approach();
  EXPECT_EQ(last_state(), 2);
  EXPECT_EQ(modes_.back(), 2);
  ASSERT_FALSE(gains_.empty());
  EXPECT_FLOAT_EQ(gains_.back(), 0.5f);
  ASSERT_FALSE(aligns_.empty());
  EXPECT_NEAR(aligns_.back(), 0.1f, 1e-6f);
  ASSERT_FALSE(descents_.empty());
  EXPECT_NEAR(descents_.back(), 0.3f, 1e-6f);
}

TEST_F(FsmNodeTest, ApproachBackToFollowWhenSwitchOff)
{
  to_approach();
  drive(0.5, snap(), rc(false));
  EXPECT_EQ(last_state(), 1);
}

TEST_F(FsmNodeTest, ApproachToLand)
{
  to_approach();
  drive(1.5, land_snap(), rc(true));
  EXPECT_EQ(last_state(), 3);
  EXPECT_EQ(modes_.back(), 3);
  EXPECT_FLOAT_EQ(gains_.back(), 0.0f);
  EXPECT_NEAR(descents_.back(), 0.4f, 1e-6f);
}

TEST_F(FsmNodeTest, LandToCompleteDisarms)
{
  to_approach();
  drive(1.5, land_snap(), rc(true));
  ASSERT_EQ(last_state(), 3);
  drive(0.8, touchdown_snap(), rc(true));
  EXPECT_EQ(last_state(), 4);
  ASSERT_FALSE(disarms_.empty());
  EXPECT_TRUE(disarms_.back());
}

TEST_F(FsmNodeTest, CompleteStopsOtherCommands)
{
  to_approach();
  drive(1.5, land_snap(), rc(true));
  drive(0.8, touchdown_snap(), rc(true));
  ASSERT_EQ(last_state(), 4);
  size_t descents = descents_.size();
  size_t modes = modes_.size();
  size_t gimbals = gimbal_.size();
  size_t gains = gains_.size();
  size_t disarms = disarms_.size();
  drive(0.6, touchdown_snap(), rc(true));
  EXPECT_EQ(descents_.size(), descents);
  EXPECT_EQ(modes_.size(), modes);
  EXPECT_EQ(gimbal_.size(), gimbals);
  EXPECT_EQ(gains_.size(), gains);
  EXPECT_GT(disarms_.size(), disarms);
  EXPECT_EQ(last_state(), 4);
}

TEST_F(FsmNodeTest, MarkerLostInFollowReturnsToSearch)
{
  to_follow();
  ASSERT_EQ(last_state(), 1);
  drive(3.2, snap(false), rc(false));
  EXPECT_EQ(last_state(), 0);
}

TEST_F(FsmNodeTest, SnapshotLossReturnsFollowToSearch)
{
  to_follow();
  ASSERT_EQ(last_state(), 1);
  silence(0.8);
  EXPECT_EQ(last_state(), 0);
}

TEST_F(FsmNodeTest, SnapshotLossReturnsApproachToSearch)
{
  to_approach();
  ASSERT_EQ(last_state(), 2);
  silence(0.8);
  EXPECT_EQ(last_state(), 0);
}

TEST_F(FsmNodeTest, PlannerTimeoutFlagReturnsFollowToSearch)
{
  to_follow();
  ASSERT_EQ(last_state(), 1);
  Snapshot s = snap();
  s.planner_timeout = true;
  drive(0.6, s, rc(false));
  EXPECT_EQ(last_state(), 0);
}

TEST_F(FsmNodeTest, ForceLandFromFollow)
{
  to_follow();
  drive(0.6, snap(), rc(false), true);
  EXPECT_EQ(last_state(), 3);
  EXPECT_NEAR(descents_.back(), 0.4f, 1e-6f);
}

TEST_F(FsmNodeTest, ForceLandFromSearch)
{
  drive(0.6, snap(true, 0.1f, 2.0f, 1.0f, 0.5f, 0.3f), rc(false), true);
  EXPECT_EQ(last_state(), 3);
}

TEST_F(FsmNodeTest, ForceLandFromApproach)
{
  to_approach();
  drive(0.6, snap(), rc(true), true);
  EXPECT_EQ(last_state(), 3);
}

TEST_F(FsmNodeTest, ForceLandThenTouchdownCompletes)
{
  to_follow();
  drive(0.6, snap(), rc(false), true);
  drive(0.8, touchdown_snap(), rc(false), true);
  EXPECT_EQ(last_state(), 4);
}

TEST_F(FsmNodeTest, LandSwitchDoesNotReviveAfterApproachAborted)
{
  to_approach();
  ASSERT_EQ(last_state(), 2);
  drive(2.0, snap(false), rc(true));
  EXPECT_EQ(last_state(), 1);
  drive(1.0, snap(), rc(true));
  EXPECT_EQ(last_state(), 1);
  drive(0.4, snap(), rc(false));
  drive(0.6, snap(), rc(true));
  EXPECT_EQ(last_state(), 2);
}

TEST_F(FsmNodeTest, KilledStopsAllPublishing)
{
  to_follow();
  ASSERT_EQ(last_state(), 1);
  kill();
  silence(0.4);
  size_t states = states_.size();
  size_t modes = modes_.size();
  size_t descents = descents_.size();
  size_t gains = gains_.size();
  drive(1.0, snap(), rc(true));
  EXPECT_EQ(states_.size(), states);
  EXPECT_EQ(modes_.size(), modes);
  EXPECT_EQ(descents_.size(), descents);
  EXPECT_EQ(gains_.size(), gains);
}

TEST_F(FsmNodeTest, KilledIgnoresForceLand)
{
  to_follow();
  kill();
  silence(0.4);
  size_t states = states_.size();
  drive(1.0, snap(), rc(false), true);
  EXPECT_EQ(states_.size(), states);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}