#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "input_state_cache/input_cache_logic.hpp"

using namespace input_state_cache;

namespace
{
const float kNaN = std::numeric_limits<float>::quiet_NaN();
const float kInf = std::numeric_limits<float>::infinity();
const double kNow = 100.0;

struct Fixture
{
  RawSensors raw;
  TopicFreshness fr;
  TimeoutThresholds th;

  Fixture()
  {
    raw.ekf.received = true;
    raw.tracking.received = true;
    raw.odom.received = true;
    raw.vision.received = true;
    raw.alt.received = true;
    raw.tracking.expired = false;
    raw.ekf.x = 1.0f; raw.ekf.y = 2.0f; raw.ekf.z = 0.0f;
    raw.odom.x = 0.0f; raw.odom.y = 0.0f; raw.odom.z = 1.5f;
    raw.vision.marker_visible = true;
    raw.vision.pixel_align_error = 0.1f;
    raw.alt.altitude = 1.5f;
    raw.alt.touchdown_flag = false;
    fr.last_ekf_time = kNow - 0.1;
    fr.last_tracking_time = kNow - 0.1;
    fr.last_odom_time = kNow - 0.1;
    fr.last_vision_time = kNow - 0.1;
    fr.last_alt_time = kNow - 0.1;
    fr.last_planner_time = kNow - 0.1;
  }

  SnapshotData run() const {return build_snapshot(raw, fr, th, kNow);}
};
}  // namespace

TEST(InputCacheLogic, AllValid)
{
  Fixture f;
  auto s = f.run();
  EXPECT_TRUE(s.valid);
  EXPECT_TRUE(s.ekf_valid);
  EXPECT_TRUE(s.odom_valid);
  EXPECT_TRUE(s.alt_valid);
  EXPECT_TRUE(s.marker_detected);
  EXPECT_TRUE(s.all_sensors_valid);
  EXPECT_FALSE(s.planner_timeout);
  EXPECT_NEAR(s.d_horiz, std::sqrt(5.0f), 1e-5);
  EXPECT_FLOAT_EQ(s.align_error, s.d_horiz);
  EXPECT_FLOAT_EQ(s.delta_h, 1.5f);
  EXPECT_FLOAT_EQ(s.altitude, 1.5f);
}

TEST(InputCacheLogic, EkfStaleKeepsValidButGeometryNaN)
{
  Fixture f;
  f.fr.last_ekf_time = kNow - 0.6;
  auto s = f.run();
  EXPECT_TRUE(s.ekf_timeout);
  EXPECT_FALSE(s.ekf_valid);
  EXPECT_TRUE(std::isnan(s.delta_h));
  EXPECT_TRUE(std::isnan(s.d_horiz));
  EXPECT_TRUE(std::isnan(s.align_error));
  EXPECT_TRUE(s.valid);
  EXPECT_FALSE(s.all_sensors_valid);
}

TEST(InputCacheLogic, TrackingModeStale)
{
  Fixture f;
  f.fr.last_tracking_time = kNow - 0.6;
  auto s = f.run();
  EXPECT_TRUE(s.tracking_timeout);
  EXPECT_FALSE(s.ekf_valid);
}

TEST(InputCacheLogic, TrackingModeExpired)
{
  Fixture f;
  f.raw.tracking.expired = true;
  auto s = f.run();
  EXPECT_TRUE(s.ekf_expired);
  EXPECT_FALSE(s.ekf_valid);
  EXPECT_TRUE(std::isnan(s.d_horiz));
}

TEST(InputCacheLogic, NeverReceivedIsTimeout)
{
  Fixture a;
  a.raw.ekf.received = false;
  EXPECT_TRUE(a.run().ekf_timeout);
  Fixture b;
  b.raw.tracking.received = false;
  EXPECT_TRUE(b.run().tracking_timeout);
  Fixture c;
  c.raw.odom.received = false;
  EXPECT_TRUE(c.run().odom_timeout);
  Fixture d;
  d.raw.vision.received = false;
  EXPECT_TRUE(d.run().vision_timeout);
  Fixture e;
  e.raw.alt.received = false;
  EXPECT_TRUE(e.run().alt_timeout);
  Fixture g;
  g.fr.last_planner_time = 0.0;
  EXPECT_TRUE(g.run().planner_timeout);
}

TEST(InputCacheLogic, EkfNaNPosition)
{
  Fixture f;
  f.raw.ekf.z = kNaN;
  auto s = f.run();
  EXPECT_FALSE(s.ekf_valid);
  EXPECT_TRUE(std::isnan(s.delta_h));
}

TEST(InputCacheLogic, OdomInfPosition)
{
  Fixture f;
  f.raw.odom.x = kInf;
  auto s = f.run();
  EXPECT_FALSE(s.odom_valid);
  EXPECT_FALSE(s.valid);
  EXPECT_TRUE(std::isnan(s.d_horiz));
}

TEST(InputCacheLogic, AltNaN)
{
  Fixture f;
  f.raw.alt.altitude = kNaN;
  auto s = f.run();
  EXPECT_FALSE(s.alt_valid);
  EXPECT_TRUE(std::isnan(s.altitude));
  EXPECT_FALSE(s.valid);
}

TEST(InputCacheLogic, MarkerNotVisible)
{
  Fixture f;
  f.raw.vision.marker_visible = false;
  EXPECT_FALSE(f.run().marker_detected);
}

TEST(InputCacheLogic, MarkerAlignErrorNaN)
{
  Fixture f;
  f.raw.vision.pixel_align_error = kNaN;
  EXPECT_FALSE(f.run().marker_detected);
}

TEST(InputCacheLogic, MarkerStale)
{
  Fixture f;
  f.fr.last_vision_time = kNow - 0.6;
  auto s = f.run();
  EXPECT_TRUE(s.vision_timeout);
  EXPECT_FALSE(s.marker_detected);
}

TEST(InputCacheLogic, TouchdownRequiresFreshAlt)
{
  Fixture f;
  f.raw.alt.touchdown_flag = true;
  EXPECT_TRUE(f.run().touchdown);
  f.fr.last_alt_time = kNow - 0.6;
  auto s = f.run();
  EXPECT_FALSE(s.touchdown);
  EXPECT_TRUE(s.touchdown_flag);
}

TEST(InputCacheLogic, PlannerTimeout)
{
  Fixture f;
  f.fr.last_planner_time = kNow - 1.5;
  EXPECT_TRUE(f.run().planner_timeout);
}

TEST(InputCacheLogic, ExactLimitIsFresh)
{
  Fixture f;
  f.fr.last_odom_time = kNow - 0.5;
  EXPECT_FALSE(f.run().odom_timeout);
}

TEST(InputCacheLogic, DeltaHNegativeWhenUavBelowPad)
{
  Fixture f;
  f.raw.odom.z = -0.5f;
  EXPECT_FLOAT_EQ(f.run().delta_h, -0.5f);
}