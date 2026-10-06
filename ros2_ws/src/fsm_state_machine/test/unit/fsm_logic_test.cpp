#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "fsm_state_machine/fsm_logic.hpp"

using namespace fsm_state_machine;

namespace
{
constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
constexpr float kDt = 0.1f;

SensorInput make_sensor(
  bool marker = true, float align = 0.1f, float alt = 2.0f, float delta_h = 1.0f,
  float d_horiz = 0.5f, float yaw_rate = 0.0f, bool touchdown = false)
{
  SensorInput s;
  s.valid = true;
  s.marker_detected = marker;
  s.align_error = align;
  s.altitude = alt;
  s.delta_h = delta_h;
  s.d_horiz = d_horiz;
  s.yaw_rate = yaw_rate;
  s.touchdown = touchdown;
  return s;
}

SensorInput validated(const SensorInput & s)
{
  return sensor_validate(s, TimeoutFlags{});
}

RcInput make_rc(bool land_switch, bool kill_switch = false)
{
  RcInput rc;
  rc.land_switch = land_switch;
  rc.kill_switch = kill_switch;
  return rc;
}

SensorInput good_follow() {return make_sensor();}
SensorInput good_land() {return make_sensor(true, 0.15f, 0.4f, 0.3f, 0.1f);}
SensorInput lost_marker() {return make_sensor(false);}
SensorInput touchdown_sensor() {return make_sensor(true, 0.0f, 0.05f, 0.0f, 0.0f, 0.0f, true);}

struct Harness
{
  FsmParams p;
  FsmContext ctx;

  void step(
    const SensorInput & raw, const RcInput & rc, bool force_land = false,
    bool planner_timeout = false, float dt = kDt)
  {
    RcInput eff = effective_rc_input(rc, force_land);
    SensorInput s = sensor_validate(raw, TimeoutFlags{});
    counters_update_marker_stable(ctx.counters, search_entry_ok(s, p));
    counters_update_marker_lost(ctx.counters, s.marker_detected, dt);
    counters_update_land_ok(ctx.counters, ctx.state, land_entry_ok(s, p));
    State prev = ctx.state;
    std::optional<State> next = force_land_transition(prev, force_land);
    if (!next) {
      next = planner_timeout_transition(prev, planner_timeout, p);
    }
    if (!next) {
      next = evaluate_transition(prev, ctx.counters, eff, s, p);
    }
    if (next) {
      ctx.state = *next;
    }
    if (ctx.state != prev) {
      counters_reset(ctx.counters);
    }
    counters_update_land_inhibit(ctx.counters, prev, ctx.state, eff);
  }

  void run(
    const SensorInput & raw, const RcInput & rc, int n, bool force_land = false,
    bool planner_timeout = false)
  {
    for (int i = 0; i < n; ++i) {
      step(raw, rc, force_land, planner_timeout);
    }
  }

  void to_follow() {run(good_follow(), make_rc(false), p.enter_follow_cycles + 1);}

  void to_approach()
  {
    to_follow();
    run(good_follow(), make_rc(true), 1);
  }

  void to_land()
  {
    to_approach();
    run(good_land(), make_rc(true), p.land_entry_cycles + 1);
  }
};
}  // namespace

TEST(FsmTypes, Defaults)
{
  FsmParams p;
  EXPECT_EQ(p.enter_follow_cycles, 10);
  EXPECT_FLOAT_EQ(p.follow_lost_timeout, 2.5f);
  EXPECT_FLOAT_EQ(p.approach_lost_timeout, 1.5f);
  EXPECT_FLOAT_EQ(p.align_threshold, 0.3f);
  EXPECT_FLOAT_EQ(p.land_entry_height, 0.5f);
  EXPECT_EQ(p.land_entry_cycles, 5);
  EXPECT_FLOAT_EQ(p.yaw_settle_rate, 0.1f);
  EXPECT_TRUE(p.land_requires_rearm);
  EXPECT_FALSE(p.ignore_planner_timeout);
  FsmContext c;
  EXPECT_EQ(c.state, State::SEARCH);
  EXPECT_EQ(c.counters.marker_stable_count, 0);
  EXPECT_EQ(c.counters.land_ok_count, 0);
  EXPECT_FALSE(c.counters.land_inhibit);
  SensorInput s;
  EXPECT_TRUE(std::isnan(s.yaw_rate));
  EXPECT_FALSE(s.geometry_valid);
}

TEST(SensorValidate, ValidInputKeepsValidAndGeometry)
{
  SensorInput s = validated(make_sensor());
  EXPECT_TRUE(s.valid);
  EXPECT_TRUE(s.geometry_valid);
}

TEST(SensorValidate, NanAlignErrorInvalidates)
{
  SensorInput s = validated(make_sensor(true, kNan));
  EXPECT_FALSE(s.valid);
  EXPECT_FALSE(s.geometry_valid);
}

TEST(SensorValidate, NanAltitudeInvalidates)
{
  SensorInput s = validated(make_sensor(true, 0.1f, kNan));
  EXPECT_FALSE(s.valid);
  EXPECT_FALSE(s.geometry_valid);
}

TEST(SensorValidate, NanDeltaHKeepsValidButGeometryFalse)
{
  SensorInput s = validated(make_sensor(true, 0.1f, 2.0f, kNan));
  EXPECT_TRUE(s.valid);
  EXPECT_FALSE(s.geometry_valid);
}

TEST(SensorValidate, NanDHorizKeepsValidButGeometryFalse)
{
  SensorInput s = validated(make_sensor(true, 0.1f, 2.0f, 1.0f, kNan));
  EXPECT_TRUE(s.valid);
  EXPECT_FALSE(s.geometry_valid);
}

TEST(SensorValidate, InputAlreadyInvalidStaysInvalid)
{
  SensorInput raw = make_sensor();
  raw.valid = false;
  SensorInput s = validated(raw);
  EXPECT_FALSE(s.valid);
  EXPECT_FALSE(s.geometry_valid);
}

TEST(SensorValidate, EkfTimeoutInvalidatesAndClearsMarker)
{
  TimeoutFlags f;
  f.ekf_timeout = true;
  SensorInput s = sensor_validate(make_sensor(), f);
  EXPECT_FALSE(s.valid);
  EXPECT_FALSE(s.marker_detected);
  EXPECT_FALSE(s.geometry_valid);
}

TEST(SensorValidate, VisionTimeoutInvalidatesAndClearsMarker)
{
  TimeoutFlags f;
  f.vision_timeout = true;
  SensorInput s = sensor_validate(make_sensor(), f);
  EXPECT_FALSE(s.valid);
  EXPECT_FALSE(s.marker_detected);
  EXPECT_FALSE(s.geometry_valid);
}

TEST(SensorValidate, PlannerTimeoutFlagDoesNotAffectSensor)
{
  TimeoutFlags f;
  f.planner_timeout = true;
  SensorInput s = sensor_validate(make_sensor(), f);
  EXPECT_TRUE(s.valid);
  EXPECT_TRUE(s.marker_detected);
  EXPECT_TRUE(s.geometry_valid);
}

TEST(SensorValidate, InfiniteDeltaHBreaksGeometryOnly)
{
  SensorInput s = validated(make_sensor(true, 0.1f, 2.0f, std::numeric_limits<float>::infinity()));
  EXPECT_TRUE(s.valid);
  EXPECT_FALSE(s.geometry_valid);
}

TEST(SearchEntry, RequiresMarker)
{
  FsmParams p;
  EXPECT_TRUE(search_entry_ok(validated(make_sensor()), p));
  EXPECT_FALSE(search_entry_ok(validated(make_sensor(false)), p));
}

TEST(SearchEntry, NanGeometryBlocks)
{
  FsmParams p;
  EXPECT_FALSE(search_entry_ok(validated(make_sensor(true, 0.1f, 2.0f, kNan)), p));
  EXPECT_FALSE(search_entry_ok(validated(make_sensor(true, 0.1f, 2.0f, 1.0f, kNan)), p));
}

TEST(SearchEntry, InvalidSensorBlocks)
{
  FsmParams p;
  EXPECT_FALSE(search_entry_ok(validated(make_sensor(true, kNan)), p));
  EXPECT_FALSE(search_entry_ok(validated(make_sensor(true, 0.1f, kNan)), p));
}

TEST(SearchEntry, YawRateThreshold)
{
  FsmParams p;
  EXPECT_FALSE(search_entry_ok(validated(make_sensor(true, 0.1f, 2.0f, 1.0f, 0.5f, 0.3f)), p));
  EXPECT_FALSE(search_entry_ok(validated(make_sensor(true, 0.1f, 2.0f, 1.0f, 0.5f, -0.3f)), p));
  EXPECT_TRUE(search_entry_ok(validated(make_sensor(true, 0.1f, 2.0f, 1.0f, 0.5f, 0.02f)), p));
  EXPECT_TRUE(search_entry_ok(validated(make_sensor(true, 0.1f, 2.0f, 1.0f, 0.5f, -0.02f)), p));
}

TEST(SearchEntry, NanYawRateBlocks)
{
  FsmParams p;
  EXPECT_FALSE(search_entry_ok(validated(make_sensor(true, 0.1f, 2.0f, 1.0f, 0.5f, kNan)), p));
}

TEST(SearchEntry, CustomYawSettleRate)
{
  FsmParams p;
  p.yaw_settle_rate = 0.5f;
  EXPECT_TRUE(search_entry_ok(validated(make_sensor(true, 0.1f, 2.0f, 1.0f, 0.5f, 0.3f)), p));
}

TEST(LandEntry, AllConditionsMet)
{
  FsmParams p;
  EXPECT_TRUE(land_entry_ok(validated(good_land()), p));
}

TEST(LandEntry, AlignTooLarge)
{
  FsmParams p;
  EXPECT_FALSE(land_entry_ok(validated(make_sensor(true, 0.5f, 0.4f, 0.3f, 0.1f)), p));
}

TEST(LandEntry, DeltaHTooLarge)
{
  FsmParams p;
  EXPECT_FALSE(land_entry_ok(validated(make_sensor(true, 0.15f, 0.4f, 1.0f, 0.1f)), p));
}

TEST(LandEntry, GeometryInvalid)
{
  FsmParams p;
  EXPECT_FALSE(land_entry_ok(validated(make_sensor(true, 0.15f, 0.4f, kNan, 0.1f)), p));
  EXPECT_FALSE(land_entry_ok(validated(make_sensor(true, kNan, 0.4f, 0.3f, 0.1f)), p));
}

TEST(LandEntry, ThresholdsAreStrict)
{
  FsmParams p;
  EXPECT_FALSE(land_entry_ok(validated(make_sensor(true, p.align_threshold, 0.4f, 0.3f, 0.1f)), p));
  EXPECT_FALSE(land_entry_ok(validated(make_sensor(true, 0.15f, 0.4f, p.land_entry_height, 0.1f)), p));
}

TEST(LandEntry, DoesNotRequireMarker)
{
  FsmParams p;
  EXPECT_TRUE(land_entry_ok(validated(make_sensor(false, 0.15f, 0.4f, 0.3f, 0.1f)), p));
}

TEST(CheckFollow, LostTimeoutHasPriorityOverApproach)
{
  FsmParams p;
  Counters c;
  c.marker_lost_time = p.follow_lost_timeout + 0.1f;
  EXPECT_EQ(check_follow_transition(c, make_rc(true), good_follow(), p), State::SEARCH);
}

TEST(Counters, MarkerStableIncrementsAndResets)
{
  Counters c;
  counters_update_marker_stable(c, true);
  counters_update_marker_stable(c, true);
  counters_update_marker_stable(c, true);
  EXPECT_EQ(c.marker_stable_count, 3);
  counters_update_marker_stable(c, false);
  EXPECT_EQ(c.marker_stable_count, 0);
}

TEST(Counters, MarkerLostAccumulatesDt)
{
  Counters c;
  counters_update_marker_lost(c, false, 0.1f);
  counters_update_marker_lost(c, false, 0.1f);
  counters_update_marker_lost(c, false, 0.3f);
  EXPECT_NEAR(c.marker_lost_time, 0.5f, 1e-5f);
}

TEST(Counters, MarkerLostResetsWhenDetected)
{
  Counters c;
  counters_update_marker_lost(c, false, 1.0f);
  counters_update_marker_lost(c, true, 0.1f);
  EXPECT_NEAR(c.marker_lost_time, 0.0f, 1e-5f);
}

TEST(Counters, LandOkOnlyCountsInApproach)
{
  Counters c;
  for (State s : {State::SEARCH, State::FOLLOW, State::LAND, State::COMPLETE}) {
    counters_update_land_ok(c, s, true);
    EXPECT_EQ(c.land_ok_count, 0);
  }
  counters_update_land_ok(c, State::APPROACH, true);
  counters_update_land_ok(c, State::APPROACH, true);
  EXPECT_EQ(c.land_ok_count, 2);
}

TEST(Counters, LandOkResetsOnFalse)
{
  Counters c;
  counters_update_land_ok(c, State::APPROACH, true);
  counters_update_land_ok(c, State::APPROACH, true);
  counters_update_land_ok(c, State::APPROACH, false);
  EXPECT_EQ(c.land_ok_count, 0);
}

TEST(Counters, LandInhibitSetWhenLeavingApproachWithSwitchOn)
{
  for (State next : {State::FOLLOW, State::SEARCH}) {
    Counters c;
    counters_update_land_inhibit(c, State::APPROACH, next, make_rc(true));
    EXPECT_TRUE(c.land_inhibit);
  }
}

TEST(Counters, LandInhibitNotSetWhenGoingToLand)
{
  Counters c;
  counters_update_land_inhibit(c, State::APPROACH, State::LAND, make_rc(true));
  EXPECT_FALSE(c.land_inhibit);
}

TEST(Counters, LandInhibitNotSetWhenSwitchOff)
{
  Counters c;
  counters_update_land_inhibit(c, State::APPROACH, State::FOLLOW, make_rc(false));
  EXPECT_FALSE(c.land_inhibit);
}

TEST(Counters, LandInhibitNotSetFromOtherStates)
{
  Counters c;
  counters_update_land_inhibit(c, State::FOLLOW, State::SEARCH, make_rc(true));
  EXPECT_FALSE(c.land_inhibit);
}

TEST(Counters, LandInhibitClearsWhenSwitchOff)
{
  Counters c;
  c.land_inhibit = true;
  counters_update_land_inhibit(c, State::FOLLOW, State::FOLLOW, make_rc(true));
  EXPECT_TRUE(c.land_inhibit);
  counters_update_land_inhibit(c, State::FOLLOW, State::FOLLOW, make_rc(false));
  EXPECT_FALSE(c.land_inhibit);
}

TEST(Counters, ResetClearsStableAndLandOkKeepsLostTime)
{
  Counters c;
  c.marker_stable_count = 7;
  c.land_ok_count = 3;
  c.marker_lost_time = 1.2f;
  counters_reset(c);
  EXPECT_EQ(c.marker_stable_count, 0);
  EXPECT_EQ(c.land_ok_count, 0);
  EXPECT_NEAR(c.marker_lost_time, 1.2f, 1e-5f);
}

TEST(CheckSearch, TransitionAtThreshold)
{
  FsmParams p;
  Counters c;
  c.marker_stable_count = p.enter_follow_cycles - 1;
  EXPECT_FALSE(check_search_transition(c, p).has_value());
  c.marker_stable_count = p.enter_follow_cycles;
  EXPECT_EQ(check_search_transition(c, p), State::FOLLOW);
  c.marker_stable_count = p.enter_follow_cycles + 5;
  EXPECT_EQ(check_search_transition(c, p), State::FOLLOW);
}

TEST(CheckFollow, LostTimeoutGoesToSearch)
{
  FsmParams p;
  Counters c;
  c.marker_lost_time = p.follow_lost_timeout - 0.1f;
  EXPECT_FALSE(check_follow_transition(c, make_rc(false), lost_marker(), p).has_value());
  c.marker_lost_time = p.follow_lost_timeout + 0.1f;
  EXPECT_EQ(check_follow_transition(c, make_rc(false), lost_marker(), p), State::SEARCH);
}

TEST(CheckFollow, LandSwitchWithMarkerGoesToApproach)
{
  FsmParams p;
  Counters c;
  EXPECT_EQ(check_follow_transition(c, make_rc(true), good_follow(), p), State::APPROACH);
}

TEST(CheckFollow, LandSwitchWithoutMarkerStays)
{
  FsmParams p;
  Counters c;
  EXPECT_FALSE(check_follow_transition(c, make_rc(true), lost_marker(), p).has_value());
}

TEST(CheckFollow, NoLandSwitchStays)
{
  FsmParams p;
  Counters c;
  EXPECT_FALSE(check_follow_transition(c, make_rc(false), good_follow(), p).has_value());
}

TEST(CheckFollow, InhibitBlocksApproachWhenRearmRequired)
{
  FsmParams p;
  Counters c;
  c.land_inhibit = true;
  EXPECT_FALSE(check_follow_transition(c, make_rc(true), good_follow(), p).has_value());
}

TEST(CheckFollow, InhibitIgnoredWhenRearmNotRequired)
{
  FsmParams p;
  p.land_requires_rearm = false;
  Counters c;
  c.land_inhibit = true;
  EXPECT_EQ(check_follow_transition(c, make_rc(true), good_follow(), p), State::APPROACH);
}

TEST(CheckApproach, SwitchOffGoesToFollow)
{
  FsmParams p;
  Counters c;
  EXPECT_EQ(check_approach_transition(c, make_rc(false), p), State::FOLLOW);
}

TEST(CheckApproach, LostTimeoutGoesToFollow)
{
  FsmParams p;
  Counters c;
  c.marker_lost_time = p.approach_lost_timeout - 0.1f;
  EXPECT_FALSE(check_approach_transition(c, make_rc(true), p).has_value());
  c.marker_lost_time = p.approach_lost_timeout + 0.1f;
  EXPECT_EQ(check_approach_transition(c, make_rc(true), p), State::FOLLOW);
}

TEST(CheckApproach, LandOkCountGoesToLand)
{
  FsmParams p;
  Counters c;
  c.land_ok_count = p.land_entry_cycles - 1;
  EXPECT_FALSE(check_approach_transition(c, make_rc(true), p).has_value());
  c.land_ok_count = p.land_entry_cycles;
  EXPECT_EQ(check_approach_transition(c, make_rc(true), p), State::LAND);
}

TEST(CheckLand, TouchdownGoesToComplete)
{
  EXPECT_EQ(check_land_transition(touchdown_sensor()), State::COMPLETE);
  EXPECT_FALSE(check_land_transition(good_land()).has_value());
}

TEST(EvaluateTransition, DispatchesPerState)
{
  FsmParams p;
  Counters c;
  c.marker_stable_count = p.enter_follow_cycles;
  EXPECT_EQ(evaluate_transition(State::SEARCH, c, make_rc(false), good_follow(), p), State::FOLLOW);

  Counters f;
  EXPECT_EQ(evaluate_transition(State::FOLLOW, f, make_rc(true), good_follow(), p), State::APPROACH);

  Counters a;
  a.land_ok_count = p.land_entry_cycles;
  EXPECT_EQ(evaluate_transition(State::APPROACH, a, make_rc(true), good_land(), p), State::LAND);

  Counters l;
  EXPECT_EQ(
    evaluate_transition(State::LAND, l, make_rc(true), touchdown_sensor(), p), State::COMPLETE);
  EXPECT_FALSE(evaluate_transition(State::LAND, l, make_rc(true), good_land(), p).has_value());
}

TEST(EvaluateTransition, CompleteIsTerminal)
{
  FsmParams p;
  Counters c;
  c.marker_stable_count = 100;
  c.land_ok_count = 100;
  EXPECT_FALSE(
    evaluate_transition(State::COMPLETE, c, make_rc(false), touchdown_sensor(), p).has_value());
}

TEST(EffectiveRc, MergesForceLand)
{
  EXPECT_FALSE(effective_rc_input(make_rc(false), false).land_switch);
  EXPECT_TRUE(effective_rc_input(make_rc(false), true).land_switch);
  EXPECT_TRUE(effective_rc_input(make_rc(true), false).land_switch);
  EXPECT_TRUE(effective_rc_input(make_rc(true), true).land_switch);
}

TEST(EffectiveRc, KeepsKillSwitch)
{
  EXPECT_TRUE(effective_rc_input(make_rc(false, true), false).kill_switch);
  EXPECT_FALSE(effective_rc_input(make_rc(false, false), true).kill_switch);
}

TEST(ForceLand, TransitionPerState)
{
  for (State s : {State::SEARCH, State::FOLLOW, State::APPROACH}) {
    EXPECT_EQ(force_land_transition(s, true), State::LAND);
    EXPECT_FALSE(force_land_transition(s, false).has_value());
  }
  for (State s : {State::LAND, State::COMPLETE}) {
    EXPECT_FALSE(force_land_transition(s, true).has_value());
    EXPECT_FALSE(force_land_transition(s, false).has_value());
  }
}

TEST(PlannerTimeout, FollowAndApproachGoToSearch)
{
  FsmParams p;
  EXPECT_EQ(planner_timeout_transition(State::FOLLOW, true, p), State::SEARCH);
  EXPECT_EQ(planner_timeout_transition(State::APPROACH, true, p), State::SEARCH);
}

TEST(PlannerTimeout, OtherStatesUnaffected)
{
  FsmParams p;
  for (State s : {State::SEARCH, State::LAND, State::COMPLETE}) {
    EXPECT_FALSE(planner_timeout_transition(s, true, p).has_value());
  }
}

TEST(PlannerTimeout, NoTimeoutNoTransition)
{
  FsmParams p;
  for (State s : {State::SEARCH, State::FOLLOW, State::APPROACH, State::LAND, State::COMPLETE}) {
    EXPECT_FALSE(planner_timeout_transition(s, false, p).has_value());
  }
}

TEST(PlannerTimeout, IgnoreFlagReturnsNullopt)
{
  FsmParams p;
  p.ignore_planner_timeout = true;
  for (State s : {State::SEARCH, State::FOLLOW, State::APPROACH, State::LAND, State::COMPLETE}) {
    EXPECT_FALSE(planner_timeout_transition(s, true, p).has_value());
  }
}

TEST(Scenario, SearchToFollowNeedsEnoughCycles)
{
  Harness h;
  h.run(good_follow(), make_rc(false), h.p.enter_follow_cycles - 1);
  EXPECT_EQ(h.ctx.state, State::SEARCH);
  h.run(good_follow(), make_rc(false), 2);
  EXPECT_EQ(h.ctx.state, State::FOLLOW);
}

TEST(Scenario, OneBadCycleRestartsStableCount)
{
  Harness h;
  h.run(good_follow(), make_rc(false), h.p.enter_follow_cycles - 2);
  h.run(lost_marker(), make_rc(false), 1);
  h.run(good_follow(), make_rc(false), h.p.enter_follow_cycles - 2);
  EXPECT_EQ(h.ctx.state, State::SEARCH);
  h.run(good_follow(), make_rc(false), 3);
  EXPECT_EQ(h.ctx.state, State::FOLLOW);
}

TEST(Scenario, HighYawRateKeepsSearch)
{
  Harness h;
  h.run(make_sensor(true, 0.1f, 2.0f, 1.0f, 0.5f, 0.3f), make_rc(false), 50);
  EXPECT_EQ(h.ctx.state, State::SEARCH);
}

TEST(Scenario, NanYawRateKeepsSearch)
{
  Harness h;
  h.run(make_sensor(true, 0.1f, 2.0f, 1.0f, 0.5f, kNan), make_rc(false), 50);
  EXPECT_EQ(h.ctx.state, State::SEARCH);
}

TEST(Scenario, NanDeltaHKeepsSearch)
{
  Harness h;
  h.run(make_sensor(true, 0.1f, 2.0f, kNan), make_rc(false), 50);
  EXPECT_EQ(h.ctx.state, State::SEARCH);
}

TEST(Scenario, NanAlignOrAltitudeKeepsSearch)
{
  Harness h1;
  h1.run(make_sensor(true, kNan), make_rc(false), 50);
  EXPECT_EQ(h1.ctx.state, State::SEARCH);
  Harness h2;
  h2.run(make_sensor(true, 0.1f, kNan), make_rc(false), 50);
  EXPECT_EQ(h2.ctx.state, State::SEARCH);
}

TEST(Scenario, FollowToSearchAfterLostTimeout)
{
  Harness h;
  h.to_follow();
  ASSERT_EQ(h.ctx.state, State::FOLLOW);
  h.run(lost_marker(), make_rc(false), 20);
  EXPECT_EQ(h.ctx.state, State::FOLLOW);
  h.run(lost_marker(), make_rc(false), 10);
  EXPECT_EQ(h.ctx.state, State::SEARCH);
}

TEST(Scenario, FollowToApproach)
{
  Harness h;
  h.to_approach();
  EXPECT_EQ(h.ctx.state, State::APPROACH);
}

TEST(Scenario, ApproachBackToFollowWhenSwitchOff)
{
  Harness h;
  h.to_approach();
  h.run(good_follow(), make_rc(false), 1);
  EXPECT_EQ(h.ctx.state, State::FOLLOW);
  EXPECT_FALSE(h.ctx.counters.land_inhibit);
}

TEST(Scenario, ApproachToFollowOnLostTimeoutSetsInhibit)
{
  Harness h;
  h.to_approach();
  h.run(lost_marker(), make_rc(true), 14);
  EXPECT_EQ(h.ctx.state, State::APPROACH);
  h.run(lost_marker(), make_rc(true), 4);
  EXPECT_EQ(h.ctx.state, State::FOLLOW);
  EXPECT_TRUE(h.ctx.counters.land_inhibit);
}

TEST(Scenario, LandSwitchDoesNotRevive)
{
  Harness h;
  h.to_approach();
  h.run(lost_marker(), make_rc(true), 18);
  ASSERT_EQ(h.ctx.state, State::FOLLOW);
  h.run(good_follow(), make_rc(true), 10);
  EXPECT_EQ(h.ctx.state, State::FOLLOW);
  h.run(good_follow(), make_rc(false), 1);
  EXPECT_FALSE(h.ctx.counters.land_inhibit);
  h.run(good_follow(), make_rc(true), 1);
  EXPECT_EQ(h.ctx.state, State::APPROACH);
}

TEST(Scenario, RearmNotRequiredAllowsImmediateReapproach)
{
  Harness h;
  h.p.land_requires_rearm = false;
  h.to_approach();
  h.run(lost_marker(), make_rc(true), 18);
  ASSERT_EQ(h.ctx.state, State::FOLLOW);
  h.run(good_follow(), make_rc(true), 2);
  EXPECT_EQ(h.ctx.state, State::APPROACH);
}

TEST(Scenario, PlannerTimeoutInApproachGoesToSearchAndInhibits)
{
  Harness h;
  h.to_approach();
  h.run(good_follow(), make_rc(true), 1, false, true);
  EXPECT_EQ(h.ctx.state, State::SEARCH);
  EXPECT_TRUE(h.ctx.counters.land_inhibit);
}

TEST(Scenario, PlannerTimeoutInFollowGoesToSearch)
{
  Harness h;
  h.to_follow();
  h.run(good_follow(), make_rc(false), 1, false, true);
  EXPECT_EQ(h.ctx.state, State::SEARCH);
}

TEST(Scenario, IgnorePlannerTimeoutKeepsFollow)
{
  Harness h;
  h.p.ignore_planner_timeout = true;
  h.to_follow();
  h.run(good_follow(), make_rc(false), 10, false, true);
  EXPECT_EQ(h.ctx.state, State::FOLLOW);
}

TEST(Scenario, LandOkMustBeConsecutive)
{
  Harness h;
  h.to_approach();
  h.run(good_land(), make_rc(true), 1);
  h.run(good_follow(), make_rc(true), 1);
  h.run(good_land(), make_rc(true), h.p.land_entry_cycles - 1);
  EXPECT_EQ(h.ctx.state, State::APPROACH);
  h.run(good_land(), make_rc(true), 2);
  EXPECT_EQ(h.ctx.state, State::LAND);
}

TEST(Scenario, ApproachToLandAfterEnoughCycles)
{
  Harness h;
  h.to_approach();
  h.run(good_land(), make_rc(true), h.p.land_entry_cycles - 1);
  EXPECT_EQ(h.ctx.state, State::APPROACH);
  h.run(good_land(), make_rc(true), 2);
  EXPECT_EQ(h.ctx.state, State::LAND);
}

TEST(Scenario, LandToCompleteOnTouchdown)
{
  Harness h;
  h.to_land();
  ASSERT_EQ(h.ctx.state, State::LAND);
  h.run(good_land(), make_rc(true), 5);
  EXPECT_EQ(h.ctx.state, State::LAND);
  h.run(touchdown_sensor(), make_rc(true), 1);
  EXPECT_EQ(h.ctx.state, State::COMPLETE);
}

TEST(Scenario, CompleteIsTerminal)
{
  Harness h;
  h.to_land();
  h.run(touchdown_sensor(), make_rc(true), 1);
  ASSERT_EQ(h.ctx.state, State::COMPLETE);
  h.run(good_follow(), make_rc(false), 30);
  h.run(good_follow(), make_rc(true), 30, true);
  h.run(good_follow(), make_rc(true), 30, false, true);
  EXPECT_EQ(h.ctx.state, State::COMPLETE);
}

TEST(Scenario, ForceLandFromEveryActiveState)
{
  {
    Harness h;
    h.run(good_follow(), make_rc(false), 1, true);
    EXPECT_EQ(h.ctx.state, State::LAND);
  }
  {
    Harness h;
    h.to_follow();
    h.run(good_follow(), make_rc(false), 1, true);
    EXPECT_EQ(h.ctx.state, State::LAND);
  }
  {
    Harness h;
    h.to_approach();
    h.run(good_follow(), make_rc(true), 1, true);
    EXPECT_EQ(h.ctx.state, State::LAND);
  }
}

TEST(Scenario, ForceLandBeatsPlannerTimeout)
{
  Harness h;
  h.to_follow();
  h.run(good_follow(), make_rc(false), 1, true, true);
  EXPECT_EQ(h.ctx.state, State::LAND);
}

TEST(Scenario, ForceLandFromApproachDoesNotSetInhibit)
{
  Harness h;
  h.to_approach();
  h.run(good_follow(), make_rc(true), 1, true);
  EXPECT_EQ(h.ctx.state, State::LAND);
  EXPECT_FALSE(h.ctx.counters.land_inhibit);
}

TEST(Scenario, ForceLandThenTouchdownCompletes)
{
  Harness h;
  h.to_follow();
  h.run(good_follow(), make_rc(false), 1, true);
  h.run(touchdown_sensor(), make_rc(false), 1, true);
  EXPECT_EQ(h.ctx.state, State::COMPLETE);
}

TEST(Scenario, TransitionResetsStableAndLandOkNotLostTime)
{
  Harness h;
  h.to_approach();
  h.ctx.counters.marker_lost_time = 1.0f;
  h.run(good_follow(), make_rc(false), 1);
  ASSERT_EQ(h.ctx.state, State::FOLLOW);
  EXPECT_EQ(h.ctx.counters.land_ok_count, 0);
  EXPECT_EQ(h.ctx.counters.marker_stable_count, 0);
}

TEST(Scenario, FullFlightSequence)
{
  Harness h;
  std::vector<State> seen;
  auto record = [&]() {
      if (seen.empty() || seen.back() != h.ctx.state) {
        seen.push_back(h.ctx.state);
      }
    };
  seen.push_back(h.ctx.state);
  for (int i = 0; i < 15; ++i) {
    h.step(good_follow(), make_rc(false));
    record();
  }
  for (int i = 0; i < 3; ++i) {
    h.step(good_follow(), make_rc(true));
    record();
  }
  for (int i = 0; i < 10; ++i) {
    h.step(good_land(), make_rc(true));
    record();
  }
  for (int i = 0; i < 3; ++i) {
    h.step(touchdown_sensor(), make_rc(true));
    record();
  }
  std::vector<State> expected = {
    State::SEARCH, State::FOLLOW, State::APPROACH, State::LAND, State::COMPLETE};
  EXPECT_EQ(seen, expected);
}