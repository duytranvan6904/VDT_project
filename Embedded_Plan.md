# Pseudocode Module Embedded/Integration — Việt Anh

Phạm vi: State Machine 4 trạng thái, Gimbal PWM + PID smoothing, Offboard mode, micro-XRCE-DDS, RC parsing, Kill switch.

---

## 1. State Machine (FSM 4 states)

### 1.1. Data structures

```
enum State { SEARCH, FOLLOW, APPROACH, LAND, COMPLETE }

struct SensorInput {
  marker_detected: bool
  align_error: float
  altitude: float
  delta_h: float
  d_horiz: float
  touchdown: bool
  valid: bool
}

struct RCInput {
  land_switch: bool
  kill_switch: bool
}

struct Counters {
  marker_stable_count: int
  marker_lost_time: float
}

struct FSMContext {
  state: State
  counters: Counters
  last_transition_time: float
}
```

### 1.2. Đọc và xử lý dữ liệu cảm biến đầu vào

```
function sensor_read(ekf_state, vision_state, alt_estimator) -> SensorInput
  s = SensorInput{}
  s.marker_detected = vision_state.marker_visible
  s.align_error = vision_state.pixel_align_error
  s.altitude = alt_estimator.altitude
  s.delta_h = ekf_state.z - alt_estimator.altitude
  s.d_horiz = norm(ekf_state.x, ekf_state.y)
  s.touchdown = alt_estimator.touchdown_flag
  s.valid = true
  return s
```

```
function sensor_validate(s, timeout_flags) -> SensorInput
  if timeout_flags.vision_timeout or timeout_flags.ekf_timeout:
    s.valid = false
    s.marker_detected = false
  if is_nan(s.align_error) or is_nan(s.altitude):
    s.valid = false
  return s
```

### 1.3. Xử lý dữ liệu RC cho FSM

```
function rc_fsm_extract(channels) -> RCInput
  r = RCInput{}
  r.land_switch = rc_get_land_trigger(channels)
  r.kill_switch = rc_get_kill_switch(channels)
  return r
```

### 1.4. Đếm thời gian / bộ đếm trạng thái

```
function counters_update_marker_stable(counters, marker_detected)
  if marker_detected:
    counters.marker_stable_count += 1
  else:
    counters.marker_stable_count = 0
```

```
function counters_update_marker_lost(counters, marker_detected, dt)
  if marker_detected:
    counters.marker_lost_time = 0
  else:
    counters.marker_lost_time += dt
```

```
function counters_reset(counters)
  counters.marker_stable_count = 0
  counters.marker_lost_time = 0
```

### 1.5. Logic chuyển trạng thái (tách riêng theo từng state)

```
function check_search_transition(counters) -> State or None
  if counters.marker_stable_count >= 10:
    return FOLLOW
  return None
```

```
function check_follow_transition(counters, rc, sensors) -> State or None
  if counters.marker_lost_time > 5.0:
    return SEARCH
  if rc.land_switch and sensors.marker_detected:
    return APPROACH
  return None
```

```
function check_approach_transition(counters, rc, sensors) -> State or None
  if not rc.land_switch:
    return FOLLOW
  if counters.marker_lost_time > 3.0:
    return FOLLOW
  if sensors.align_error < 0.3 and sensors.altitude < 0.5:
    return LAND
  return None
```

```
function check_land_transition(sensors) -> State or None
  if sensors.touchdown:
    return COMPLETE
  return None
```

### 1.6. Hành động cụ thể của từng trạng thái

```
function action_search(ctx)
  set_yaw_rate(YAW_SEARCH_RATE)
  set_gimbal_state_request(SEARCH)
```

```
function action_follow(ctx, sensors)
  set_planner_mode(FOLLOW)
  set_apf_gain(1.0)
  set_gimbal_state_request(FOLLOW)
```

```
function action_approach(ctx, sensors)
  set_planner_mode(APPROACH)
  set_apf_gain(0.5)
  set_gimbal_state_request(APPROACH)
  align_marker_center(sensors.align_error)
```

```
function action_land(ctx, sensors)
  set_planner_mode(LAND)
  set_apf_gain(0.0)
  set_gimbal_state_request(LAND)
  set_vertical_descent_rate(LAND_DESCENT_RATE)
  if sensors.touchdown:
    disarm_request()
```

### 1.7. Vòng lặp chính — ghép các khối trên

```
function fsm_init() -> FSMContext
  ctx = FSMContext{state=SEARCH, counters=Counters{0,0}}
  return ctx
```

```
function fsm_update(ctx, ekf_state, vision_state, alt_estimator, channels, timeout_flags, dt) -> State
  s = sensor_read(ekf_state, vision_state, alt_estimator)
  s = sensor_validate(s, timeout_flags)
  rc = rc_fsm_extract(channels)

  counters_update_marker_stable(ctx.counters, s.marker_detected)
  counters_update_marker_lost(ctx.counters, s.marker_detected, dt)

  next_state = None
  switch ctx.state:
    case SEARCH:   next_state = check_search_transition(ctx.counters)
    case FOLLOW:   next_state = check_follow_transition(ctx.counters, rc, s)
    case APPROACH: next_state = check_approach_transition(ctx.counters, rc, s)
    case LAND:     next_state = check_land_transition(s)

  if next_state is not None and next_state != ctx.state:
    ctx.state = next_state
    counters_reset(ctx.counters)
    ctx.last_transition_time = now()

  switch ctx.state:
    case SEARCH:   action_search(ctx)
    case FOLLOW:   action_follow(ctx, s)
    case APPROACH: action_approach(ctx, s)
    case LAND:     action_land(ctx, s)

  return ctx.state
```

### 1.8. Debug

```
function fsm_debug_log(ctx, s, rc)
  print(timestamp(), ctx.state, ctx.counters.marker_stable_count,
        ctx.counters.marker_lost_time, s.align_error, s.altitude, rc.land_switch)
```

---

## 2. Servo PWM control (Gimbal actuator)

### 2.1. Data structures

```
struct ServoConfig {
  gpio_pin: int
  pwm_freq_hz: int
  pwm_min_us: int
  pwm_max_us: int
  angle_min_deg: float
  angle_max_deg: float
  home_angle_deg: float
}

struct ServoState {
  current_pwm_us: int
  current_angle_deg: float
  initialized: bool
}
```

### 2.2. Khởi tạo và đưa về home position

```
function servo_init(cfg) -> ServoHandle, ServoState
  handle = gpio_pwm_open(cfg.gpio_pin, cfg.pwm_freq_hz)
  state = ServoState{initialized=false}
  servo_write(handle, cfg.home_angle_deg, cfg, state)
  state.initialized = true
  return handle, state
```

### 2.3. Chuyển đổi góc sang độ rộng xung PWM

```
function angle_to_pwm(angle_deg, cfg) -> int
  angle_clamped = clamp(angle_deg, cfg.angle_min_deg, cfg.angle_max_deg)
  ratio = (angle_clamped - cfg.angle_min_deg) / (cfg.angle_max_deg - cfg.angle_min_deg)
  return cfg.pwm_min_us + ratio * (cfg.pwm_max_us - cfg.pwm_min_us)
```

### 2.4. Ghi giá trị ra phần cứng

```
function servo_write(handle, angle_deg, cfg, state)
  pwm_us = angle_to_pwm(angle_deg, cfg)
  gpio_pwm_set_pulse(handle, pwm_us)
  state.current_pwm_us = pwm_us
  state.current_angle_deg = angle_deg
```

### 2.5. Đọc phản hồi vị trí thực tế (nếu servo có feedback)

```
function servo_read_feedback(adc_handle, cfg) -> float
  raw = adc_read(adc_handle)
  return map_range(raw, cfg.feedback_min, cfg.feedback_max,
                    cfg.angle_min_deg, cfg.angle_max_deg)
```

### 2.6. Debug

```
function servo_debug_log(state)
  print(timestamp(), state.current_angle_deg, state.current_pwm_us)
```

---

## 3. Multi-phase gimbal angle với PID smoothing

### 3.1. Data structures

```
struct PIDState {
  kp: float
  ki: float
  kd: float
  integral: float
  prev_error: float
  out_min: float
  out_max: float
}

struct GimbalTelemetry {
  delta_h: float
  d_horiz: float
  altitude: float
  current_gimbal_angle: float
  valid: bool
}

struct GimbalContext {
  pid: PIDState
  current_angle: float
  last_state: State
  max_slew_rate_deg_s: float
}
```

### 3.2. Đọc và xử lý dữ liệu telemetry đầu vào

```
function gimbal_telemetry_read(ekf_state, alt_estimator, servo_feedback) -> GimbalTelemetry
  t = GimbalTelemetry{}
  t.delta_h = ekf_state.z - alt_estimator.altitude
  t.d_horiz = norm(ekf_state.x, ekf_state.y)
  t.altitude = alt_estimator.altitude
  t.current_gimbal_angle = servo_feedback.angle_deg
  t.valid = true
  return t
```

```
function gimbal_telemetry_validate(t, timeout_flags) -> GimbalTelemetry
  if timeout_flags.ekf_timeout or timeout_flags.alt_timeout:
    t.valid = false
  if is_nan(t.delta_h) or is_nan(t.d_horiz):
    t.valid = false
  return t
```

### 3.3. Tính góc mục tiêu theo từng phase (tách riêng từng hàm)

```
function target_angle_search() -> float
  return 0.0
```

```
function target_angle_follow(t) -> float
  return -atan2(t.delta_h, t.d_horiz)
```

```
function target_angle_approach(t) -> float
  return -atan2(t.d_horiz, t.altitude)
```

```
function target_angle_land(t) -> float
  ratio = clamp(t.altitude / 0.5, 0.0, 1.0)
  return lerp(-60.0, -90.0, ratio)
```

```
function gimbal_target_angle(state, t) -> float
  switch state:
    case SEARCH:   return target_angle_search()
    case FOLLOW:   return target_angle_follow(t)
    case APPROACH: return target_angle_approach(t)
    case LAND:     return target_angle_land(t)
```

### 3.4. Bộ lọc làm mượt PID

```
function pid_init(kp, ki, kd, out_min, out_max) -> PIDState
  return PIDState{kp=kp, ki=ki, kd=kd, integral=0, prev_error=0,
                  out_min=out_min, out_max=out_max}
```

```
function pid_reset(pid)
  pid.integral = 0.0
  pid.prev_error = 0.0
```

```
function pid_update(pid, target, current, dt) -> float
  error = target - current
  pid.integral += error * dt
  derivative = (error - pid.prev_error) / dt
  output = pid.kp*error + pid.ki*pid.integral + pid.kd*derivative
  output = clamp(output, pid.out_min, pid.out_max)
  pid.prev_error = error
  return current + output * dt
```

### 3.5. Giới hạn tốc độ đổi góc (slew rate limiter)

```
function clamp_slew_rate(prev_angle, new_angle, max_rate_deg_s, dt) -> float
  max_delta = max_rate_deg_s * dt
  delta = clamp(new_angle - prev_angle, -max_delta, max_delta)
  return prev_angle + delta
```

### 3.6. Vòng lặp điều khiển gimbal — ghép các khối trên

```
function gimbal_init(kp, ki, kd, max_slew_rate) -> GimbalContext
  ctx = GimbalContext{}
  ctx.pid = pid_init(kp, ki, kd, out_min=-90.0, out_max=90.0)
  ctx.current_angle = 0.0
  ctx.last_state = SEARCH
  ctx.max_slew_rate_deg_s = max_slew_rate
  return ctx
```

```
function gimbal_control_step(ctx, state, ekf_state, alt_estimator, servo_feedback,
                              timeout_flags, servo_handle, servo_cfg, servo_state, dt)
  t = gimbal_telemetry_read(ekf_state, alt_estimator, servo_feedback)
  t = gimbal_telemetry_validate(t, timeout_flags)
  if not t.valid:
    return ctx.current_angle

  if state != ctx.last_state:
    pid_reset(ctx.pid)
    ctx.last_state = state

  target = gimbal_target_angle(state, t)
  smoothed = pid_update(ctx.pid, target, ctx.current_angle, dt)
  limited = clamp_slew_rate(ctx.current_angle, smoothed, ctx.max_slew_rate_deg_s, dt)

  servo_write(servo_handle, limited, servo_cfg, servo_state)
  ctx.current_angle = limited
  gimbal_debug_log(state, target, smoothed, limited)
  return limited
```

### 3.7. Debug

```
function gimbal_debug_log(state, target, smoothed, limited)
  print(timestamp(), state, target, smoothed, limited)
```

---

## 4. Offboard mode manager

### 4.1. Data structures

```
enum SetpointType { POSITION, VELOCITY }

struct Setpoint {
  type: SetpointType
  x: float
  y: float
  z: float
  vx: float
  vy: float
  vz: float
  yaw: float
}

struct OffboardContext {
  last_heartbeat_time: float
  armed: bool
  offboard_active: bool
  engage_counter: int
  setpoint: Setpoint
}
```

### 4.2. Xây dựng setpoint theo state và output từ planner

```
function build_setpoint_search() -> Setpoint
  return Setpoint{type=VELOCITY, vx=0, vy=0, vz=0, yaw_rate=YAW_SEARCH_RATE}
```

```
function build_setpoint_follow(planner_output) -> Setpoint
  return Setpoint{type=VELOCITY, vx=planner_output.vx, vy=planner_output.vy,
                  vz=planner_output.vz, yaw=planner_output.yaw}
```

```
function build_setpoint_approach(planner_output) -> Setpoint
  return Setpoint{type=VELOCITY, vx=planner_output.vx, vy=planner_output.vy,
                  vz=planner_output.vz, yaw=planner_output.yaw}
```

```
function build_setpoint_land(planner_output) -> Setpoint
  return Setpoint{type=VELOCITY, vx=0, vy=0,
                  vz=LAND_DESCENT_RATE, yaw=planner_output.yaw}
```

```
function build_setpoint(state, planner_output) -> Setpoint
  switch state:
    case SEARCH:   return build_setpoint_search()
    case FOLLOW:   return build_setpoint_follow(planner_output)
    case APPROACH: return build_setpoint_approach(planner_output)
    case LAND:     return build_setpoint_land(planner_output)
```

### 4.3. Gửi heartbeat (offboard control mode)

```
function offboard_send_heartbeat(ctx)
  publish_offboard_control_mode(position=true, velocity=true)
  ctx.last_heartbeat_time = now()
```

### 4.4. Gửi setpoint

```
function offboard_set_setpoint(ctx, setpoint)
  ctx.setpoint = setpoint
  publish_trajectory_setpoint(setpoint)
```

### 4.5. Watchdog giám sát và failsafe

```
function offboard_watchdog_check(ctx, timeout=0.5) -> bool
  if now() - ctx.last_heartbeat_time > timeout:
    offboard_enter_failsafe(ctx)
    return false
  return true
```

```
function offboard_enter_failsafe(ctx)
  ctx.offboard_active = false
  publish_vehicle_command(CMD_SET_MODE, mode=HOLD)
```

### 4.6. Chuỗi engage Offboard mode (yêu cầu riêng của PX4)

```
function offboard_engage_request(ctx, required_cycles=10)
  if ctx.offboard_active:
    return
  offboard_send_heartbeat(ctx)
  offboard_set_setpoint(ctx, build_setpoint_search())
  ctx.engage_counter += 1
  if ctx.engage_counter >= required_cycles:
    publish_vehicle_command(CMD_SET_MODE, mode=OFFBOARD)
    publish_vehicle_command(CMD_COMPONENT_ARM_DISARM, param1=1)
    ctx.offboard_active = true
    ctx.armed = true
```

### 4.7. Vòng lặp chính — ghép các khối trên

```
function offboard_init() -> OffboardContext
  return OffboardContext{last_heartbeat_time=0, armed=false,
                          offboard_active=false, engage_counter=0}
```

```
function offboard_main_loop(ctx, fsm_state, planner_output, rate=20Hz)
  loop at rate:
    if not ctx.offboard_active:
      offboard_engage_request(ctx)
    else:
      offboard_send_heartbeat(ctx)
      if offboard_watchdog_check(ctx):
        setpoint = build_setpoint(fsm_state, planner_output)
        offboard_set_setpoint(ctx, setpoint)
    offboard_debug_log(ctx)
```

### 4.8. Debug

```
function offboard_debug_log(ctx)
  print(timestamp(), ctx.offboard_active, ctx.armed, ctx.engage_counter, ctx.setpoint)
```

---

## 5. micro-XRCE-DDS setup

### 5.1. Data structures

```
enum TopicDirection { PUB, SUB }

struct TopicConfig {
  name: string
  type: string
  direction: TopicDirection
}

struct XRCEContext {
  handle: XRCEHandle
  participant: ParticipantHandle
  connected: bool
  last_check_time: float
  retry_count: int
}
```

### 5.2. Khởi động Agent (cầu nối serial trên Pi 5)

```
function xrce_agent_start(serial_port, baudrate)
  if not xrce_agent_is_running(serial_port):
    spawn_process("MicroXRCEAgent serial --dev " + serial_port + " -b " + baudrate)
```

```
function xrce_agent_is_running(serial_port) -> bool
  return process_exists("MicroXRCEAgent", serial_port)
```

### 5.3. Khởi tạo client và participant

```
function xrce_client_init() -> XRCEHandle
  handle = uxrce_dds_client_init()
  return handle
```

```
function xrce_create_participant(handle, namespace) -> ParticipantHandle
  return uxrce_create_participant(handle, namespace)
```

### 5.4. Đăng ký danh sách topic (publisher/subscriber)

```
function xrce_topic_list() -> list of TopicConfig
  return [
    TopicConfig("offboard_control_mode", "OffboardControlMode", PUB),
    TopicConfig("trajectory_setpoint", "TrajectorySetpoint", PUB),
    TopicConfig("vehicle_command", "VehicleCommand", PUB),
    TopicConfig("vehicle_odometry", "VehicleOdometry", SUB),
    TopicConfig("battery_status", "BatteryStatus", SUB),
  ]
```

```
function xrce_create_topics(participant, topic_list)
  for topic in topic_list:
    if topic.direction == PUB:
      uxrce_create_publisher(participant, topic.name, topic.type)
    else:
      uxrce_create_subscriber(participant, topic.name, topic.type)
```

### 5.5. Giám sát kết nối và tự phục hồi

```
function xrce_check_connection(handle) -> bool
  return uxrce_ping(handle, timeout=1.0)
```

```
function xrce_reconnect(ctx, serial_port, baudrate, namespace)
  ctx.retry_count += 1
  xrce_agent_start(serial_port, baudrate)
  ctx.handle = xrce_client_init()
  ctx.participant = xrce_create_participant(ctx.handle, namespace)
  xrce_create_topics(ctx.participant, xrce_topic_list())
  ctx.connected = xrce_check_connection(ctx.handle)
```

```
function xrce_monitor_loop(ctx, serial_port, baudrate, namespace, rate=1Hz)
  loop at rate:
    ctx.connected = xrce_check_connection(ctx.handle)
    if not ctx.connected:
      xrce_reconnect(ctx, serial_port, baudrate, namespace)
    ctx.last_check_time = now()
    xrce_debug_status(ctx)
```

### 5.6. Thiết lập toàn bộ pipeline — ghép các khối trên

```
function xrce_setup_all(serial_port, baudrate, namespace) -> XRCEContext
  ctx = XRCEContext{retry_count=0}
  xrce_agent_start(serial_port, baudrate)
  ctx.handle = xrce_client_init()
  ctx.participant = xrce_create_participant(ctx.handle, namespace)
  xrce_create_topics(ctx.participant, xrce_topic_list())
  ctx.connected = xrce_check_connection(ctx.handle)
  return ctx
```

### 5.7. Debug

```
function xrce_debug_status(ctx)
  print(timestamp(), ctx.connected, ctx.retry_count)
```

---

## 6. RC channel parsing

### 6.1. Data structures

```
enum SwitchPos { LOW, MID, HIGH }

struct RCConfig {
  land_channel: int
  kill_channel: int
  low_threshold: int
  high_threshold: int
  frame_timeout: float
}

struct RCChannels {
  ch: array[16] of int
  valid: bool
  failsafe: bool
  timestamp: float
}
```

### 6.2. Đọc frame thô từ RC receiver

```
function rc_read_raw_frame(uart_handle) -> RawFrame
  return uart_read_packet(uart_handle, protocol=SBUS)
```

### 6.3. Parse frame thô thành cấu trúc kênh

```
function rc_parse_frame(raw_frame) -> RCChannels
  ch_out = RCChannels{}
  for i in 0..raw_frame.channel_count:
    ch_out.ch[i] = raw_frame.values[i]
  ch_out.failsafe = raw_frame.failsafe_flag
  ch_out.timestamp = now()
  ch_out.valid = true
  return ch_out
```

### 6.4. Kiểm tra tính hợp lệ của tín hiệu

```
function rc_validate(channels, cfg, last_valid_time) -> RCChannels
  if now() - last_valid_time > cfg.frame_timeout:
    channels.valid = false
  if channels.failsafe:
    channels.valid = false
  for v in channels.ch:
    if v < 800 or v > 2200:
      channels.valid = false
  return channels
```

### 6.5. Chuyển giá trị PWM thành vị trí switch

```
function rc_get_switch_pos(channels, ch_idx, cfg) -> SwitchPos
  val = channels.ch[ch_idx]
  if val < cfg.low_threshold: return LOW
  if val > cfg.high_threshold: return HIGH
  return MID
```

### 6.6. Trích xuất tín hiệu chức năng cụ thể

```
function rc_get_land_trigger(channels, cfg) -> bool
  return rc_get_switch_pos(channels, cfg.land_channel, cfg) == HIGH
```

```
function rc_get_kill_switch(channels, cfg) -> bool
  return rc_get_switch_pos(channels, cfg.kill_channel, cfg) == HIGH
```

### 6.7. Vòng lặp đọc RC chính — ghép các khối trên

```
function rc_read_loop(uart_handle, cfg, rate=50Hz)
  last_valid_time = now()
  loop at rate:
    raw = rc_read_raw_frame(uart_handle)
    channels = rc_parse_frame(raw)
    channels = rc_validate(channels, cfg, last_valid_time)
    if channels.valid:
      last_valid_time = now()
    publish_rc_state(channels)
    rc_debug_print(channels)
```

### 6.8. Debug

```
function rc_debug_print(channels)
  print(timestamp(), channels.ch, channels.valid, channels.failsafe)
```

---

## 7. Kill switch

### 7.1. Data structures

```
struct KillSwitchContext {
  triggered: bool
  last_trigger_time: float
  debounce_count: int
}
```

### 7.2. Khởi tạo

```
function kill_switch_init() -> KillSwitchContext
  return KillSwitchContext{triggered=false, debounce_count=0}
```

### 7.3. Debounce tín hiệu (chống trigger giả do nhiễu RC)

```
function kill_switch_debounce(ctx, raw_triggered, threshold=3) -> bool
  if raw_triggered:
    ctx.debounce_count += 1
  else:
    ctx.debounce_count = 0
  return ctx.debounce_count >= threshold
```

### 7.4. Thực thi cắt hệ thống ngay lập tức

```
function kill_switch_execute(ctx, disarm_fn)
  ctx.triggered = true
  ctx.last_trigger_time = now()
  disarm_fn()
  publish_vehicle_command(CMD_COMPONENT_ARM_DISARM, param1=0, force=true)
  set_global_flag(SYSTEM_KILLED, true)
  kill_switch_debug_log(ctx)
```

### 7.5. Vòng lặp giám sát tần số cao — ghép các khối trên

```
function kill_switch_monitor_loop(ctx, rc_get_fn, rc_cfg, disarm_fn, rate=50Hz)
  loop at rate:
    channels = rc_get_fn()
    raw_triggered = rc_get_kill_switch(channels, rc_cfg)
    if kill_switch_debounce(ctx, raw_triggered) and not ctx.triggered:
      kill_switch_execute(ctx, disarm_fn)
```

### 7.6. Debug

```
function kill_switch_debug_log(ctx)
  print(timestamp(), ctx.triggered, ctx.debounce_count)
```

Ghi chú: kill switch chạy trên thread/task riêng, tần số cao hơn main loop, độc lập với FSM và Offboard manager. Cờ `SYSTEM_KILLED` được các module khác (offboard, planner) kiểm tra trước khi gửi bất kỳ setpoint nào, đảm bảo không có lệnh nào lọt ra sau khi đã kill.

---

## Ghi chú tích hợp chung

- Tất cả module publish debug log qua hàm `*_debug_log` riêng, có thể bật/tắt bằng flag `DEBUG_ENABLED` global.
- FSM là nguồn state duy nhất, các module gimbal/offboard/planner đọc `ctx.state` read-only, không tự ý đổi state.
- Kill switch có độ ưu tiên cao nhất; khi `SYSTEM_KILLED = true`, Offboard manager và FSM phải dừng gửi mọi setpoint/lệnh mới.
