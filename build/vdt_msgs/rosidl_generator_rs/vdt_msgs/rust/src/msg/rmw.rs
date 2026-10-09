#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "vdt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__vdt_msgs__msg__AltEstimate() -> *const std::ffi::c_void;
}

#[link(name = "vdt_msgs__rosidl_generator_c")]
extern "C" {
    fn vdt_msgs__msg__AltEstimate__init(msg: *mut AltEstimate) -> bool;
    fn vdt_msgs__msg__AltEstimate__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AltEstimate>, size: usize) -> bool;
    fn vdt_msgs__msg__AltEstimate__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AltEstimate>);
    fn vdt_msgs__msg__AltEstimate__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AltEstimate>, out_seq: *mut rosidl_runtime_rs::Sequence<AltEstimate>) -> bool;
}

// Corresponds to vdt_msgs__msg__AltEstimate
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AltEstimate {

    // This member is not documented.
    #[allow(missing_docs)]
    pub altitude: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub touchdown_flag: bool,

}



impl Default for AltEstimate {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !vdt_msgs__msg__AltEstimate__init(&mut msg as *mut _) {
        panic!("Call to vdt_msgs__msg__AltEstimate__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AltEstimate {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__AltEstimate__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__AltEstimate__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__AltEstimate__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AltEstimate {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AltEstimate where Self: Sized {
  const TYPE_NAME: &'static str = "vdt_msgs/msg/AltEstimate";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__vdt_msgs__msg__AltEstimate() }
  }
}


#[link(name = "vdt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__vdt_msgs__msg__VisionMarker() -> *const std::ffi::c_void;
}

#[link(name = "vdt_msgs__rosidl_generator_c")]
extern "C" {
    fn vdt_msgs__msg__VisionMarker__init(msg: *mut VisionMarker) -> bool;
    fn vdt_msgs__msg__VisionMarker__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<VisionMarker>, size: usize) -> bool;
    fn vdt_msgs__msg__VisionMarker__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<VisionMarker>);
    fn vdt_msgs__msg__VisionMarker__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<VisionMarker>, out_seq: *mut rosidl_runtime_rs::Sequence<VisionMarker>) -> bool;
}

// Corresponds to vdt_msgs__msg__VisionMarker
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VisionMarker {

    // This member is not documented.
    #[allow(missing_docs)]
    pub marker_visible: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pixel_align_error: f32,

}



impl Default for VisionMarker {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !vdt_msgs__msg__VisionMarker__init(&mut msg as *mut _) {
        panic!("Call to vdt_msgs__msg__VisionMarker__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for VisionMarker {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__VisionMarker__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__VisionMarker__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__VisionMarker__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for VisionMarker {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for VisionMarker where Self: Sized {
  const TYPE_NAME: &'static str = "vdt_msgs/msg/VisionMarker";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__vdt_msgs__msg__VisionMarker() }
  }
}


#[link(name = "vdt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__vdt_msgs__msg__InputSnapshot() -> *const std::ffi::c_void;
}

#[link(name = "vdt_msgs__rosidl_generator_c")]
extern "C" {
    fn vdt_msgs__msg__InputSnapshot__init(msg: *mut InputSnapshot) -> bool;
    fn vdt_msgs__msg__InputSnapshot__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<InputSnapshot>, size: usize) -> bool;
    fn vdt_msgs__msg__InputSnapshot__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<InputSnapshot>);
    fn vdt_msgs__msg__InputSnapshot__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<InputSnapshot>, out_seq: *mut rosidl_runtime_rs::Sequence<InputSnapshot>) -> bool;
}

// Corresponds to vdt_msgs__msg__InputSnapshot
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct InputSnapshot {

    // This member is not documented.
    #[allow(missing_docs)]
    pub valid: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub marker_detected: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub align_error: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub altitude: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub delta_h: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub d_horiz: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub touchdown: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub planner_timeout: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw_rate: f32,

}



impl Default for InputSnapshot {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !vdt_msgs__msg__InputSnapshot__init(&mut msg as *mut _) {
        panic!("Call to vdt_msgs__msg__InputSnapshot__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for InputSnapshot {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__InputSnapshot__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__InputSnapshot__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__InputSnapshot__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for InputSnapshot {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for InputSnapshot where Self: Sized {
  const TYPE_NAME: &'static str = "vdt_msgs/msg/InputSnapshot";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__vdt_msgs__msg__InputSnapshot() }
  }
}


#[link(name = "vdt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__vdt_msgs__msg__TimeoutFlags() -> *const std::ffi::c_void;
}

#[link(name = "vdt_msgs__rosidl_generator_c")]
extern "C" {
    fn vdt_msgs__msg__TimeoutFlags__init(msg: *mut TimeoutFlags) -> bool;
    fn vdt_msgs__msg__TimeoutFlags__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TimeoutFlags>, size: usize) -> bool;
    fn vdt_msgs__msg__TimeoutFlags__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TimeoutFlags>);
    fn vdt_msgs__msg__TimeoutFlags__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TimeoutFlags>, out_seq: *mut rosidl_runtime_rs::Sequence<TimeoutFlags>) -> bool;
}

// Corresponds to vdt_msgs__msg__TimeoutFlags
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TimeoutFlags {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ekf_timeout: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub vision_timeout: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub alt_timeout: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub planner_timeout: bool,

}



impl Default for TimeoutFlags {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !vdt_msgs__msg__TimeoutFlags__init(&mut msg as *mut _) {
        panic!("Call to vdt_msgs__msg__TimeoutFlags__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TimeoutFlags {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__TimeoutFlags__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__TimeoutFlags__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__TimeoutFlags__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TimeoutFlags {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TimeoutFlags where Self: Sized {
  const TYPE_NAME: &'static str = "vdt_msgs/msg/TimeoutFlags";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__vdt_msgs__msg__TimeoutFlags() }
  }
}


#[link(name = "vdt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__vdt_msgs__msg__PlannerOutput() -> *const std::ffi::c_void;
}

#[link(name = "vdt_msgs__rosidl_generator_c")]
extern "C" {
    fn vdt_msgs__msg__PlannerOutput__init(msg: *mut PlannerOutput) -> bool;
    fn vdt_msgs__msg__PlannerOutput__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PlannerOutput>, size: usize) -> bool;
    fn vdt_msgs__msg__PlannerOutput__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PlannerOutput>);
    fn vdt_msgs__msg__PlannerOutput__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PlannerOutput>, out_seq: *mut rosidl_runtime_rs::Sequence<PlannerOutput>) -> bool;
}

// Corresponds to vdt_msgs__msg__PlannerOutput
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlannerOutput {

    // This member is not documented.
    #[allow(missing_docs)]
    pub vx: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub vy: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub vz: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw: f32,

}



impl Default for PlannerOutput {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !vdt_msgs__msg__PlannerOutput__init(&mut msg as *mut _) {
        panic!("Call to vdt_msgs__msg__PlannerOutput__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PlannerOutput {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__PlannerOutput__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__PlannerOutput__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__PlannerOutput__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PlannerOutput {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PlannerOutput where Self: Sized {
  const TYPE_NAME: &'static str = "vdt_msgs/msg/PlannerOutput";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__vdt_msgs__msg__PlannerOutput() }
  }
}


#[link(name = "vdt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__vdt_msgs__msg__OffboardStatus() -> *const std::ffi::c_void;
}

#[link(name = "vdt_msgs__rosidl_generator_c")]
extern "C" {
    fn vdt_msgs__msg__OffboardStatus__init(msg: *mut OffboardStatus) -> bool;
    fn vdt_msgs__msg__OffboardStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<OffboardStatus>, size: usize) -> bool;
    fn vdt_msgs__msg__OffboardStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<OffboardStatus>);
    fn vdt_msgs__msg__OffboardStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<OffboardStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<OffboardStatus>) -> bool;
}

// Corresponds to vdt_msgs__msg__OffboardStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct OffboardStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub offboard_active: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub heartbeat_age_sec: f32,

}



impl Default for OffboardStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !vdt_msgs__msg__OffboardStatus__init(&mut msg as *mut _) {
        panic!("Call to vdt_msgs__msg__OffboardStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for OffboardStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__OffboardStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__OffboardStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__OffboardStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for OffboardStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for OffboardStatus where Self: Sized {
  const TYPE_NAME: &'static str = "vdt_msgs/msg/OffboardStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__vdt_msgs__msg__OffboardStatus() }
  }
}


#[link(name = "vdt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__vdt_msgs__msg__RcFsmInput() -> *const std::ffi::c_void;
}

#[link(name = "vdt_msgs__rosidl_generator_c")]
extern "C" {
    fn vdt_msgs__msg__RcFsmInput__init(msg: *mut RcFsmInput) -> bool;
    fn vdt_msgs__msg__RcFsmInput__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RcFsmInput>, size: usize) -> bool;
    fn vdt_msgs__msg__RcFsmInput__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RcFsmInput>);
    fn vdt_msgs__msg__RcFsmInput__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RcFsmInput>, out_seq: *mut rosidl_runtime_rs::Sequence<RcFsmInput>) -> bool;
}

// Corresponds to vdt_msgs__msg__RcFsmInput
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RcFsmInput {

    // This member is not documented.
    #[allow(missing_docs)]
    pub land_switch: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub kill_switch: bool,

}



impl Default for RcFsmInput {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !vdt_msgs__msg__RcFsmInput__init(&mut msg as *mut _) {
        panic!("Call to vdt_msgs__msg__RcFsmInput__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RcFsmInput {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__RcFsmInput__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__RcFsmInput__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__RcFsmInput__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RcFsmInput {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RcFsmInput where Self: Sized {
  const TYPE_NAME: &'static str = "vdt_msgs/msg/RcFsmInput";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__vdt_msgs__msg__RcFsmInput() }
  }
}


#[link(name = "vdt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__vdt_msgs__msg__RcChannelsRaw() -> *const std::ffi::c_void;
}

#[link(name = "vdt_msgs__rosidl_generator_c")]
extern "C" {
    fn vdt_msgs__msg__RcChannelsRaw__init(msg: *mut RcChannelsRaw) -> bool;
    fn vdt_msgs__msg__RcChannelsRaw__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RcChannelsRaw>, size: usize) -> bool;
    fn vdt_msgs__msg__RcChannelsRaw__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RcChannelsRaw>);
    fn vdt_msgs__msg__RcChannelsRaw__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RcChannelsRaw>, out_seq: *mut rosidl_runtime_rs::Sequence<RcChannelsRaw>) -> bool;
}

// Corresponds to vdt_msgs__msg__RcChannelsRaw
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RcChannelsRaw {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ch: [i16; 16],


    // This member is not documented.
    #[allow(missing_docs)]
    pub valid: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub failsafe: bool,

}



impl Default for RcChannelsRaw {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !vdt_msgs__msg__RcChannelsRaw__init(&mut msg as *mut _) {
        panic!("Call to vdt_msgs__msg__RcChannelsRaw__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RcChannelsRaw {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__RcChannelsRaw__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__RcChannelsRaw__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { vdt_msgs__msg__RcChannelsRaw__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RcChannelsRaw {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RcChannelsRaw where Self: Sized {
  const TYPE_NAME: &'static str = "vdt_msgs/msg/RcChannelsRaw";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__vdt_msgs__msg__RcChannelsRaw() }
  }
}


