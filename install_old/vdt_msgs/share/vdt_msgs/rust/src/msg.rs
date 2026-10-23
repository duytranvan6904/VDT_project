#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to vdt_msgs__msg__AltEstimate

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::AltEstimate::default())
  }
}

impl rosidl_runtime_rs::Message for AltEstimate {
  type RmwMsg = super::msg::rmw::AltEstimate;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        altitude: msg.altitude,
        touchdown_flag: msg.touchdown_flag,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      altitude: msg.altitude,
      touchdown_flag: msg.touchdown_flag,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      altitude: msg.altitude,
      touchdown_flag: msg.touchdown_flag,
    }
  }
}


// Corresponds to vdt_msgs__msg__VisionMarker

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::VisionMarker::default())
  }
}

impl rosidl_runtime_rs::Message for VisionMarker {
  type RmwMsg = super::msg::rmw::VisionMarker;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        marker_visible: msg.marker_visible,
        pixel_align_error: msg.pixel_align_error,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      marker_visible: msg.marker_visible,
      pixel_align_error: msg.pixel_align_error,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      marker_visible: msg.marker_visible,
      pixel_align_error: msg.pixel_align_error,
    }
  }
}


// Corresponds to vdt_msgs__msg__InputSnapshot

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::InputSnapshot::default())
  }
}

impl rosidl_runtime_rs::Message for InputSnapshot {
  type RmwMsg = super::msg::rmw::InputSnapshot;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        valid: msg.valid,
        marker_detected: msg.marker_detected,
        align_error: msg.align_error,
        altitude: msg.altitude,
        delta_h: msg.delta_h,
        d_horiz: msg.d_horiz,
        touchdown: msg.touchdown,
        planner_timeout: msg.planner_timeout,
        yaw_rate: msg.yaw_rate,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      valid: msg.valid,
      marker_detected: msg.marker_detected,
      align_error: msg.align_error,
      altitude: msg.altitude,
      delta_h: msg.delta_h,
      d_horiz: msg.d_horiz,
      touchdown: msg.touchdown,
      planner_timeout: msg.planner_timeout,
      yaw_rate: msg.yaw_rate,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      valid: msg.valid,
      marker_detected: msg.marker_detected,
      align_error: msg.align_error,
      altitude: msg.altitude,
      delta_h: msg.delta_h,
      d_horiz: msg.d_horiz,
      touchdown: msg.touchdown,
      planner_timeout: msg.planner_timeout,
      yaw_rate: msg.yaw_rate,
    }
  }
}


// Corresponds to vdt_msgs__msg__TimeoutFlags

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TimeoutFlags::default())
  }
}

impl rosidl_runtime_rs::Message for TimeoutFlags {
  type RmwMsg = super::msg::rmw::TimeoutFlags;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ekf_timeout: msg.ekf_timeout,
        vision_timeout: msg.vision_timeout,
        alt_timeout: msg.alt_timeout,
        planner_timeout: msg.planner_timeout,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      ekf_timeout: msg.ekf_timeout,
      vision_timeout: msg.vision_timeout,
      alt_timeout: msg.alt_timeout,
      planner_timeout: msg.planner_timeout,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      ekf_timeout: msg.ekf_timeout,
      vision_timeout: msg.vision_timeout,
      alt_timeout: msg.alt_timeout,
      planner_timeout: msg.planner_timeout,
    }
  }
}


// Corresponds to vdt_msgs__msg__PlannerOutput

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::PlannerOutput::default())
  }
}

impl rosidl_runtime_rs::Message for PlannerOutput {
  type RmwMsg = super::msg::rmw::PlannerOutput;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        vx: msg.vx,
        vy: msg.vy,
        vz: msg.vz,
        yaw: msg.yaw,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      vx: msg.vx,
      vy: msg.vy,
      vz: msg.vz,
      yaw: msg.yaw,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      vx: msg.vx,
      vy: msg.vy,
      vz: msg.vz,
      yaw: msg.yaw,
    }
  }
}


// Corresponds to vdt_msgs__msg__OffboardStatus

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::OffboardStatus::default())
  }
}

impl rosidl_runtime_rs::Message for OffboardStatus {
  type RmwMsg = super::msg::rmw::OffboardStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        offboard_active: msg.offboard_active,
        heartbeat_age_sec: msg.heartbeat_age_sec,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      offboard_active: msg.offboard_active,
      heartbeat_age_sec: msg.heartbeat_age_sec,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      offboard_active: msg.offboard_active,
      heartbeat_age_sec: msg.heartbeat_age_sec,
    }
  }
}


// Corresponds to vdt_msgs__msg__RcFsmInput

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RcFsmInput::default())
  }
}

impl rosidl_runtime_rs::Message for RcFsmInput {
  type RmwMsg = super::msg::rmw::RcFsmInput;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        land_switch: msg.land_switch,
        kill_switch: msg.kill_switch,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      land_switch: msg.land_switch,
      kill_switch: msg.kill_switch,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      land_switch: msg.land_switch,
      kill_switch: msg.kill_switch,
    }
  }
}


// Corresponds to vdt_msgs__msg__RcChannelsRaw

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RcChannelsRaw::default())
  }
}

impl rosidl_runtime_rs::Message for RcChannelsRaw {
  type RmwMsg = super::msg::rmw::RcChannelsRaw;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ch: msg.ch,
        valid: msg.valid,
        failsafe: msg.failsafe,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ch: msg.ch,
      valid: msg.valid,
      failsafe: msg.failsafe,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      ch: msg.ch,
      valid: msg.valid,
      failsafe: msg.failsafe,
    }
  }
}


