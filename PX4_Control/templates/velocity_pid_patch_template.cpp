// Template: cach chen mot he so tuy chinh vao vong PID van toc trong
// PositionControl.cpp, khong xoa logic goc, co the tat bang tham so.
//
// Buoc 1: tim dong anchor bang grep (xem muc 3 trong guide).
// Buoc 2: bao boc dong goc bang khoi CUSTOM PATCH nhu duoi day.
// Buoc 3: dam bao _param_mpc_custom_gain va _param_mpc_custom_en
//         da duoc khai bao trong PositionControl.hpp va lay gia tri
//         qua ParamFloat<px4::params::MPC_CUSTOM_GAIN> giong cach
//         cac tham so MPC_* khac dang lam trong file nay.

// >>> CUSTOM PATCH START — anh huong vong PID van toc ngang
float custom_gain = 1.0f;
if (_param_mpc_custom_en.get() != 0) {
    custom_gain = _param_mpc_custom_gain.get();
}

thrust_desired_NE(0) = custom_gain *
    (_gain_vel_p(0) * vel_err(0) + _gain_vel_d(0) * _vel_dot(0) + _thr_int(0));
thrust_desired_NE(1) = custom_gain *
    (_gain_vel_p(1) * vel_err(1) + _gain_vel_d(1) * _vel_dot(1) + _thr_int(1));
// <<< CUSTOM PATCH END

// Dong PID goc (KHONG XOA — comment lai de doi chieu khi debug):
// thrust_desired_NE(0) = _gain_vel_p(0) * vel_err(0) + _gain_vel_d(0) * _vel_dot(0) + _thr_int(0);
// thrust_desired_NE(1) = _gain_vel_p(1) * vel_err(1) + _gain_vel_d(1) * _vel_dot(1) + _thr_int(1);