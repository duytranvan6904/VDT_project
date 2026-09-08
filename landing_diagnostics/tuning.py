from landing_diagnostics.classify import DriftCause


def propose_param_change(cause, current_params):
    suggestions = {}

    if cause == DriftCause.UNDERDAMPED:
        suggestions['MPC_XY_VEL_P_ACC'] = current_params.get('MPC_XY_VEL_P_ACC', 0.0) * 0.85
        suggestions['MPC_XY_VEL_D_ACC'] = current_params.get('MPC_XY_VEL_D_ACC', 0.0) * 1.2

    elif cause == DriftCause.STEADY_STATE_OFFSET:
        suggestions['MPC_XY_VEL_I_ACC'] = current_params.get('MPC_XY_VEL_I_ACC', 0.0) * 1.3

    elif cause == DriftCause.GROUND_EFFECT:
        suggestions['MPC_LAND_SPEED'] = current_params.get('MPC_LAND_SPEED', 0.0) * 0.8
        suggestions['MPC_LAND_ALT2'] = current_params.get('MPC_LAND_ALT2', 0.0) * 1.2

    return suggestions