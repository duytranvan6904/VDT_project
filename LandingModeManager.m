function [dVp_toInt, dalpha_p_toInt, dgamma_p_toInt, ...
          landingMode, resetIntegrator, ...
          Vp0, alpha_p0, gamma_p0, enableIntegration] = ...
    LandingModeManager( ...
    Vp_actual, alpha_p_actual, gamma_p_actual, ...
    dVp_guidance, dalpha_p_guidance, dgamma_p_guidance, ...
    Rxy, Rswitch, validGuidance)

% All inputs scalar; validGuidance logical.
% Actual states come from measured UAV velocity, NOT Landing Integrators.
% Units: speed [m/s], angles [rad], distance [m].
% gamma_p_actual is positive upward.
% GuidanceLaw must run in APF as well to evaluate readiness for handover.
%
% landingMode: false = APF; true = Landing, latched for this simulation.
% resetIntegrator: one-sample pulse only on APF -> Landing.
% x0 outputs follow valid actual states in APF, then freeze at handover.
% d*_toInt: guidance derivatives when enabled, otherwise zero.

persistent inLanding VpInit alphaInit gammaInit
if isempty(inLanding)
    inLanding = false;
    VpInit = 0;
    alphaInit = 0;
    gammaInit = 0;
end

resetIntegrator = false;
dVp_toInt = 0;
dalpha_p_toInt = 0;
dgamma_p_toInt = 0;

stateOK = isfinite(Vp_actual) && Vp_actual > 1e-6 ...
    && isfinite(alpha_p_actual) && isfinite(gamma_p_actual) ...
    && abs(cos(gamma_p_actual)) > 1e-6;

ratesOK = isfinite(dVp_guidance) ...
    && isfinite(dalpha_p_guidance) && isfinite(dgamma_p_guidance);

distanceOK = isfinite(Rxy) && Rxy > 0;
guidanceOK = validGuidance && stateOK && ratesOK && distanceOK;

rangeOK = distanceOK && isfinite(Rswitch) ...
    && Rswitch > 0 && Rxy <= Rswitch;

if ~inLanding
    % Use the actual UAV state at handover as the Landing initial state.
    if stateOK
        VpInit = Vp_actual;
        alphaInit = alpha_p_actual;
        gammaInit = gamma_p_actual;
    end

    if rangeOK && guidanceOK
        inLanding = true;
        resetIntegrator = true;
    end
end

% Pause integration on invalid guidance; keep Landing mode latched.
enableIntegration = inLanding && guidanceOK;
if enableIntegration
    dVp_toInt = dVp_guidance;
    dalpha_p_toInt = dalpha_p_guidance;
    dgamma_p_toInt = dgamma_p_guidance;
end

landingMode = inLanding;
Vp0 = VpInit;
alpha_p0 = alphaInit;
gamma_p0 = gammaInit;
end
