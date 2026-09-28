function [V_cmd, yaw_cmd, Stop] = IAPF_Planner_3D_MultiObs(pos, goal, obs, katt, krep, d0, v_max, numobs)

% =========================================================
% Improved APF Planner for 3D UAV
%
% Coordinate frame: NED
%
% FEATURES
%   1. Attractive force
%   2. Improved repulsive force for GNRON
%   3. Local-minimum detection
%   4. Dynamic tangent escape
%   5. Forward-looking tangent selection
%   6. Directional weighting for oscillation suppression
%   7. Speed scheduling
%
% INPUTS
%   pos     : UAV position        [1x3] [N E D]
%   goal    : Goal position       [1x3] [N E D]
%   obs     : Obstacle positions  [Nx3] [N E D]
%   katt    : Attractive gain
%   krep    : Repulsive gain
%   d0      : Obstacle influence distance [m]
%   v_max   : Maximum velocity [m/s]
%   numobs  : Number of obstacles
%
% OUTPUTS
%   V_cmd   : Velocity command [1x3] [VN VE VD]
%   yaw_cmd : Heading command [rad]
%   Stop    : 1 when goal reached
%
% =========================================================


%% ========================================================
% 1. INITIALIZATION
% =========================================================

V_cmd   = zeros(1,3);
yaw_cmd = 0.0;
Stop    = 0.0;

F_att     = zeros(1,3);
F_rep     = zeros(1,3);
F_cmd_raw = zeros(1,3);

p = zeros(1,3);
g = zeros(1,3);

p(1) = pos(1);
p(2) = pos(2);
p(3) = pos(3);

g(1) = goal(1);
g(2) = goal(2);
g(3) = goal(3);

% Đại lượng tránh chia cho 0 
small_num = 1e-6;


%% ========================================================
% 2. PARAMETERS
% =========================================================

% Ngưỡng xác định UAV đã tới goal.
goal_tol = 0.30;          % [m]

% ---------------------------------------------------------
% Local minimum detection
% ---------------------------------------------------------

% Nếu F_total < F_enter --> Local Minima --> Bật Tangent
F_enter = 0.10;

% Nếu F_total > F_exit --> Thoát khỏi Local Minima
% F_enter < F_total < F_exit --> Vẫn bật Tangent, F_enter < F_exit để tránh Tangent bật tắt liên tục 
F_exit  = 0.30;

% Khoảng cách giữa UAV và target phải lớn hơn goal_min_dist thì mới được
% coi là Local Minima
goal_min_dist = 0.50;


% ---------------------------------------------------------
% Tangential force
% ---------------------------------------------------------

% Hệ số độ mạnh của lực tiếp tuyến
k_tan = 1.0;

% Number of directions searched on tangent plane (Mặt phẳng tiếp tuyến
% vuông góc với vector hướng từ Obs ra UAV
N_tangent = 12;

% Prediction horizon for tangent evaluation
N_pred = 3;

% Prediction distance (Xem UAV sẽ ở đâu)
look_ahead = 0.25*d0;

if look_ahead < 0.30
    look_ahead = 0.30;
end


% Tangent candidate cost
w_goal  = 1.0;
w_clear = 1.0;
w_prev  = 0.60;


% ---------------------------------------------------------
% Oscillation suppression
%
% Δalpha <= pi/6:
%       m = m'
%
% pi/6 < Δalpha <= pi/3:
%       m > m'
%
% Δalpha > pi/3:
%       m >> m'
%
% Numerical values below are implementation choices.
% ---------------------------------------------------------

theta_small = pi/6;
theta_large = pi/3;

% Small direction change
m_prev_1 = 0.50;
m_new_1  = 0.50;

% Medium direction change
m_prev_2 = 0.70;
m_new_2  = 0.30;

% Large direction change
m_prev_3 = 0.90;
m_new_3  = 0.10;


%% ========================================================
% 3. PERSISTENT STATES
% =========================================================

persistent tangent_active
persistent tangent_prev
persistent previous_obs_idx

persistent prev_move_dir
persistent prev_dir_valid


if isempty(tangent_active)
    tangent_active = false;
end

if isempty(tangent_prev)
    tangent_prev = zeros(1,3);
end

if isempty(previous_obs_idx)
    previous_obs_idx = 0;
end

if isempty(prev_move_dir)
    prev_move_dir = zeros(1,3);
end

if isempty(prev_dir_valid)
    prev_dir_valid = false;
end


%% ========================================================
% 4. NUMBER OF OBSTACLES
% =========================================================

Nobs = numobs;

if Nobs > size(obs,1)
    Nobs = size(obs,1);
end

if Nobs < 0
    Nobs = 0;
end


%% ========================================================
% 5. GOAL VECTOR
% =========================================================

goal_vec = g - p;

d_goal = sqrt(goal_vec(1)^2 + goal_vec(2)^2 + goal_vec(3)^2);


%% ========================================================
% 6. GOAL REACHED
% =========================================================

if d_goal <= goal_tol

    V_cmd   = zeros(1,3);
    yaw_cmd = 0.0;
    Stop    = 1.0;

    % Reset memory
    tangent_active   = false;
    tangent_prev     = zeros(1,3);
    previous_obs_idx = 0;

    prev_move_dir = zeros(1,3);
    prev_dir_valid = false;

    return;
end


dir_goal = goal_vec / max(d_goal,small_num);


%% ========================================================
% 7. ATTRACTIVE FORCE
% =========================================================
%
% Uatt = katt*d_goal^2
%
% Fatt = 2*katt*(goal-pos)
%

F_att = 2.0*katt*goal_vec;


%% ========================================================
% 8. IMPROVED REPULSIVE FORCE
%
% GNRON improvement
%
% Frep = Frep1 + Frep2
% =========================================================

min_obs_dist = 1e6;
nearest_idx  = 0;


for i = 1:Nobs

    obs_i = zeros(1,3);

    obs_i(1) = obs(i,1);
    obs_i(2) = obs(i,2);
    obs_i(3) = obs(i,3);


    % Vector obstacle -> UAV
    obs_vec = p - obs_i;

    d_obs = sqrt(obs_vec(1)^2 + obs_vec(2)^2 + obs_vec(3)^2);


    % Nearest obstacle
    if d_obs < min_obs_dist
        min_obs_dist = d_obs;
        nearest_idx  = i;
    end


    if d_obs <= d0

        d_safe = max(d_obs,small_num);

        dir_away = obs_vec/d_safe;


        % -------------------------------------------------
        % Goal-distance weighting
        %
        % Approaches zero when UAV approaches goal
        % -------------------------------------------------

        exp_goal = exp(-d_goal);

        weight_rep = 1.0/(1.0 + exp_goal) - 0.5;


        % -------------------------------------------------
        % Frep1
        % -------------------------------------------------

        coeff1 = 2.0 * krep * (1.0/d_safe - 1.0/d0) * (1.0/(d_safe^2)) * weight_rep;

        F_rep1 = coeff1 * dir_away;


        % -------------------------------------------------
        % Frep2
        % -------------------------------------------------

        sigmoid_derivative = exp_goal/((1.0 + exp_goal)^2);

        coeff2 = krep * (1.0/d_safe - 1.0/d0)^2 * sigmoid_derivative;

        F_rep2 = coeff2 * dir_goal;


        F_rep = F_rep + F_rep1 + F_rep2;

    end

end


%% ========================================================
% 9. NORMAL APF RESULTANT FORCE
% =========================================================

F_total = F_att + F_rep;

F_total_norm = sqrt(F_total(1)^2 + F_total(2)^2 + F_total(3)^2);


%% ========================================================
% 10. OBSTACLE NEARBY
% =========================================================

obstacle_near = false;

if (Nobs > 0) && (min_obs_dist < d0)
    obstacle_near = true;
end


%% ========================================================
% 11. LOCAL MINIMUM DETECTION WITH HYSTERESIS
% =========================================================

if ~tangent_active

    if (F_total_norm < F_enter) && (d_goal > goal_min_dist) && obstacle_near

        tangent_active = true;


        % Do not use tangent from unrelated obstacle
        if nearest_idx ~= previous_obs_idx
            tangent_prev = zeros(1,3);
        end

        previous_obs_idx = nearest_idx;
    end

else

    if (F_total_norm > F_exit) || (~obstacle_near) || (d_goal <= goal_min_dist)

        tangent_active = false;

    end

end


%% ========================================================
% 12. DYNAMIC TANGENT ESCAPE
%
% Tangent is ONLY added when local minimum is active.
% =========================================================

if tangent_active && nearest_idx > 0

    %% ----------------------------------------------------
    % 12.1 Nearest obstacle normal
    % -----------------------------------------------------

    obs_near = zeros(1,3);

    obs_near(1) = obs(nearest_idx,1);
    obs_near(2) = obs(nearest_idx,2);
    obs_near(3) = obs(nearest_idx,3);


    normal_vec = p - obs_near;

    normal_norm = sqrt(normal_vec(1)^2 + normal_vec(2)^2 + normal_vec(3)^2);


    if normal_norm > small_num
        n = normal_vec/normal_norm;
    else
        n = -dir_goal;
    end


    %% ----------------------------------------------------
    % 12.2 Construct tangent plane basis
    % -----------------------------------------------------

    abs_n = abs(n);

    [~,idx_ref] = min(abs_n);

    ref = zeros(1,3);
    ref(idx_ref) = 1.0;


    e1 = cross(n,ref);

    e1_norm = sqrt(e1(1)^2 + e1(2)^2 + e1(3)^2);

    e1 = e1/max(e1_norm,small_num);


    e2 = cross(n,e1);

    e2_norm = sqrt(e2(1)^2 + e2(2)^2 + e2(3)^2);

    e2 = e2/max(e2_norm,small_num);


    %% ----------------------------------------------------
    % 12.3 Forward-looking tangent search
    %
    % Inspired by paper Eq. (12)-(13)
    %
    % Every candidate tangent is predicted several
    % positions ahead and evaluated by:
    %
    %   goal direction
    %   obstacle clearance
    %   continuity with previous tangent
    % -----------------------------------------------------

    best_score = -1e9;
    best_tangent = zeros(1,3);


    tangent_prev_norm = sqrt(tangent_prev(1)^2 + tangent_prev(2)^2 + tangent_prev(3)^2);


    for k = 1:N_tangent

        theta = 2.0 * pi * (k-1)/N_tangent;


        t_candidate = cos(theta) * e1 + sin(theta) * e2;


        t_norm = sqrt(t_candidate(1)^2 + t_candidate(2)^2 + t_candidate(3)^2);

        t_candidate = t_candidate/max(t_norm,small_num);


        % ---------------------------------------------
        % Goal-direction score
        % ---------------------------------------------

        goal_score = dot(t_candidate,dir_goal);


        % ---------------------------------------------
        % Previous tangent continuity score
        % ---------------------------------------------

        prev_score = 0.0;

        if tangent_prev_norm > 1e-4

            prev_tangent_dir = tangent_prev/tangent_prev_norm;

            prev_score = dot(t_candidate,prev_tangent_dir);

        end


        % ---------------------------------------------
        % Forward-looking clearance
        % ---------------------------------------------

        min_pred_dist = 1e6;


        for h = 1:N_pred

            pred_ratio = h/N_pred;

            p_test = p + pred_ratio * look_ahead * t_candidate;


            for j = 1:Nobs

                test_vec = zeros(1,3);

                test_vec(1) = p_test(1) - obs(j,1);

                test_vec(2) = p_test(2) - obs(j,2);

                test_vec(3) = p_test(3) - obs(j,3);


                test_dist = sqrt(test_vec(1)^2 + test_vec(2)^2 + test_vec(3)^2);


                if test_dist < min_pred_dist
                    min_pred_dist = test_dist;
                end

            end

        end


        clear_score = min_pred_dist/max(d0,small_num);


        if clear_score > 2.0
            clear_score = 2.0;
        end


        % ---------------------------------------------
        % Comprehensive score
        % ---------------------------------------------

        score = w_goal*goal_score + w_clear*clear_score + w_prev*prev_score;


        if score > best_score

            best_score = score;

            best_tangent = t_candidate;

        end

    end


    % Store tangent for next sample
    tangent_prev = best_tangent;


    % Tangential force
    F_tan = k_tan*best_tangent;


    % APF + tangent
    F_cmd_raw = F_total + F_tan;


else

    % Normal operation
    F_cmd_raw = F_total;

end


%% ========================================================
% 13. RAW COMMAND DIRECTION
% =========================================================

F_raw_norm = sqrt(F_cmd_raw(1)^2 + F_cmd_raw(2)^2 + F_cmd_raw(3)^2);


if F_raw_norm > small_num

    w_new = F_cmd_raw/F_raw_norm;

else

    w_new = dir_goal;

end


%% ========================================================
% 14. OSCILLATION SUPPRESSION
%
% Based on Eq. (10)-(11) of the paper.
%
% Paper:
%
% X(n+1) =
%   X(n) + L*(m*w_n + m'*w_(n+1))
%
% Our planner outputs velocity instead of waypoint,
% therefore:
%
% w_cmd =
%   normalize(m*w_prev + m'*w_new)
%
% =========================================================

move_dir = w_new;


% Apply directional continuity mainly near obstacles.
if obstacle_near && prev_dir_valid

    % -----------------------------------------------------
    % Angle between previous and current directions
    % -----------------------------------------------------

    cos_alpha = dot(prev_move_dir,w_new);


    % Numerical clamp
    if cos_alpha > 1.0
        cos_alpha = 1.0;
    elseif cos_alpha < -1.0
        cos_alpha = -1.0;
    end


    delta_alpha = acos(cos_alpha);


    % -----------------------------------------------------
    % Direction weighting according to paper Eq. (11)
    % -----------------------------------------------------

    if delta_alpha <= theta_small

        % m = m'
        m_prev = m_prev_1;
        m_new  = m_new_1;


    elseif delta_alpha <= theta_large

        % m > m'
        m_prev = m_prev_2;
        m_new  = m_new_2;


    else

        % m >> m'
        m_prev = m_prev_3;
        m_new  = m_new_3;

    end


    % -----------------------------------------------------
    % Eq. (10)-style weighted direction
    % -----------------------------------------------------

    w_weighted = m_prev*prev_move_dir + m_new*w_new;


    w_weighted_norm = sqrt(w_weighted(1)^2 + w_weighted(2)^2 + w_weighted(3)^2);


    if w_weighted_norm > small_num

        move_dir = w_weighted/w_weighted_norm;

    else

        move_dir = w_new;

    end

end


% ---------------------------------------------------------
% Save actual direction used
% ---------------------------------------------------------

prev_move_dir = move_dir;
prev_dir_valid = true;


%% ========================================================
% 15. SPEED SCHEDULING
% =========================================================

% Slow near goal
goal_speed_factor = d_goal/max(d0,small_num);


if goal_speed_factor > 1.0
    goal_speed_factor = 1.0;
elseif goal_speed_factor < 0.15
    goal_speed_factor = 0.15;
end


% Slow near obstacles
obs_speed_factor = 1.0;


if obstacle_near

    obs_speed_factor = min_obs_dist/max(d0,small_num);


    if obs_speed_factor < 0.25
        obs_speed_factor = 0.25;
    elseif obs_speed_factor > 1.0
        obs_speed_factor = 1.0;
    end

end


v_cmd_mag = v_max * goal_speed_factor * obs_speed_factor;


% Do not become too slow while escaping local minimum
if tangent_active

    v_escape_min = 0.35*v_max;

    if v_cmd_mag < v_escape_min
        v_cmd_mag = v_escape_min;
    end

end


%% ========================================================
% 16. VELOCITY COMMAND
% =========================================================

V_cmd = v_cmd_mag * move_dir;


%% ========================================================
% 17. YAW COMMAND
% =========================================================

horizontal_speed = sqrt(V_cmd(1)^2 + V_cmd(2)^2);


if horizontal_speed > small_num

    yaw_cmd = atan2(V_cmd(2),V_cmd(1));

else

    yaw_cmd = 0.0;

end


Stop = 0.0;

end