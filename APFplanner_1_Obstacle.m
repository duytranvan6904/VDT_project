function [V_cmd, yaw_cmd] = APF_Planner(pos, goal, obs, numobs)

% =========================================================
% APF Planner for 3D Quadrotor
% pos  : [1x3] UAV position [N E D]
% goal : [1x3] goal position [N E D]
% obs  : [1x3] obstacle position [N E D]
% V_cmd: [1x3] velocity command [VN VE VD]
% =========================================================

% ---------------------------------------------------------
% 1. Initialize outputs with FIXED SIZE
% ---------------------------------------------------------

V_cmd   = zeros(1,3);
yaw_cmd = 0.0;
Stop    = 0.0;
F_rep = zeros(1,3);
F_att = zeros(1,3);
F_total = zeros(1,3);
d_vec = zeros(1,3);

d0 = 5;         % Khoảng cách an toàn (m)
v_max = 2;      % Vận tốc m/s
katt = 10;      % Hệ số hút
krep = 90000;   % Hệ số đẩy

% ---------------------------------------------------------
% 2. Force inputs to fixed-size row vectors
% ---------------------------------------------------------

p = zeros(1,3);
g = zeros(1,3);
o = zeros(1,3);

p(1) = pos(1);
p(2) = pos(2);
p(3) = pos(3);

g(1) = goal(1);
g(2) = goal(2);
g(3) = goal(3);

o(1) = obs(1);
o(2) = obs(2);
o(3) = obs(3);

% Scalar
v_lim = v_max(1);

% Ép obs thành ma trận đúng định dạng (N x 3)
obs_mat = reshape(obs, [], 3);

% 5. Lực đẩy đa vật cản (Repulsive Force)
n_obs = floor(double(numobs(1)));

% Khoảng cách từ UAV đến target
d_goal = g - p;
d_goal_mag = sqrt(d_goal(1)^2 + d_goal(2)^2 + d_goal(3)^2);

% ---------------------------------------------------------
% 3. Attractive force
% ---------------------------------------------------------

    F_att(1) = 2 * katt * d_goal(1);
    F_att(2) = 2 * katt * d_goal(2);
    F_att(3) = 2 * katt * d_goal(3);

% ---------------------------------------------------------
% 4. Repulsive force
% ---------------------------------------------------------

 for i = 1:n_obs
         % Lấy tọa độ vật cản thứ i và tính vector khoảng cách [1 x 3]
         obs_i = [obs_mat(i, 1), obs_mat(i, 2), obs_mat(i, 3)];

         % Khoảng cách từ UAV đến vật cản
         d_vec = p - obs_i;
         d = norm(d_vec);
         F_rep_i = zeros(1,3);
         % Xét điều kiện vùng ảnh hưởng d0
         if d <= d0
             % Vector đơn vị hướng ra xa tâm vật cản
             grad_d = zeros(1, 3);
             grad_d(1) = d_vec(1) / d;
             grad_d(2) = d_vec(2) / d;
             grad_d(3) = d_vec(3) / d;

             % Độ lớn lực đẩy
             rep_mag = 2 * krep * (1/d - 1/d0) * (1/(d^2));

             % Lực tiếp tuyến (bẻ góc lượn tránh kẹt)
             tan_dir = cross([0, 0, 1], grad_d);
             if norm(tan_dir) > 1e-3
                  tan_dir = tan_dir / norm(tan_dir);
             else
                  tan_dir = [0, 1, 0];
             end
             F_tan = rep_mag * tan_dir;

             % Cộng dồn lực đẩy của vật cản thứ i vào tổng lực F_rep
             F_rep_i = rep_mag * grad_d + F_tan;
         end
         F_rep = F_rep + F_rep_i;
  end

% ---------------------------------------------------------
% 5. Total force
% ---------------------------------------------------------

F_total(1) = F_att(1) + F_rep(1);
F_total(2) = F_att(2) + F_rep(2);
F_total(3) = F_att(3) + F_rep(3);

F_norm = sqrt(F_total(1)^2 + F_total(2)^2 + F_total(3)^2);


% ---------------------------------------------------------
% 6. Generate velocity command
% ---------------------------------------------------------

if d_goal_mag < 0.15

    V_cmd(1) = 0;
    V_cmd(2) = 0;
    V_cmd(3) = 0;

    Stop = 1.0;

elseif F_norm > 1e-4

    V_cmd(1) = v_lim * F_total(1) / F_norm;
    V_cmd(2) = v_lim * F_total(2) / F_norm;
    V_cmd(3) = v_lim * F_total(3) / F_norm;

else

    V_cmd(1) = 0;
    V_cmd(2) = 0;
    V_cmd(3) = 0;

end

% ---------------------------------------------------------
% 7. Yaw command
% ---------------------------------------------------------

V_horizontal = sqrt(V_cmd(1)^2 + V_cmd(2)^2);

if V_horizontal > 0.05

    yaw_cmd = atan2(V_cmd(2), V_cmd(1));

else

    yaw_cmd = 0.0;

end

end