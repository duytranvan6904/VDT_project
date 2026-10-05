Waypoints_Matrix = [0, 50, 50, 0; 0, 0, 50, 50]; 
Arrival_Radius = 2.0;
R = 4.0;                % Look-ahead distance
Mass = 0.1;             % Kg, mass of quadrotor

%% Position controller
P_pos_lead = 1;
P_vel_lead = 5;
P_psi_lead = 2;

P_pos_fl1 = 1;
P_vel_fl1 = 5;
P_psi_fl1 = 5;

P_pos_fl2 = 1;
P_vel_fl2 = 5;
P_psi_fl2 = 5;

%% Lidar config
zGround = -0.15; %m
maxRange = 10;   %m

%% APF
katt = 10;
krep = 90000;
v_max = 2;  % m/s
d0 = 7;     % m (khoảng cách an toàn)

%% Landing
m = 5;
n = 3;
ka = 0.2; %0.015;
kb = 0.6; %0.045;
kc = 0.4; %0.03;
k1 = 0.1395;
k2 = 0.1784;
k3 = 0.0442;
zeta_des = 0;
theta_des = pi/4;
Rmin = 0.1;
alpha_t = pi/2;
thresholdLanding = 15;

%% Vị trí target
%target = [50 50 -5];
%target = [0 50 -5];
%target = [50 0 -5];

%% Vị trí của Hpad
posHpad = [-25 -25 0];
target = [-25 -25 -5];