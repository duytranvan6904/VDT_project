% Initialize the scenario 
sceneUAV = uavScenario(UpdateRate=10,ReferenceLocation=[0 0 0]);

%Create a ground for visualization
addMesh(sceneUAV,"polygon",{[-50 -50; 50 -50; 50 50; -50 50] [-0.5 0]},[0.3 0.3 0.3]);


%% =============================== Map hình cầu random =========================================
% 1. CÁC HÌNH CẦU (Bán kính 3m, Màu đỏ: [1 0 0])

% 1. Tạo ma trận đỉnh (Vertices) và mặt (Faces) gốc của hình cầu bán kính r = 3m

% sceneUAV cần được tạo trước khi chạy đoạn code này
% Ví dụ: sceneUAV = uavScenario;

% %% 1. THÔNG SỐ
% N = 30;
% numobs = N;
% safeDist = 5;
% radius = 2;
% 
% startPos = [-50, -50];
% minDistFromStart = 8;
% minDistBetweenObs = 2 * safeDist - 2;  % Khoảng cách giữa các tâm
% 
% maxAttempts = 100000;                 % Tránh vòng lặp vô hạn
% 
% %% 2. TẠO MESH HÌNH CẦU TÂM TẠI [0, 0, 0]
% [X, Y, Z] = sphere(20);
% fv = surf2patch(radius * X, radius * Y, radius * Z, 'triangles');
% 
% vBase = fv.vertices;
% fBase = fv.faces;
% % Tự chỉnh hình cầu ngay khi Scope tạo mesh
% set(groot, 'defaultPatchCreateFcn', ...
%     @(p, ~) configureSphereAppearance(p, fBase));
% 
% % Thêm các cầu với màu xanh trực tiếp vào scenario
% sphereColor = [0 1 0];
% 
% for k = 1:numobs
%     vShifted = vBase + obstacles(k, :);
% 
%     addMesh(sceneUAV, "custom", ...
%         {vShifted, fBase}, sphereColor);
% end
% 
% %% 3. SINH NGẪU NHIÊN N TÂM CẦU
% obstacles = zeros(N, 3);              % Mỗi hàng: [x, y, z]
% count = 1;
% attempts = 0;
% 
% while count <= N && attempts < maxAttempts
%     attempts = attempts + 1;
% 
%     x = randi([-50, 30]);
%     y = randi([-50, 30]);
%     z = 5; %randi([0, 10]);
% 
%     newPos = [x, y, z];
% 
%     % Giữ vùng trống quanh điểm xuất phát trên mặt phẳng XY
%     if norm(newPos(1:2) - startPos) <= minDistFromStart
%         continue;
%     end
% 
%     % Kiểm tra khoảng cách 3D tới các tâm cầu đã tạo
%     if count > 1
%         delta = obstacles(1:count-1, :) - newPos;
%         dists = sqrt(sum(delta.^2, 2));
% 
%         if any(dists < minDistBetweenObs)
%             continue;
%         end
%     end
% 
%     obstacles(count, :) = newPos;
%     count = count + 1;
% end
% 
% % Chỉ thêm mesh sau khi đã sinh đủ N vật cản
% if count <= N
%     error('Chi sinh duoc %d/%d vat can sau %d lan thu.', ...
%           count - 1, N, maxAttempts);
% end
% 
% %% 4. TỌA ĐỘ VẬT CẢN CHO THUẬT TOÁN UAV
% % Nếu thuật toán dùng NED: [North, East, Down]
% obs = [obstacles(:, 2), obstacles(:, 1), -obstacles(:, 3)];
% 
% % Nếu thuật toán dùng ENU giống mesh, thay dòng trên bằng:
% % obs = obstacles;
% 
% %% 5. THÊM CÁC HÌNH CẦU VÀO SCENARIO
% for k = 1:numobs
%     vShifted = vBase + obstacles(k, :);
%     addMesh(sceneUAV, "custom", {vShifted, fBase}, [0, 1, 0]);
% end


%% =========================== Local Minima ===============================
% numobs = 1;
% addMesh(sceneUAV, "cylinder", {[ -40, -40, 1.5], [0, 12]}, [0 1 0]);
% obs = [-40 -40 -5];


%% =========================== Oscillation ================================
% numobs = 15;
% obs = [-37.17 -42.83 -5;
%     -42.83 -37.17 -5;
%     -34.83 -29.17 -5;
%     -29.17 -34.83 -5;
%     -26.83 -21.17 -5;
%     -21.27 -26.83 -5;
%     -18.83 -13.17 -5;
%     -13.17 -18.83 -5;
%     -38.83 -33.17 -5;
%     -33.17 -38.83 -5;
%     -30.83 -25.17 -5;
%     -25.17 -30.83 -5;
%     -22.83 -17.17 -5;
%     -17.17 -22.83 -5;
%     0 0 -5];

% % Cặp 1 (t = -40)
% addMesh(sceneUAV, "cylinder", {[-42.83, -37.17, 2], [0, 12]}, [0 1 0]);
% addMesh(sceneUAV, "cylinder", {[-37.17, -42.83, 2], [0, 12]}, [0 1 0]);
% 
% % Cặp 2 - xen kẽ (t = -36)
% addMesh(sceneUAV, "cylinder", {[-38.83, -33.17, 2], [0, 12]}, [0 1 0]);
% addMesh(sceneUAV, "cylinder", {[-33.17, -38.83, 2], [0, 12]}, [0 1 0]);
% 
% % Cặp 3 (t = -32)
% addMesh(sceneUAV, "cylinder", {[-34.83, -29.17, 2], [0, 12]}, [0 1 0]);
% addMesh(sceneUAV, "cylinder", {[-29.17, -34.83, 2], [0, 12]}, [0 1 0]);
% 
% % Cặp 4 - xen kẽ (t = -28)
% addMesh(sceneUAV, "cylinder", {[-30.83, -25.17, 2], [0, 12]}, [0 1 0]);
% addMesh(sceneUAV, "cylinder", {[-25.17, -30.83, 2], [0, 12]}, [0 1 0]);
% 
% % Cặp 5 (t = -24)
% addMesh(sceneUAV, "cylinder", {[-26.83, -21.17, 2], [0, 12]}, [0 1 0]);
% addMesh(sceneUAV, "cylinder", {[-21.17, -26.83, 2], [0, 12]}, [0 1 0]);
% 
% % Cặp 6 - xen kẽ (t = -20)
% addMesh(sceneUAV, "cylinder", {[-22.83, -17.17, 2], [0, 12]}, [0 1 0]);
% addMesh(sceneUAV, "cylinder", {[-17.17, -22.83, 2], [0, 12]}, [0 1 0]);
% 
% % Cặp 7 (t = -16)
% addMesh(sceneUAV, "cylinder", {[-18.83, -13.17, 2], [0, 12]}, [0 1 0]);
% addMesh(sceneUAV, "cylinder", {[-13.17, -18.83, 2], [0, 12]}, [0 1 0]);


%% =================================== Map hình trụ random ========================================
% === 1. THIẾT LẬP THÔNG SỐ ===
% N = 40;                             % Số lượng vật cản
% numobs = N;
% safeDist = 3.5;
% radius = 1.5;                         % Bán kính (luôn > 0)
% zBottom = 4;                        
% zTop = 6;                          
% startPos = [-50, -50];              
% minDistBetweenObs = 2 * safeDist - 2; 
% 
% % === 2. SINH MA TRẬN TỌA ĐỘ ===
% obstacles = zeros(N, 3);
% obs = zeros(N, 3);
% count = 1;
% 
% while count <= N 
%     x = randi([-50, 0]);
%     y = randi([-50, 0]);
%     newPos = [x, y];
% 
%     % Kiểm tra né điểm xuất phát UAV
%     if norm(newPos - startPos) <= 8
%         continue;
%     end
% 
%     % Kiểm tra không chạm các vật cản đã tạo trước đó (từ 1 đến count - 1)
%     if count > 1
%         dists = sqrt(sum((obstacles(1:count-1, 1:2) - newPos).^2, 2));
%         if any(dists < minDistBetweenObs)
%             continue;
%         end
%     end
% 
%     % Gán dữ liệu vào hàng hiện tại (count) TRƯỚC
%     obstacles(count, :) = [x, y, radius];
%     obs(count, 1) = y;
%     obs(count, 2) = x;
%     obs(count, 3) = -5;
% 
%     % Tăng count SAU CÙNG
%     count = count + 1;
% end
% 
% % === 3. VẼ LÊN SCENEUAV ===
% for i = 1:size(obstacles, 1)
%     addMesh(sceneUAV, "cylinder", {obstacles(i, :), [zBottom, zTop]}, [0 1 0]);
% end


%% ================= Bãi đáp di động trên đường thẳng =====================

% === 1. THÔNG SỐ CHUYỂN ĐỘNG ===
numobs = 0;
obs = zeros(1,3);

Vt = 1;                  % Tốc độ không đổi [m/s]
padSize = 3;              % Chiều dài/rộng [m]
padThickness = 0.5;       % Chiều dày [m]
% Mặt đất trong NED
groundD = 0;

% Tâm cuboid nằm cao hơn mặt đất nửa chiều dày
padCenterD = groundD - padThickness/2;   % -0.25 m
startPos = [0 -50 padCenterD];    % [North East Down], m
endPos   = [0  50 padCenterD];

dist = norm(endPos - startPos);

assert(Vt > 0, 'v phải lớn hơn 0.');
assert(dist > 0, 'Hai vị trí phải khác nhau.');

totalTime = dist / Vt;
dirVec = (endPos - startPos) / dist;
velPad = Vt * dirVec;

% Yaw thân bãi đáp trùng với hướng chuyển động
yawRad = atan2(dirVec(2), dirVec(1));
yawDeg = rad2deg(yawRad);
yawRateRad = 0;           % Đi thẳng, hướng thân không đổi [rad/s]

% === 2. TẠO QUỸ ĐẠO ===
waypoints = [startPos; endPos];
timeOfArrival = [0; totalTime];

% Gán cùng vận tốc tại hai đầu để chuyển động đều
velocities = [velPad; velPad];

% Orientation yêu cầu mảng quaternion 2x1
q0 = eul2quat([yawRad 0 0], 'ZYX');
quatWaypoints = quaternion(repmat(q0, 2, 1));

trajPad = waypointTrajectory( ...
    'Waypoints', waypoints, ...
    'TimeOfArrival', timeOfArrival, ...
    'Velocities', velocities, ...
    'Orientation', quatWaypoints, ...
    'SampleRate', sceneUAV.UpdateRate, ...
    'ReferenceFrame', 'NED');

% === 3. THÊM PLATFORM VÀ MESH BÃI ĐÁP ===
landingPad = uavPlatform('MovingPad', sceneUAV, ...
    'ReferenceFrame', 'NED', ...
    'Trajectory', trajPad);

% Cuboid nhận kích thước [dài rộng cao], không phải tọa độ vị trí
updateMesh(landingPad, 'cuboid', ...
    {[padSize padSize padThickness]}, ...
    [1 1 0], eye(4));

%% ================= BÃI ĐÁP CHUYỂN ĐỘNG TRÒN ĐỀU =================

% === 1. KÍCH THƯỚC VÀ ĐỘ CAO ===
% numobs = 0;
% obs = zeros(1,3);
% padSize = 3;
% padThickness = 0.5;
% 
% groundD = 0;
% padCenterD = groundD - padThickness/2;  % Tâm bãi: D = -0.25 m
% % Mặt trên bãi: D = -0.5 m
% 
% % === 2. THÔNG SỐ CHUYỂN ĐỘNG ===
% Vt = 0.5;                 % Tốc độ không đổi [m/s]
% R = 10;                   % Bán kính quỹ đạo tâm bãi đáp [m]
% turnSign = 1;             % +1: thuận chiều kim đồng hồ trong NED
%                           % -1: ngược chiều kim đồng hồ, nhìn từ trên
% 
% numLaps = 10;              % Số vòng chuyển động
% pointsPerLap = 72;        % Số khoảng chia mỗi vòng
% 
% startPos = [-10 -10 padCenterD];
% yaw0 = pi/4;              % Hướng chuyển động ban đầu: 45 độ
% 
% assert(Vt > 0 && R > 0, 'Vt và R phải lớn hơn 0.');
% assert(abs(turnSign) == 1, 'turnSign phải bằng +1 hoặc -1.');
% 
% % === 3. TÍNH ĐƯỜNG TRÒN ===
% omega = turnSign * Vt/R;  % Vận tốc góc [rad/s]
% period = 2*pi*R/Vt;       % Thời gian một vòng [s]
% totalTime = numLaps * period;
% 
% % Tâm đường tròn từ vị trí và hướng chuyển động ban đầu
% centerNE = startPos(1:2) ...
%          + turnSign*R*[-sin(yaw0), cos(yaw0)];
% 
% % Góc bán kính ban đầu, khác với góc yaw của bãi đáp
% theta0 = yaw0 - turnSign*pi/2;
% 
% timeOfArrival = linspace( ...
%     0, totalTime, numLaps*pointsPerLap + 1).';
% 
% theta = theta0 + omega*timeOfArrival;
% 
% % Tọa độ tâm platform theo NED
% waypoints = [centerNE(1) + R*cos(theta), ...
%              centerNE(2) + R*sin(theta), ...
%              padCenterD*ones(size(theta))];
% 
% % Vận tốc tiếp tuyến tại từng waypoint
% velocities = [-R*omega*sin(theta), ...
%                R*omega*cos(theta), ...
%                zeros(size(theta))];
% 
% % === 4. TẠO QUỸ ĐẠO ===
% % Yaw tự bám hướng chuyển động.
% % Giữ roll/pitch bằng 0 để mặt bãi đáp luôn nằm ngang.
% trajPad = waypointTrajectory( ...
%     'Waypoints', waypoints, ...
%     'TimeOfArrival', timeOfArrival, ...
%     'Velocities', velocities, ...
%     'AutoPitch', false, ...
%     'AutoBank', false, ...
%     'SampleRate', sceneUAV.UpdateRate, ...
%     'ReferenceFrame', 'NED');
% 
% % === 5. THÊM PLATFORM VÀ MESH ===
% landingPad = uavPlatform('MovingPad', sceneUAV, ...
%     'ReferenceFrame', 'NED', ...
%     'Trajectory', trajPad);
% 
% updateMesh(landingPad, 'cuboid', ...
%     {[padSize padSize padThickness]}, ...
%     [1 1 0], eye(4));
% 
% % === 6. ĐẠO HÀM YAW THAM CHIẾU ===
% yawRateRad = omega;        % Đạo hàm yaw bậc 1 [rad/s]
% yawAccelRad = 0;           % Đạo hàm yaw bậc 2 [rad/s^2]

%% ===========================================================================

% Platform/UAV initial position and orientation
initpos = [-50 -50 -5]; % NED Frame
initori = [0 0 0];

% Add two UAV Platform to the scenario and scale them for easier visualization
platform = uavPlatform("platformUAV",sceneUAV,ReferenceFrame="NED",...
    InitialPosition=initpos,InitialOrientation=eul2quat(initori));

updateMesh(platform,"quadrotor",{2},[0 0 0],eul2tform([0 0 pi]));

LidarModel = uavLidarPointCloudGenerator();
uavSensor("Lidar",platform,LidarModel,"MountingLocation",[0,0,0],"MountingAngles",[0 0 0]);

% ------------------------------------------------------------------------
% function configureSphereAppearance(p, sphereFaces)
%     fig = ancestor(p, 'figure');
% 
%     if isempty(fig) || ~strcmp(fig.Name, 'UAV Scenario Scope')
%         return;
%     end
% 
%     % Chỉ chỉnh mesh hình cầu
%     if isequal(double(p.Faces), double(sphereFaces))
%         set(p, ...
%             'EdgeColor', 'none', ...
%             'FaceAlpha', 1, ...
%             'FaceLighting', 'none');
%     end
% end