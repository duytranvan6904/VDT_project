% Initialize the scenario 
sceneUAV = uavScenario(UpdateRate=10,ReferenceLocation=[0 0 0]);

%Create a ground for visualization
addMesh(sceneUAV,"polygon",{[-50 -50; 50 -50; 50 50; -50 50] [-0.5 0]},[0.3 0.3 0.3]);

%% ========================================================================
% 1. CÁC HÌNH CẦU (Bán kính 3m, Màu đỏ: [1 0 0])

% 1. Tạo ma trận đỉnh (Vertices) và mặt (Faces) gốc của hình cầu bán kính r = 3m
% r = 3.0;
% [X, Y, Z] = sphere(20);
% fv = surf2patch(r*X, r*Y, r*Z, 'triangles');
% vBase = fv.vertices;
% fBase = fv.faces;
% 
% % % 2. Thêm 5 hình cầu: cộng tâm [Xc, Yc, Zc] trực tiếp vào Vertices
% centers = [
%     -35, -30,  6;
%     -15, -20,  8;
%       0,  -5, 10;
%      15,  25, 11;
%      35,  30, 14
% ];
% 
% for k = 1:size(centers, 1)
%     vShifted = vBase + centers(k, :);
%     addMesh(sceneUAV, "custom", {vShifted, fBase}, [1 0 0]);
% end
% addMesh(sceneUAV, "cylinder", {[-35, -30, 3], [0, 12]}, [0 1 0]);
% addMesh(sceneUAV, "cylinder", {[-15, -20, 3], [0, 12]}, [0 1 0]);
% addMesh(sceneUAV, "cylinder", {[ 0,   -5, 3], [0, 12]}, [0 1 0]);
% addMesh(sceneUAV, "cylinder", {[ 15,  25, 3], [0, 12]}, [0 1 0]);
% addMesh(sceneUAV, "cylinder", {[ 35,  30, 3], [0, 12]}, [0 1 0]);

% 2. CÁC HÌNH TRỤ (Bán kính 3m, Chiều cao từ mặt đất z=0 kéo xuống -25m, Màu xanh: [0 1 0])
% Cú pháp: {[X Y radius], [zBottom zTop]} (trong NED: zBottom = 0, zTop = -25 để chặn đường bay)
% addMesh(sceneUAV, "cylinder", {[-40, -40, 2], [0, 12]}, [0 1 0]);
% addMesh(sceneUAV, "cylinder", {[-15, -15, 2], [0, 12]}, [0 1 0]);
% addMesh(sceneUAV, "cylinder", {[-44, -34, 2], [0, 12]}, [0 1 0]);
% addMesh(sceneUAV, "cylinder", {[-35, -30, 2], [0, 12]}, [0 1 0]);
% addMesh(sceneUAV, "cylinder", {[-25, -10, 2], [0, 12]}, [0 1 0]);
% addMesh(sceneUAV, "cylinder", {[ -5,   8, 2], [0, 12]}, [0 1 0]);
% addMesh(sceneUAV, "cylinder", {[ 20,  15, 2], [0, 12]}, [0 1 0]);
% addMesh(sceneUAV, "cylinder", {[ 38,  42, 2], [0, 12]}, [0 1 0]);
% addMesh(sceneUAV, "cylinder", {[ 2,  2, 2], [0, 12]}, [0 1 0]);

%% ===========================================================================
% === 1. THIẾT LẬP THÔNG SỐ ===
% N = 60;                             % Số lượng vật cản
% numobs = N;
% safeDist = 5;
% radius = 2;                         % Bán kính (luôn > 0)
% zBottom = 0;                        
% zTop = 12;                          
% startPos = [-50, -50];              
% minDistBetweenObs = 2 * safeDist - 2; 
% 
% % === 2. SINH MA TRẬN TỌA ĐỘ ===
% obstacles = zeros(N, 3);
% obs = zeros(N, 3);
% count = 1;
% 
% while count <= N 
%     x = randi([-50, 50]);
%     y = randi([-50, 50]);
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
%% ===============================================================================

% Platform/UAV initial position and orientation
initpos = [-50 -50 -5]; % NED Frame
initori = [0 0 0];

% Add two UAV Platform to the scenario and scale them for easier visualization
platform = uavPlatform("platformUAV",sceneUAV,ReferenceFrame="NED",...
    InitialPosition=initpos,InitialOrientation=eul2quat(initori));

updateMesh(platform,"quadrotor",{2},[0 0 0],eul2tform([0 0 pi]));

LidarModel = uavLidarPointCloudGenerator();
uavSensor("Lidar",platform,LidarModel,"MountingLocation",[0,0,0],"MountingAngles",[0 0 0]);