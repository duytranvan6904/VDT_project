%% VỊ TRÍ UAV
%Pos_APF  = out.Pos_APF;
Pos_IAPF = out.Pos_IAPF;
% S1 = out.S1;
% S2 = out.S2;
% S3 = out.S3;

% time_Pos_APF  = Pos_APF.Time;
% data_Pos_APF  = Pos_APF.Data;

time_Pos_IAPF = Pos_IAPF.Time;
data_Pos_IAPF = Pos_IAPF.Data;

% timeS1 = S1.Time;
% timeS2 = S2.Time;
% timeS3 = S3.Time;
% 
% dataS1 = squeeze(S1.Data);
% dataS2 = squeeze(S2.Data);
% dataS3 = squeeze(S3.Data);

% Đưa dữ liệu về ma trận có 3 cột: [North, East, Down]
% Mỗi hàng là vị trí UAV tại một thời điểm.
% if Pos_APF.IsTimeFirst
%     data_Pos_APF = reshape(data_Pos_APF, numel(time_Pos_APF), 3);
% else
%     data_Pos_APF = reshape(data_Pos_APF, 3, numel(time_Pos_APF)).';
% end

if Pos_IAPF.IsTimeFirst
    data_Pos_IAPF = reshape(data_Pos_IAPF, numel(time_Pos_IAPF), 3);
else
    data_Pos_IAPF = reshape(data_Pos_IAPF, 3, numel(time_Pos_IAPF)).';
end

% Tách các tọa độ
% North_APF  = data_Pos_APF(:,1);
% East_APF   = data_Pos_APF(:,2);
% Height_APF = -data_Pos_APF(:,3);  % Chiều cao = -Down

North_IAPF  = data_Pos_IAPF(:,1);
East_IAPF   = data_Pos_IAPF(:,2);
Height_IAPF = -data_Pos_IAPF(:,3);

%% VẬN TỐC UAV
% Vx_APF = out.Vx_APF;
% Vy_APF = out.Vy_APF;
% Vz_APF = out.Vz_APF;

% Vx_IAPF = out.Vx_IAPF;
% Vy_IAPF = out.Vy_IAPF;
% Vz_IAPF = out.Vz_IAPF;

% time_Vx_APF = Vx_APF.Time;
% data_Vx_APF = squeeze(Vx_APF.Data);
% 
% time_Vy_APF = Vy_APF.Time;
% data_Vy_APF = squeeze(Vy_APF.Data);
% 
% time_Vz_APF = Vz_APF.Time;
% data_Vz_APF = squeeze(Vz_APF.Data);

% time_Vx_IAPF = Vx_IAPF.Time;
% data_Vx_IAPF = squeeze(Vx_IAPF.Data);
% 
% time_Vy_IAPF = Vy_IAPF.Time;
% data_Vy_IAPF = squeeze(Vy_IAPF.Data);
% 
% time_Vz_IAPF = Vz_IAPF.Time;
% data_Vz_IAPF = squeeze(Vz_IAPF.Data);

%% GÓC ROLL, PITCH, YAW
% Roll_APF  = out.Roll_APF;
% Pitch_APF = out.Pitch_APF;
% Yaw_APF   = out.Yaw_APF;

% Roll_IAPF  = out.Roll_IAPF;
% Pitch_IAPF = out.Pitch_IAPF;
% Yaw_IAPF   = out.Yaw_IAPF;

% time_Roll_APF = Roll_APF.Time;
% data_Roll_APF = squeeze(Roll_APF.Data);
% 
% time_Pitch_APF = Pitch_APF.Time;
% data_Pitch_APF = squeeze(Pitch_APF.Data);
% 
% time_Yaw_APF = Yaw_APF.Time;
% data_Yaw_APF = squeeze(Yaw_APF.Data);

% time_Roll_IAPF = Roll_IAPF.Time;
% data_Roll_IAPF = squeeze(Roll_IAPF.Data);
% 
% time_Pitch_IAPF = Pitch_IAPF.Time;
% data_Pitch_IAPF = squeeze(Pitch_IAPF.Data);
% 
% time_Yaw_IAPF = Yaw_IAPF.Time;
% data_Yaw_IAPF = squeeze(Yaw_IAPF.Data);

%% CỠ CHỮ
FS_label  = 16;
FS_title  = 16;
FS_legend = 12;

%% FIGURE 1: QUỸ ĐẠO UAV 3D
%% FIGURE 1: QUỸ ĐẠO UAV, VẬT CẢN VÀ TARGET
figure(1); clf;
set(gcf, 'Color', 'w', 'Position', [100 100 900 750]);
hold on; grid off; box on;

% ----- Quỹ đạo APF và IAPF -----
% hAPF = plot3(East_APF, North_APF, Height_APF, ...
%              'r-', 'LineWidth', 1.5);

hIAPF = plot3(East_IAPF, North_IAPF, Height_IAPF, ...
              'b--', 'LineWidth', 1.5);

% % ----- Hình trụ xanh lá cây -----
% radius = 1.5;                        % Bán kính (m)
% [Xc, Yc, Zc] = cylinder(radius, 80);
% 
% Xc = Xc - 40;                       % Tâm theo East (m)
% Yc = Yc - 40;                       % Tâm theo North (m)
% Zc = 12*Zc;                         % Chiều cao 0 đến 12 m
% 
% greenColor = [0 0.8 0];
% 
% hObstacle = surf(Xc, Yc, Zc, ...
%     'FaceColor', greenColor, ...
%     'EdgeColor', 'none', ...
%     'FaceAlpha', 0.4);
% 
% % Mặt đáy và mặt trên của hình trụ
% fill3(Xc(1,:), Yc(1,:), Zc(1,:), greenColor, ...
%     'EdgeColor', 'none', 'FaceAlpha', 0.4, ...
%     'HandleVisibility', 'off');
% 
% fill3(Xc(2,:), Yc(2,:), Zc(2,:), greenColor, ...
%     'EdgeColor', 'none', 'FaceAlpha', 0.4, ...
%     'HandleVisibility', 'off');

% ----- Target: tọa độ NED [North, East, Down] -----
target_NED = [-30 -30 0];

% 'p' là ngôi sao năm cánh, tô vàng và viền đen
hTarget = plot3(target_NED(2), target_NED(1), -target_NED(3), ...
    'kp', 'MarkerSize', 14, ...
    'MarkerFaceColor', 'y', 'LineWidth', 1.2);

% ----- Title, các trục và Legend -----
% title('Quỹ đạo UAV: APF và IAPF', 'FontSize', FS_title);
% xlabel('East (m)', 'FontSize', FS_label);
% ylabel('North (m)', 'FontSize', FS_label);
% zlabel('Height relative to NED origin (m)', 'FontSize', FS_label);
title('Quỹ đạo UAV hạ cánh', 'FontSize', FS_title);
xlabel('East (m)', 'FontSize', FS_label);
ylabel('North (m)', 'FontSize', FS_label);
zlabel('Height relative to NED origin (m)', 'FontSize', FS_label);

% legend([hAPF hIAPF hObstacle hTarget], ...
%     {'APF', 'IAPF', 'Obstacle', 'Target'}, ...
%     'FontSize', FS_legend, 'Location', 'best');

% legend([hIAPF hTarget], ...
%     {'IAPF', 'Target'}, ...
%     'FontSize', FS_legend, 'Location', 'best');

axis equal;
view(3);
% exportgraphics(gcf, 'Pos.png', 'Resolution',600);

%% VẼ VẬT CẢN HÌNH TRỤ TỪ ĐỘ CAO 4 ĐẾN 6 M
figure(1);
hold on;

obsRadius = 1.5;
obsBottom = 4;
obsTop    = 6;
greenColor = [0 0.8 0];

% Tạo hình trụ mẫu
[Xbase, Ybase, Zbase] = cylinder(obsRadius, 60);

% Đưa đáy lên 4 m, đỉnh lên 6 m
Zc = obsBottom + (obsTop - obsBottom)*Zbase;

nObs = size(obs,1);
hObstacle = gobjects(nObs,1);

for iObs = 1:nObs
    % Dùng trực tiếp: cột 1 là x, cột 2 là y
    Xc = Xbase + obs(iObs,1);
    Yc = Ybase + obs(iObs,2);

    % Mặt bên
    hObstacle(iObs) = surf(Xc, Yc, Zc, ...
        'FaceColor', greenColor, ...
        'EdgeColor', 'none', ...
        'FaceAlpha', 0.4);

    % Mặt đáy ở độ cao 4 m
    fill3(Xc(1,:), Yc(1,:), Zc(1,:), greenColor, ...
        'EdgeColor', 'none', 'FaceAlpha', 0.4, ...
        'HandleVisibility', 'off');

    % Mặt trên ở độ cao 6 m
    fill3(Xc(2,:), Yc(2,:), Zc(2,:), greenColor, ...
        'EdgeColor', 'none', 'FaceAlpha', 0.4, ...
        'HandleVisibility', 'off');
end

%% TẠO GIF UAV BAY THEO QUỸ ĐẠO IAPF

% ----- Thiết lập GIF -----
gifFile = 'UAV_IAPF.gif';    % Tên file xuất
gifDuration = 10;           % Thời lượng mỗi vòng GIF (giây)
fps = 20;                  % Số khung hình mỗi giây

% ----- Lấy Figure và axes đang chứa quỹ đạo -----
figGIF = ancestor(hIAPF, 'figure');
axGIF  = ancestor(hIAPF, 'axes');

figure(figGIF);
hold(axGIF, 'on');

% Khóa giới hạn trục theo toàn bộ quỹ đạo hiện có
drawnow;

legend([hIAPF hObstacle(1) hTarget], ...
    {'IAPF', 'Obstacles', 'Target'}, ...
    'FontName', 'Times New Roman', ...
    'FontSize', FS_legend, 'Location', 'best');

axis(axGIF, 'manual');

% ----- Chuẩn bị dữ liệu theo thời gian -----
% Loại các thời điểm trùng nhau để nội suy
[tLog, indexTime] = unique(time_Pos_IAPF(:), 'last');

eastLog   = East_IAPF(indexTime);
northLog  = North_IAPF(indexTime);
heightLog = Height_IAPF(indexTime);

assert(numel(tLog) >= 2, ...
    'Cần ít nhất hai thời điểm khác nhau để tạo GIF.');

% Tạo các thời điểm cách đều cho GIF
nFrames = max(2, round(gifDuration*fps));
tFrame = linspace(tLog(1), tLog(end), nFrames);

% Vị trí UAV tại mỗi khung hình
eastFrame   = interp1(tLog, eastLog, tFrame, 'linear');
northFrame  = interp1(tLog, northLog, tFrame, 'linear');
heightFrame = interp1(tLog, heightLog, tFrame, 'linear');

% ----- Cho đường bay bắt đầu từ trạng thái chưa vẽ -----
set(hIAPF, 'XData', NaN, 'YData', NaN, 'ZData', NaN);

% Tam giác đỏ đại diện cho UAV
hUAV = plot3(axGIF, NaN, NaN, NaN, ...
    'r^', 'MarkerSize', 10, ...
    'MarkerFaceColor', 'r', ...
    'HandleVisibility', 'off');

fontname(figGIF, 'Times New Roman');

% ----- Cập nhật chuyển động và lưu từng khung -----
for iFrame = 1:nFrames

    % Các mẫu quỹ đạo gốc UAV đã đi qua
    lastIndex = find(tLog <= tFrame(iFrame), 1, 'last');

    % Hiện dần đường bay đến vị trí hiện tại
    set(hIAPF, ...
        'XData', [eastLog(1:lastIndex); eastFrame(iFrame)], ...
        'YData', [northLog(1:lastIndex); northFrame(iFrame)], ...
        'ZData', [heightLog(1:lastIndex); heightFrame(iFrame)]);

    % Di chuyển UAV
    set(hUAV, ...
        'XData', eastFrame(iFrame), ...
        'YData', northFrame(iFrame), ...
        'ZData', heightFrame(iFrame));

    title(axGIF, sprintf('UAV bay | t = %.2f s', tFrame(iFrame)));

    drawnow;

    % Chụp toàn bộ Figure, gồm nhãn trục và legend
    frame = getframe(figGIF);
    imageRGB = frame2im(frame);
    [imageIndexed, colorMap] = rgb2ind(imageRGB, 256);

    % Ghi khung đầu hoặc nối thêm các khung tiếp theo
    if iFrame == 1
        imwrite(imageIndexed, colorMap, gifFile, 'gif', ...
            'LoopCount', Inf, 'DelayTime', 1/fps);
    else
        imwrite(imageIndexed, colorMap, gifFile, 'gif', ...
            'WriteMode', 'append', 'DelayTime', 1/fps);
    end
end

disp(['Đã lưu GIF: ' gifFile]);

%% FIGURE 2: VẬN TỐC VX, VY, VZ
% figure(2); clf;
% set(gcf, 'Color', 'w', 'Position', [100 100 900 750]);
% 
% % ----- Ô 1: Vx -----
% subplot(3,1,1);
% hold on; grid off; box on;
% 
% %plot(time_Vx_APF, data_Vx_APF, 'r-', 'LineWidth', 1.5);
% plot(time_Vx_IAPF, data_Vx_IAPF, 'b--', 'LineWidth', 1.5);
% title('Vận tốc UAV', ...
%         'FontSize', FS_title);
% %title('V_x', 'FontSize', FS_title);
% %xlabel('Time (s)', 'FontSize', FS_label);
% ylabel('V_x m/s', 'FontSize', FS_label);
% % legend({'IAPF'}, 'FontSize', FS_legend, ...
% %        'Location', 'best');
% 
% % ----- Ô 2: Vy -----
% subplot(3,1,2);
% hold on; grid off; box on;
% 
% % plot(time_Vy_APF, data_Vy_APF, 'r-', 'LineWidth', 1.5);
% plot(time_Vy_IAPF, data_Vy_IAPF, 'b--', 'LineWidth', 1.5);
% 
% %title('V_y', 'FontSize', FS_title);
% %xlabel('Time (s)', 'FontSize', FS_label);
% ylabel('V_y m/s', 'FontSize', FS_label);
% % legend({ 'IAPF'}, 'FontSize', FS_legend, ...
% %       'Location', 'best');
% % 
% % % ----- Ô 3: Vz -----
% subplot(3,1,3);
% hold on; grid off; box on;
% 
% %plot(time_Vz_APF, data_Vz_APF, 'r-', 'LineWidth', 1.5);
% plot(time_Vz_IAPF, data_Vz_IAPF, 'b--', 'LineWidth', 1.5);
% 
% %title('V_z', 'FontSize', FS_title);
% xlabel('Time (s)', 'FontSize', FS_label);
% ylabel('V_z m/s', 'FontSize', FS_label);
% %legend({'APF', 'IAPF'}, 'FontSize', FS_legend, ...
% %       'Location', 'best');
% exportgraphics(gcf, 'Vel.png', 'Resolution',600);
% 
% %% FIGURE 3: GÓC ROLL, PITCH, YAW
% figure(3); clf;
% set(gcf, 'Color', 'w', 'Position', [100 100 900 750]);
% 
% % ----- Ô 1: Roll -----
% subplot(3,1,1);
% hold on; grid off; box on;
% title('Góc Roll, Pitch và Yaw', ...
%         'FontSize', FS_title);
% %plot(time_Roll_APF, data_Roll_APF, 'r-', 'LineWidth', 1.5);
% plot(time_Roll_IAPF, data_Roll_IAPF, 'b--', 'LineWidth', 1.5);
% 
% %title('Góc Roll', 'FontSize', FS_title);
% %xlabel('Time (s)', 'FontSize', FS_label);
% ylabel('Roll (degree)', 'FontSize', FS_label);
% % legend({'APF', 'IAPF'}, 'FontSize', FS_legend, ...
% %        'Location', 'best');
% 
% % ----- Ô 2: Pitch -----
% subplot(3,1,2);
% hold on; grid off; box on;
% 
% %plot(time_Pitch_APF, data_Pitch_APF, 'r-', 'LineWidth', 1.5);
% plot(time_Pitch_IAPF, data_Pitch_IAPF, 'b--', 'LineWidth', 1.5);
% 
% %title('Góc Pitch', 'FontSize', FS_title);
% %xlabel('Time (s)', 'FontSize', FS_label);
% ylabel('Pitch (degree)', 'FontSize', FS_label);
% % legend({'APF', 'IAPF'}, 'FontSize', FS_legend, ...
% %       'Location', 'best');
% % 
% % % ----- Ô 3: Yaw -----
% subplot(3,1,3);
% hold on; grid off; box on;
% 
% %plot(time_Yaw_APF, data_Yaw_APF, 'r-', 'LineWidth', 1.5);
% plot(time_Yaw_IAPF, data_Yaw_IAPF, 'b--', 'LineWidth', 1.5);
% 
% %title('Góc Yaw', 'FontSize', FS_title);
% xlabel('Time (s)', 'FontSize', FS_label);
% ylabel('Yaw (degree)', 'FontSize', FS_label);
% % legend({'APF', 'IAPF'}, 'FontSize', FS_legend, ...
% %        'Location', 'best');
% exportgraphics(gcf, 'angle.png', 'Resolution',600);
% 
% %% Figure 4: Mặt trượt
% figure(4); clf;
% set(gcf, 'Color', 'w', 'Position', [100 100 900 750]);
% 
% % ----- Ô 1: S1 -----
% subplot(3,1,1);
% hold on; grid off; box on;
% 
% plot(timeS1, dataS1, 'b-', 'LineWidth', 1.5);
% title('Mặt trượt', ...
%         'FontSize', FS_title);
% ylabel('Mặt trượt S1', 'FontSize', FS_label);
% 
% % ----- Ô 2: S2 -----
% subplot(3,1,2);
% hold on; grid off; box on;
% plot(timeS2, dataS2, 'b-', 'LineWidth', 1.5);
% ylabel('Mặt trượt S2', 'FontSize', FS_label);
% 
% % % ----- Ô 3: S3 -----
% subplot(3,1,3);
% hold on; grid off; box on;
% plot(timeS3, dataS3, 'b-', 'LineWidth', 1.5);
% xlabel('Time (s)', 'FontSize', FS_label);
% ylabel('Mặt trượt S3', 'FontSize', FS_label);
% exportgraphics(gcf, 'SlidingSurface.png', 'Resolution',600);

fontname(figure(1), 'Times New Roman');
fontname(figure(2), 'Times New Roman');
fontname(figure(3), 'Times New Roman');
fontname(figure(4), 'Times New Roman');