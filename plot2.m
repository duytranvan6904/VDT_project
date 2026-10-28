%% 1. THIẾT LẬP
FS_label  = 16;
FS_title  = 16;
FS_legend = 12;
fontName  = 'Times New Roman';

gifFile     = 'UAV_IAPF.gif';
gifDuration = 10;
fps         = 20;

obsRadius  = 1.5;
obsBottom  = 4;
obsTop     = 6;
greenColor = [0 0.8 0];

target_NED = [-10 -10 0];

%% 2. LẤY DỮ LIỆU VỊ TRÍ UAV
Pos_IAPF = out.Pos_IAPF;

time_Pos_IAPF = Pos_IAPF.Time;
data_Pos_IAPF = Pos_IAPF.Data;

if Pos_IAPF.IsTimeFirst
    data_Pos_IAPF = reshape( ...
        data_Pos_IAPF, numel(time_Pos_IAPF), 3);
else
    data_Pos_IAPF = reshape( ...
        data_Pos_IAPF, 3, numel(time_Pos_IAPF)).';
end

North_IAPF  = data_Pos_IAPF(:,1);
East_IAPF   = data_Pos_IAPF(:,2);
Height_IAPF = -data_Pos_IAPF(:,3);

%% 3. CHUẨN BỊ DỮ LIỆU CHO GIF
[tLog, indexTime] = unique(time_Pos_IAPF(:), 'last');

eastLog   = East_IAPF(indexTime);
northLog  = North_IAPF(indexTime);
heightLog = Height_IAPF(indexTime);

assert(numel(tLog) >= 2, ...
    'Cần ít nhất hai thời điểm khác nhau để tạo GIF.');

nFrames = max(2, round(gifDuration*fps));
tFrame  = linspace(tLog(1), tLog(end), nFrames);

eastFrame = interp1( ...
    tLog, eastLog, tFrame, 'linear');

northFrame = interp1( ...
    tLog, northLog, tFrame, 'linear');

heightFrame = interp1( ...
    tLog, heightLog, tFrame, 'linear');

%% 4. TẠO FIGURE VÀ VẼ TOÀN BỘ QUỸ ĐẠO
figGIF = figure(1);
clf(figGIF);

set(figGIF, ...
    'Color', 'w', ...
    'Position', [100 100 900 750]);

axGIF = gca;
hold(axGIF, 'on');
grid(axGIF, 'off');
box(axGIF, 'on');

hIAPF = plot3(axGIF, ...
    East_IAPF, North_IAPF, Height_IAPF, ...
    'b--', 'LineWidth', 1.5);

%% 5. VẼ VẬT CẢN: CỘT 1 LÀ X, CỘT 2 LÀ Y
[Xbase, Ybase, Zbase] = cylinder(obsRadius, 60);

Zc = obsBottom + (obsTop - obsBottom)*Zbase;

nObs = size(obs,1);
hObstacle = gobjects(nObs,1);

for iObs = 1:nObs
    Yc = Xbase + obs(iObs,1);
    Xc = Ybase + obs(iObs,2);

    hObstacle(iObs) = surf(axGIF, Xc, Yc, Zc, ...
        'FaceColor', greenColor, ...
        'EdgeColor', 'none', ...
        'FaceAlpha', 0.4);

    fill3(axGIF, ...
        Xc(1,:), Yc(1,:), Zc(1,:), greenColor, ...
        'EdgeColor', 'none', ...
        'FaceAlpha', 0.4, ...
        'HandleVisibility', 'off');

    fill3(axGIF, ...
        Xc(2,:), Yc(2,:), Zc(2,:), greenColor, ...
        'EdgeColor', 'none', ...
        'FaceAlpha', 0.4, ...
        'HandleVisibility', 'off');
end

%% 6. VẼ TARGET VÀ ĐỊNH DẠNG ĐỒ THỊ
hTarget = plot3(axGIF, ...
    target_NED(2), target_NED(1), -target_NED(3), ...
    'kp', ...
    'MarkerSize', 14, ...
    'MarkerFaceColor', 'y', ...
    'LineWidth', 1.2);

hTitle = title(axGIF, ...
    'Quỹ đạo UAV hạ cánh', 'FontSize', FS_title);

xlabel(axGIF, 'East (m)', 'FontSize', FS_label);
ylabel(axGIF, 'North (m)', 'FontSize', FS_label);
zlabel(axGIF, ...
    'Height relative to NED origin (m)', ...
    'FontSize', FS_label);

legend(axGIF, [hIAPF hObstacle(1) hTarget], ...
    {'IAPF', 'Obstacles', 'Target'}, ...
    'FontSize', FS_legend, ...
    'Location', 'best');

axis(axGIF, 'equal');
view(axGIF, 3);
fontname(figGIF, fontName);

drawnow;
axis(axGIF, 'manual');

%% 7. TẠO BIỂU TƯỢNG UAV VÀ KHỞI TẠO ĐƯỜNG BAY
hUAV = plot3(axGIF, NaN, NaN, NaN, ...
    'r^', ...
    'MarkerSize', 10, ...
    'MarkerFaceColor', 'r', ...
    'HandleVisibility', 'off');


set(hIAPF, ...
    'XData', NaN, ...
    'YData', NaN, ...
    'ZData', NaN);
set(hUAV, 'HandleVisibility', 'on');

legend(axGIF, [hIAPF hObstacle(1) hTarget hUAV], ...
    {'Quỹ đạo của UAV', 'Obstacles', 'Target', 'UAV'}, ...
    'FontName', fontName, ...
    'FontSize', FS_legend, ...
    'Location', 'best');
%% 8. CẬP NHẬT CHUYỂN ĐỘNG VÀ GHI GIF
for iFrame = 1:nFrames
    lastIndex = find( ...
        tLog <= tFrame(iFrame), 1, 'last');

    set(hIAPF, ...
        'XData', [eastLog(1:lastIndex); eastFrame(iFrame)], ...
        'YData', [northLog(1:lastIndex); northFrame(iFrame)], ...
        'ZData', [heightLog(1:lastIndex); heightFrame(iFrame)]);

    set(hUAV, ...
        'XData', eastFrame(iFrame), ...
        'YData', northFrame(iFrame), ...
        'ZData', heightFrame(iFrame));

    hTitle.String = sprintf( ...
        'UAV bay | t = %.2f s', tFrame(iFrame));

    drawnow;

    frame = getframe(figGIF);
    imageRGB = frame2im(frame);
    [imageIndexed, colorMap] = rgb2ind(imageRGB, 256);

    if iFrame == 1
        imwrite(imageIndexed, colorMap, gifFile, 'gif', ...
            'LoopCount', Inf, ...
            'DelayTime', 1/fps);
    else
        imwrite(imageIndexed, colorMap, gifFile, 'gif', ...
            'WriteMode', 'append', ...
            'DelayTime', 1/fps);
    end
end

disp(['Đã lưu GIF: ' fullfile(pwd, gifFile)]);