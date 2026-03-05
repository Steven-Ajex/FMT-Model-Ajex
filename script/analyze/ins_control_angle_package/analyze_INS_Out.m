%% INS_Out.mat 分析脚本
% 功能：
% 1) 读取 INS_Out.mat 中的 timeseries 字段
% 2) 计算采样、姿态、速度、加速度、四元数一致性等指标
% 3) 输出文本报告 + 图像

clear; clc; close all;

mat_file = 'E:/SUST/log/1/INS_Out.mat';
out_dir = fileparts(mat_file);

if ~exist(mat_file, 'file')
    error('File not found: %s', mat_file);
end

S = load(mat_file, 'INS_Out');
if ~isfield(S, 'INS_Out') || ~isstruct(S.INS_Out)
    error('INS_Out struct not found in mat file.');
end
INS = S.INS_Out;

% -------- 读取核心信号 --------
[t, phi]   = ts_data(INS, 'phi');
[~, theta] = ts_data(INS, 'theta');
[~, psi]   = ts_data(INS, 'psi');
[~, quat]  = ts_data(INS, 'quat');

[~, p] = ts_data(INS, 'p');
[~, q] = ts_data(INS, 'q');
[~, r] = ts_data(INS, 'r');

[~, ax] = ts_data(INS, 'ax');
[~, ay] = ts_data(INS, 'ay');
[~, az] = ts_data(INS, 'az');

[~, vn] = ts_data(INS, 'vn');
[~, ve] = ts_data(INS, 've');
[~, vd] = ts_data(INS, 'vd');

[~, airspeed] = ts_data(INS, 'airspeed');

[~, lat] = ts_data(INS, 'lat');
[~, lon] = ts_data(INS, 'lon');
[~, alt] = ts_data(INS, 'alt');

[~, x_R] = ts_data(INS, 'x_R');
[~, y_R] = ts_data(INS, 'y_R');
[~, h_R] = ts_data(INS, 'h_R');

[~, flag] = ts_data(INS, 'flag');
[~, status] = ts_data(INS, 'status');

N = numel(t);
if N < 2
    error('Not enough samples in INS_Out data.');
end

% -------- 基础统计 --------
dt = diff(t);
fs_med = 1 / median(dt);

phi_deg = rad2deg(phi);
theta_deg = rad2deg(theta);
psi_unwrap_deg = rad2deg(unwrap(psi));

quat_norm = sqrt(sum(quat.^2, 2));
acc_norm = sqrt(ax.^2 + ay.^2 + az.^2);
vh = sqrt(vn.^2 + ve.^2);
v3d = sqrt(vn.^2 + ve.^2 + vd.^2);

lat_deg = rad2deg(lat);
lon_deg = rad2deg(lon);

% 离散状态统计（先转 double，避免 uint 差分溢出）
flag_d = double(flag);
status_d = double(status);

[flag_vals, flag_cnt] = value_counts(flag_d);
[status_vals, status_cnt] = value_counts(status_d);

flag_trans_idx = find(diff(flag_d) ~= 0);
status_trans_idx = find(diff(status_d) ~= 0);

% -------- 输出文本报告 --------
report_file = fullfile(out_dir, 'INS_Out_analysis_report.txt');
fid = fopen(report_file, 'w');
if fid < 0
    error('Cannot open report file for writing: %s', report_file);
end

fprintf(fid, 'INS_Out.mat 数据分析报告\n');
fprintf(fid, '生成时间: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

fprintf(fid, '一、数据概况\n');
fprintf(fid, '- 文件: %s\n', mat_file);
fprintf(fid, '- 样本数: %d\n', N);
fprintf(fid, '- 时长: %.3f s\n', t(end) - t(1));
fprintf(fid, '- 采样频率(中位): %.3f Hz\n', fs_med);
fprintf(fid, '- dt范围: [%.6f, %.6f] s, dt标准差: %.6f s\n\n', min(dt), max(dt), std(dt));

fprintf(fid, '二、姿态与角速度\n');
fprintf(fid, '- Roll  (deg) 范围: [%.3f, %.3f], 均值 %.3f, 标准差 %.3f\n', min(phi_deg), max(phi_deg), mean(phi_deg), std(phi_deg));
fprintf(fid, '- Pitch (deg) 范围: [%.3f, %.3f], 均值 %.3f, 标准差 %.3f\n', min(theta_deg), max(theta_deg), mean(theta_deg), std(theta_deg));
fprintf(fid, '- Yaw(unwrap, deg) 范围: [%.3f, %.3f]\n', min(psi_unwrap_deg), max(psi_unwrap_deg));
fprintf(fid, '- p (rad/s) 范围: [%.3f, %.3f], std %.3f\n', min(p), max(p), std(p));
fprintf(fid, '- q (rad/s) 范围: [%.3f, %.3f], std %.3f\n', min(q), max(q), std(q));
fprintf(fid, '- r (rad/s) 范围: [%.3f, %.3f], std %.3f\n\n', min(r), max(r), std(r));

fprintf(fid, '三、速度与加速度\n');
fprintf(fid, '- 水平速度 vh (m/s): max %.3f, mean %.3f\n', max(vh), mean(vh));
fprintf(fid, '- 三维速度 v3d (m/s): max %.3f, mean %.3f\n', max(v3d), mean(v3d));
fprintf(fid, '- 加速度模长 |a| (m/s^2): min %.3f, max %.3f, mean %.3f, std %.3f\n\n', ...
    min(acc_norm), max(acc_norm), mean(acc_norm), std(acc_norm));

fprintf(fid, '四、定位与相对位移\n');
fprintf(fid, '- lat (deg) 范围: [%.6f, %.6f]\n', min(lat_deg), max(lat_deg));
fprintf(fid, '- lon (deg) 范围: [%.6f, %.6f]\n', min(lon_deg), max(lon_deg));
fprintf(fid, '- alt (m)   范围: [%.3f, %.3f], 均值 %.3f\n', min(alt), max(alt), mean(alt));
fprintf(fid, '- x_R (m) 范围: [%.3f, %.3f]\n', min(x_R), max(x_R));
fprintf(fid, '- y_R (m) 范围: [%.3f, %.3f]\n', min(y_R), max(y_R));
fprintf(fid, '- h_R (m) 范围: [%.3f, %.3f]\n\n', min(h_R), max(h_R));

fprintf(fid, '五、四元数一致性\n');
fprintf(fid, '- ||q|| 范围: [%.6f, %.6f], 均值 %.6f, 标准差 %.6f\n\n', ...
    min(quat_norm), max(quat_norm), mean(quat_norm), std(quat_norm));

fprintf(fid, '六、离散状态统计\n');
fprintf(fid, 'flag 分布:\n');
for i = 1:numel(flag_vals)
    fprintf(fid, '  - flag=%g, count=%d, ratio=%.4f\n', flag_vals(i), flag_cnt(i), flag_cnt(i) / N);
end
fprintf(fid, 'status 分布:\n');
for i = 1:numel(status_vals)
    fprintf(fid, '  - status=%g, count=%d, ratio=%.4f\n', status_vals(i), status_cnt(i), status_cnt(i) / N);
end
fprintf(fid, '\nflag 切换次数: %d\n', numel(flag_trans_idx));
for k = 1:numel(flag_trans_idx)
    idx = flag_trans_idx(k);
    fprintf(fid, '  - t=%.3f s: %g -> %g\n', t(idx+1), flag_d(idx), flag_d(idx+1));
end
fprintf(fid, 'status 切换次数: %d\n', numel(status_trans_idx));
for k = 1:numel(status_trans_idx)
    idx = status_trans_idx(k);
    fprintf(fid, '  - t=%.3f s: %g -> %g\n', t(idx+1), status_d(idx), status_d(idx+1));
end
fprintf(fid, '\n');

fprintf(fid, '七、自动诊断结论\n');
if std(dt) < 1e-3
    fprintf(fid, '- 采样节拍稳定。\n');
else
    fprintf(fid, '- 采样节拍存在明显抖动，建议检查日志同步。\n');
end

if max(abs(quat_norm - 1)) < 1e-3
    fprintf(fid, '- 四元数范数保持良好，姿态解算数值一致性正常。\n');
else
    fprintf(fid, '- 四元数范数偏离1较大，需关注归一化/积分误差。\n');
end

if std(airspeed) < 1e-8
    fprintf(fid, '- airspeed 全程几乎常量（0.1），可能为占位值或未接入有效空速。\n');
end

if max(vh) < 2.0 && max(abs(phi_deg)) < 10 && max(abs(theta_deg)) < 10
    fprintf(fid, '- 当前数据整体表现为低动态飞行/准悬停工况。\n');
end

fclose(fid);

% -------- 绘图 --------
fig1 = figure('Name', 'INS Attitude and Rates', 'Color', 'w');
subplot(3,1,1); hold on; grid on;
plot(t, phi_deg, 'LineWidth', 1.2);
plot(t, theta_deg, 'LineWidth', 1.2);
plot(t, rad2deg(psi), 'LineWidth', 1.2);
ylabel('Angle (deg)');
legend('roll','pitch','yaw');
title('Attitude Euler Angles');

subplot(3,1,2); hold on; grid on;
plot(t, p, 'LineWidth', 1.1);
plot(t, q, 'LineWidth', 1.1);
plot(t, r, 'LineWidth', 1.1);
ylabel('Rate (rad/s)');
legend('p','q','r');
title('Body Rates');

subplot(3,1,3); hold on; grid on;
plot(t, quat_norm, 'LineWidth', 1.2);
ylabel('||q||');
xlabel('Time (s)');
title('Quaternion Norm');

fig2 = figure('Name', 'INS Motion', 'Color', 'w');
subplot(3,1,1); hold on; grid on;
plot(t, ax, 'LineWidth', 1.1);
plot(t, ay, 'LineWidth', 1.1);
plot(t, az, 'LineWidth', 1.1);
ylabel('Accel (m/s^2)');
legend('ax','ay','az');
title('Body Acceleration');

subplot(3,1,2); hold on; grid on;
plot(t, vn, 'LineWidth', 1.1);
plot(t, ve, 'LineWidth', 1.1);
plot(t, vd, 'LineWidth', 1.1);
plot(t, vh, '--k', 'LineWidth', 1.2);
ylabel('Velocity (m/s)');
legend('vn','ve','vd','vh');
title('NED Velocity');

subplot(3,1,3); hold on; grid on;
plot(t, x_R, 'LineWidth', 1.1);
plot(t, y_R, 'LineWidth', 1.1);
plot(t, h_R, 'LineWidth', 1.1);
ylabel('Relative Pos (m)');
xlabel('Time (s)');
legend('x_R','y_R','h_R');
title('Relative Position');

fig3 = figure('Name', 'INS Discrete States', 'Color', 'w');
subplot(2,1,1); hold on; grid on;
stairs(t, flag_d, 'LineWidth', 1.2);
ylabel('flag');
title('Flag State');

subplot(2,1,2); hold on; grid on;
stairs(t, status_d, 'LineWidth', 1.2);
ylabel('status');
xlabel('Time (s)');
title('Status State');

fig4 = figure('Name', 'INS Navigation', 'Color', 'w');
subplot(2,1,1); hold on; grid on;
plot(t, alt, 'LineWidth', 1.2);
plot(t, airspeed, 'LineWidth', 1.2);
ylabel('Alt / Airspeed');
legend('alt (m)','airspeed');
title('Altitude and Airspeed');

subplot(2,1,2); hold on; grid on;
plot(lon_deg, lat_deg, 'LineWidth', 1.2);
xlabel('lon (deg)');
ylabel('lat (deg)');
title('Lat-Lon Track');
axis equal;

saveas(fig1, fullfile(out_dir, 'INS_attitude_rates.png'));
saveas(fig2, fullfile(out_dir, 'INS_motion.png'));
saveas(fig3, fullfile(out_dir, 'INS_states.png'));
saveas(fig4, fullfile(out_dir, 'INS_navigation.png'));

fprintf('Analysis completed.\n');
fprintf('Report: %s\n', report_file);
fprintf('Figures saved in: %s\n', out_dir);

%% -------------------------
% Local helper functions
%% -------------------------
function [t, d] = ts_data(INS, field_name)
    if ~isfield(INS, field_name)
        error('Missing field: %s', field_name);
    end
    ts = INS.(field_name);
    if ~isa(ts, 'timeseries')
        error('Field %s is not timeseries.', field_name);
    end
    t = double(ts.Time(:));
    d = double(ts.Data);
    if size(d,1) ~= numel(t)
        error('Field %s data length mismatch with time.', field_name);
    end
end

function [vals, cnt] = value_counts(x)
    [vals, ~, ic] = unique(x(:));
    cnt = accumarray(ic, 1);
end

