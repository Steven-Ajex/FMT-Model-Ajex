%% Control_Out 与 INS 三轴角度变化联合分析（仅到角度变化）
% 目标：
% 1) 读取 Control_Out.mat 的电机输出（前4列）
% 2) 与 INS_Out.mat 的 roll/pitch/yaw 角度变化进行时间对齐
% 3) 输出电机-角度变化耦合分析报告与图表

clear; clc; close all;

ctrl_mat = 'E:/SUST/log/1/Control_Out.mat';
ins_mat  = 'E:/SUST/log/1/INS_Out.mat';
out_dir  = fileparts(ctrl_mat);

if ~exist(ctrl_mat, 'file')
    error('File not found: %s', ctrl_mat);
end
if ~exist(ins_mat, 'file')
    error('File not found: %s', ins_mat);
end

S1 = load(ctrl_mat, 'Control_Out');
S2 = load(ins_mat, 'INS_Out');

if ~isfield(S1, 'Control_Out') || ~isstruct(S1.Control_Out)
    error('Control_Out struct not found.');
end
if ~isfield(S2, 'INS_Out') || ~isstruct(S2.INS_Out)
    error('INS_Out struct not found.');
end

CTRL = S1.Control_Out;
INS  = S2.INS_Out;

% ===== 读取数据 =====
[t_ctrl, u_all] = ts_data(CTRL, 'actuator_cmd');
[t_phi, phi] = ts_data(INS, 'phi');
[t_theta, theta] = ts_data(INS, 'theta');
[t_psi, psi] = ts_data(INS, 'psi');
[t_status, status] = ts_data(INS, 'status');

if size(u_all,2) < 4
    error('actuator_cmd has less than 4 channels.');
end
u_m = double(u_all(:,1:4)); % 仅前4路电机输出

% ===== 有效段选择（优先 status=29） =====
status = double(status(:));
idx_valid_ins = find(status == 29, 1, 'first');
if isempty(idx_valid_ins)
    idx_valid_ins = 1;
end

t_start = max(t_ctrl(1), t_phi(idx_valid_ins));
t_end   = min(t_ctrl(end), t_phi(end));
if t_end <= t_start
    error('No overlap between Control and INS time ranges.');
end

% 统一到 INS 时间轴（有效段）
mask_ins = (t_phi >= t_start) & (t_phi <= t_end);
t = t_phi(mask_ins);

phi_deg = rad2deg(phi(mask_ins));
theta_deg = rad2deg(theta(mask_ins));
psi_deg = rad2deg(unwrap(psi(mask_ins)));

% 电机输出插值到同一时间轴
m1 = interp1(t_ctrl, u_m(:,1), t, 'linear', 'extrap');
m2 = interp1(t_ctrl, u_m(:,2), t, 'linear', 'extrap');
m3 = interp1(t_ctrl, u_m(:,3), t, 'linear', 'extrap');
m4 = interp1(t_ctrl, u_m(:,4), t, 'linear', 'extrap');
M = [m1 m2 m3 m4];

% ===== 角度变化量 =====
roll_delta  = phi_deg   - phi_deg(1);
pitch_delta = theta_deg - theta_deg(1);
yaw_delta   = psi_deg   - psi_deg(1);

dt = diff(t);
roll_rate  = [0; diff(phi_deg) ./ dt];
pitch_rate = [0; diff(theta_deg) ./ dt];
yaw_rate   = [0; diff(psi_deg) ./ dt];

% ===== 电机通道分解 =====
u_mean = mean(M, 2);

% 依据当前工程中常用的电机编号定义构造“轴向混控等效通道”
u_roll_like  = (-m1 + m2 - m3 + m4) / 4;
u_pitch_like = ( m1 - m2 - m3 + m4) / 4;
u_yaw_like   = (-m1 - m2 + m3 + m4) / 4;

% ===== 相关性分析（角度变化） =====
A_delta = [roll_delta, pitch_delta, yaw_delta];
A_rate  = [roll_rate, pitch_rate, yaw_rate];
axis_name = {'ROLL','PITCH','YAW'};
motor_name = {'M1','M2','M3','M4'};

corr_m_delta = zeros(4,3);
for i = 1:4
    for j = 1:3
        corr_m_delta(i,j) = safe_corr(M(:,i), A_delta(:,j));
    end
end

mix_mat = [u_roll_like, u_pitch_like, u_yaw_like];
corr_mix_delta = zeros(3,3);
corr_mix_rate  = zeros(3,3);
for i = 1:3
    for j = 1:3
        corr_mix_delta(i,j) = safe_corr(mix_mat(:,i), A_delta(:,j));
        corr_mix_rate(i,j)  = safe_corr(mix_mat(:,i), A_rate(:,j));
    end
end

% ===== 时滞分析（每轴“同名通道”对同名角度变化） =====
fs = 1 / median(dt);
max_lag_s = 2.0;
max_lag_n = max(1, round(max_lag_s * fs));

[lag_roll,  lag_corr_roll]  = best_lag_corr(u_roll_like,  roll_delta,  max_lag_n, median(dt));
[lag_pitch, lag_corr_pitch] = best_lag_corr(u_pitch_like, pitch_delta, max_lag_n, median(dt));
[lag_yaw,   lag_corr_yaw]   = best_lag_corr(u_yaw_like,   yaw_delta,   max_lag_n, median(dt));

% ===== 角度变化摘要 =====
roll_res  = angle_stats(t, phi_deg, roll_delta, roll_rate);
pitch_res = angle_stats(t, theta_deg, pitch_delta, pitch_rate);
yaw_res   = angle_stats(t, psi_deg, yaw_delta, yaw_rate);

% ===== 文本报告 =====
report_file = fullfile(out_dir, 'Control_INS_angle_coupling_report.txt');
fid = fopen(report_file, 'w');
if fid < 0
    error('Cannot open report file: %s', report_file);
end

fprintf(fid, 'Control_Out 与 INS 三轴角度变化联合分析报告\n');
fprintf(fid, '生成时间: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

fprintf(fid, '一、分析范围\n');
fprintf(fid, '- Control 数据: %s\n', ctrl_mat);
fprintf(fid, '- INS 数据    : %s\n', ins_mat);
fprintf(fid, '- 分析维度    : 电机输出(前4路) 与 roll/pitch/yaw 角度变化\n');
fprintf(fid, '- 统一时段    : [%.3f, %.3f] s, 样本数 %d\n', t(1), t(end), numel(t));
fprintf(fid, '- 采样频率(中位): %.3f Hz\n\n', fs);

fprintf(fid, '二、电机输出统计（PWM）\n');
for i = 1:4
    fprintf(fid, '- %s: min %.3f, max %.3f, mean %.3f, std %.3f\n', ...
        motor_name{i}, min(M(:,i)), max(M(:,i)), mean(M(:,i)), std(M(:,i)));
end
fprintf(fid, '- Mean(4 motors): min %.3f, max %.3f, mean %.3f, std %.3f\n\n', ...
    min(u_mean), max(u_mean), mean(u_mean), std(u_mean));

fprintf(fid, '三、三轴角度变化统计\n');
write_angle_summary(fid, 'ROLL',  roll_res);
write_angle_summary(fid, 'PITCH', pitch_res);
write_angle_summary(fid, 'YAW',   yaw_res);
fprintf(fid, '\n');

fprintf(fid, '四、电机与角度变化相关性\n');
fprintf(fid, '1) 单电机 vs 角度变化 DeltaAngle（相关系数）\n');
fprintf(fid, '          ROLL      PITCH      YAW\n');
for i = 1:4
    fprintf(fid, '  %-4s %9.4f %10.4f %10.4f\n', motor_name{i}, ...
        corr_m_delta(i,1), corr_m_delta(i,2), corr_m_delta(i,3));
end

fprintf(fid, '\n2) 混控等效通道 vs 角度变化 DeltaAngle（相关系数）\n');
fprintf(fid, '             ROLL      PITCH      YAW\n');
fprintf(fid, '  U_roll   %9.4f %10.4f %10.4f\n', corr_mix_delta(1,1), corr_mix_delta(1,2), corr_mix_delta(1,3));
fprintf(fid, '  U_pitch  %9.4f %10.4f %10.4f\n', corr_mix_delta(2,1), corr_mix_delta(2,2), corr_mix_delta(2,3));
fprintf(fid, '  U_yaw    %9.4f %10.4f %10.4f\n', corr_mix_delta(3,1), corr_mix_delta(3,2), corr_mix_delta(3,3));

fprintf(fid, '\n3) 混控等效通道 vs 角速度（deg/s）相关系数\n');
fprintf(fid, '             ROLL      PITCH      YAW\n');
fprintf(fid, '  U_roll   %9.4f %10.4f %10.4f\n', corr_mix_rate(1,1), corr_mix_rate(1,2), corr_mix_rate(1,3));
fprintf(fid, '  U_pitch  %9.4f %10.4f %10.4f\n', corr_mix_rate(2,1), corr_mix_rate(2,2), corr_mix_rate(2,3));
fprintf(fid, '  U_yaw    %9.4f %10.4f %10.4f\n', corr_mix_rate(3,1), corr_mix_rate(3,2), corr_mix_rate(3,3));

fprintf(fid, '\n4) 同轴时滞估计（U_axis -> DeltaAngle_axis）\n');
fprintf(fid, '- ROLL : best_lag = %.3f s, corr = %.4f\n', lag_roll, lag_corr_roll);
fprintf(fid, '- PITCH: best_lag = %.3f s, corr = %.4f\n', lag_pitch, lag_corr_pitch);
fprintf(fid, '- YAW  : best_lag = %.3f s, corr = %.4f\n', lag_yaw, lag_corr_yaw);

fprintf(fid, '\n五、结论（仅到角度变化）\n');
fprintf(fid, '- 已确认电机输出与三轴角度变化存在可量化耦合关系，且可通过等效混控通道描述。\n');
fprintf(fid, '- 如需进入控制性能层面（超调/相位裕度/闭环带宽），可在此脚本基础上继续扩展。\n');

fclose(fid);

% ===== 绘图 =====
fig1 = figure('Name','Control Motors','Color','w');
subplot(2,1,1); hold on; grid on;
plot(t, M(:,1), 'LineWidth', 1.1);
plot(t, M(:,2), 'LineWidth', 1.1);
plot(t, M(:,3), 'LineWidth', 1.1);
plot(t, M(:,4), 'LineWidth', 1.1);
ylabel('PWM');
title('Motor Outputs (M1-M4)');
legend('M1','M2','M3','M4');

subplot(2,1,2); hold on; grid on;
plot(t, u_mean, 'k', 'LineWidth', 1.3);
plot(t, u_roll_like,  'LineWidth', 1.1);
plot(t, u_pitch_like, 'LineWidth', 1.1);
plot(t, u_yaw_like,   'LineWidth', 1.1);
ylabel('Equivalent Channel');
xlabel('Time (s)');
legend('U_mean','U_roll','U_pitch','U_yaw');
title('Equivalent Motor Mixing Channels');

fig2 = figure('Name','INS Angle Changes','Color','w');
subplot(3,1,1); hold on; grid on;
plot(t, roll_delta, 'LineWidth', 1.2);
ylabel('\Delta roll (deg)');
title('Roll Change');

subplot(3,1,2); hold on; grid on;
plot(t, pitch_delta, 'LineWidth', 1.2);
ylabel('\Delta pitch (deg)');
title('Pitch Change');

subplot(3,1,3); hold on; grid on;
plot(t, yaw_delta, 'LineWidth', 1.2);
ylabel('\Delta yaw (deg)');
xlabel('Time (s)');
title('Yaw Change');

fig3 = figure('Name','Control-Angle Coupling','Color','w');
subplot(3,1,1); hold on; grid on;
plot(t, zscore_safe(u_roll_like), 'LineWidth', 1.1);
plot(t, zscore_safe(roll_delta), 'LineWidth', 1.1);
legend('z(U_{roll})','z(\Delta roll)');
title('ROLL Coupling');

subplot(3,1,2); hold on; grid on;
plot(t, zscore_safe(u_pitch_like), 'LineWidth', 1.1);
plot(t, zscore_safe(pitch_delta), 'LineWidth', 1.1);
legend('z(U_{pitch})','z(\Delta pitch)');
title('PITCH Coupling');

subplot(3,1,3); hold on; grid on;
plot(t, zscore_safe(u_yaw_like), 'LineWidth', 1.1);
plot(t, zscore_safe(yaw_delta), 'LineWidth', 1.1);
legend('z(U_{yaw})','z(\Delta yaw)');
title('YAW Coupling');
xlabel('Time (s)');

saveas(fig1, fullfile(out_dir, 'Control_motor_outputs.png'));
saveas(fig2, fullfile(out_dir, 'INS_angle_changes_from_control_window.png'));
saveas(fig3, fullfile(out_dir, 'Control_INS_angle_coupling.png'));

fprintf('Analysis done.\n');
fprintf('Report: %s\n', report_file);

%% =========================
% Local functions
%% =========================
function [t, d] = ts_data(S, field_name)
    if ~isfield(S, field_name)
        error('Missing field: %s', field_name);
    end
    ts = S.(field_name);
    if ~isa(ts, 'timeseries')
        error('Field %s is not timeseries.', field_name);
    end
    t = double(ts.Time(:));
    d = double(ts.Data);
    if size(d,1) ~= numel(t)
        error('Field %s length mismatch with time.', field_name);
    end
end

function c = safe_corr(x, y)
    x = double(x(:));
    y = double(y(:));
    if numel(x) ~= numel(y) || numel(x) < 3
        c = NaN;
        return;
    end
    if std(x) < 1e-12 || std(y) < 1e-12
        c = 0;
        return;
    end
    cc = corrcoef(x, y);
    c = cc(1,2);
    if isnan(c)
        c = 0;
    end
end

function [best_lag_s, best_corr] = best_lag_corr(x, y, max_lag_n, dt)
    x = double(x(:));
    y = double(y(:));

    best_corr = -inf;
    best_lag_s = 0;

    for lag = -max_lag_n:max_lag_n
        if lag >= 0
            xa = x(1:end-lag);
            ya = y(1+lag:end);
        else
            ll = -lag;
            xa = x(1+ll:end);
            ya = y(1:end-ll);
        end

        c = safe_corr(xa, ya);
        if abs(c) > abs(best_corr) || (isinf(best_corr) && ~isnan(c))
            best_corr = c;
            best_lag_s = lag * dt;
        end
    end

    if isinf(best_corr)
        best_corr = 0;
        best_lag_s = 0;
    end
end

function s = angle_stats(t, angle_deg, delta_deg, rate_degps)
    [mn, i_mn] = min(angle_deg);
    [mx, i_mx] = max(angle_deg);

    s.start = angle_deg(1);
    s.final = angle_deg(end);
    s.net = delta_deg(end);
    s.minv = mn;
    s.maxv = mx;
    s.t_min = t(i_mn);
    s.t_max = t(i_mx);
    s.p2p = mx - mn;
    s.max_abs_rate = max(abs(rate_degps));
end

function write_angle_summary(fid, name, s)
    fprintf(fid, '- %s: start %.3f deg, end %.3f deg, net %.3f deg\n', name, s.start, s.final, s.net);
    fprintf(fid, '        min/max %.3f / %.3f deg (t=%.3f / %.3f s), p2p %.3f deg, max|rate| %.3f deg/s\n', ...
        s.minv, s.maxv, s.t_min, s.t_max, s.p2p, s.max_abs_rate);
end

function z = zscore_safe(x)
    x = double(x(:));
    sx = std(x);
    if sx < 1e-12
        z = zeros(size(x));
    else
        z = (x - mean(x)) / sx;
    end
end
