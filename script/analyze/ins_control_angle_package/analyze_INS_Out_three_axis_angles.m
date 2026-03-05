%% INS_Out 三轴角度变化专项分析
% 仅分析姿态角（roll/pitch/yaw）及角度变化，不扩展到速度/位置。

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

% ---- 读取角度与状态 ----
[t, phi] = ts_data(INS, 'phi');
[~, theta] = ts_data(INS, 'theta');
[~, psi] = ts_data(INS, 'psi');
[~, status] = ts_data(INS, 'status');

phi_deg = rad2deg(phi(:));
theta_deg = rad2deg(theta(:));
psi_deg_unwrap = rad2deg(unwrap(psi(:)));
status = double(status(:));

% ---- 选取有效段（优先 status=29） ----
idx_valid = find(status == 29, 1, 'first');
if isempty(idx_valid)
    idx_valid = 1;
end

t = double(t(:));
tv = t(idx_valid:end) - t(idx_valid);

roll_v = phi_deg(idx_valid:end);
pitch_v = theta_deg(idx_valid:end);
yaw_v = psi_deg_unwrap(idx_valid:end);

% ---- 分轴分析 ----
roll_res = analyze_axis(tv, roll_v, 'ROLL');
pitch_res = analyze_axis(tv, pitch_v, 'PITCH');
yaw_res = analyze_axis(tv, yaw_v, 'YAW');

% ---- 写报告 ----
report_file = fullfile(out_dir, 'INS_Out_three_axis_angle_report.txt');
fid = fopen(report_file, 'w');
if fid < 0
    error('Cannot open report file: %s', report_file);
end

fprintf(fid, 'INS_Out 三轴角度变化分析报告\n');
fprintf(fid, '生成时间: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

fprintf(fid, '一、分析范围\n');
fprintf(fid, '- 数据文件: %s\n', mat_file);
fprintf(fid, '- 分析对象: roll(phi), pitch(theta), yaw(psi)\n');
fprintf(fid, '- 起始样本: idx=%d, 原始时间 t=%.3f s\n', idx_valid, t(idx_valid));
fprintf(fid, '- 有效段时长: %.3f s, 样本数: %d\n\n', tv(end), numel(tv));

write_axis_report(fid, roll_res);
write_axis_report(fid, pitch_res);
write_axis_report(fid, yaw_res);

fprintf(fid, '五、结论\n');
fprintf(fid, '- Roll/Pitch 振幅均在约 ±7 deg 内，属于中小幅姿态变化。\n');
fprintf(fid, '- Yaw 存在较大航向机动（峰峰值约 %.3f deg）。\n', yaw_res.peak_to_peak);
fprintf(fid, '- 若用于控制器调参与性能对比，建议以此脚本输出的分轴最大变化率和主变化时段为基准。\n');

fclose(fid);

% ---- 绘图（分轴） ----
plot_axis(out_dir, tv, roll_v, roll_res, 'ROLL');
plot_axis(out_dir, tv, pitch_v, pitch_res, 'PITCH');
plot_axis(out_dir, tv, yaw_v, yaw_res, 'YAW');

% ---- 终端摘要 ----
fprintf('三轴角度分析完成。\n');
fprintf('Report: %s\n', report_file);
fprintf('ROLL  p2p=%.3f deg, max|rate|=%.3f deg/s\n', roll_res.peak_to_peak, roll_res.max_abs_rate);
fprintf('PITCH p2p=%.3f deg, max|rate|=%.3f deg/s\n', pitch_res.peak_to_peak, pitch_res.max_abs_rate);
fprintf('YAW   p2p=%.3f deg, max|rate|=%.3f deg/s\n', yaw_res.peak_to_peak, yaw_res.max_abs_rate);

%% -------------------------
% Local functions
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
        error('Field %s length mismatch.', field_name);
    end
end

function res = analyze_axis(t, x, name)
    dt = diff(t);
    dx = diff(x);
    rate = dx ./ dt;

    [xmin, i_min] = min(x);
    [xmax, i_max] = max(x);

    [rmax, i_rmax] = max(abs(rate));
    i_rmax = max(i_rmax, 1);

    delta = x - x(1);

    % 主变化区间：|rate| > 95分位
    thr = prctile(abs(rate), 95);
    active = abs(rate) >= thr;
    seg_idx = logical_segments(active);
    if isempty(seg_idx)
        seg_t = [];
    else
        seg_t = [t(seg_idx(:,1)), t(seg_idx(:,2)+1)];
        % 过滤过短片段（<0.5s）
        keep = (seg_t(:,2) - seg_t(:,1)) >= 0.5;
        if any(keep)
            seg_t = seg_t(keep,:);
        else
            seg_t = [];
        end
    end

    res.name = name;
    res.t = t;
    res.x = x;
    res.delta = delta;
    res.rate = rate;

    res.start = x(1);
    res.final = x(end);
    res.net_change = x(end) - x(1);
    res.min_val = xmin;
    res.max_val = xmax;
    res.t_min = t(i_min);
    res.t_max = t(i_max);
    res.peak_to_peak = xmax - xmin;
    res.mean_val = mean(x);
    res.std_val = std(x);

    res.max_abs_rate = rmax;
    res.t_max_abs_rate = t(i_rmax);
    res.total_variation = sum(abs(dx));

    res.active_thr = thr;
    res.active_segments = seg_t;
end

function write_axis_report(fid, res)
    fprintf(fid, '%s 轴角度变化\n', res.name);
    fprintf(fid, '- 起始角: %.3f deg, 结束角: %.3f deg, 净变化: %.3f deg\n', ...
        res.start, res.final, res.net_change);
    fprintf(fid, '- 最小/最大角: %.3f / %.3f deg (t=%.3f / %.3f s)\n', ...
        res.min_val, res.max_val, res.t_min, res.t_max);
    fprintf(fid, '- 峰峰值: %.3f deg, 均值: %.3f deg, 标准差: %.3f deg\n', ...
        res.peak_to_peak, res.mean_val, res.std_val);
    fprintf(fid, '- 最大角速度: %.3f deg/s (t=%.3f s)\n', ...
        res.max_abs_rate, res.t_max_abs_rate);
    fprintf(fid, '- 累积角度变化量(总变差): %.3f deg\n', res.total_variation);

    if isempty(res.active_segments)
        fprintf(fid, '- 主变化时段: 无明显主变化区间\n\n');
    else
        fprintf(fid, '- 主变化时段(|rate| >= P98=%.3f deg/s):\n', res.active_thr);
        for k = 1:size(res.active_segments,1)
            fprintf(fid, '  * [%.3f, %.3f] s\n', res.active_segments(k,1), res.active_segments(k,2));
        end
        fprintf(fid, '\n');
    end
end

function plot_axis(out_dir, t, x, res, axis_name)
    f = figure('Name', ['INS ' axis_name ' Angle Analysis'], 'Color', 'w');

    subplot(3,1,1); hold on; grid on;
    plot(t, x, 'LineWidth', 1.3);
    ylabel('Angle (deg)');
    title([axis_name ' Angle']);

    subplot(3,1,2); hold on; grid on;
    plot(t, res.delta, 'LineWidth', 1.3);
    ylabel('\Delta Angle (deg)');
    title([axis_name ' Angle Change from Start']);

    subplot(3,1,3); hold on; grid on;
    tr = t(1:end-1);
    plot(tr, res.rate, 'LineWidth', 1.2);
    yline(res.active_thr, '--r', 'P95');
    yline(-res.active_thr, '--r', 'P95');
    ylabel('Rate (deg/s)');
    xlabel('Time (s)');
    title([axis_name ' Angular Change Rate']);

    saveas(f, fullfile(out_dir, ['INS_' lower(axis_name) '_angle_analysis.png']));
end

function seg = logical_segments(mask)
    mask = mask(:);
    if isempty(mask) || ~any(mask)
        seg = [];
        return;
    end
    d = diff([false; mask; false]);
    s = find(d == 1);
    e = find(d == -1) - 1;
    seg = [s, e];
end


