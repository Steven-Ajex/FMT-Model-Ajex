
%% 加载数据（假设数据已加载到工作区，如 z_est_R 和 z_cmd_R）
% 如果数据未加载，请取消以下注释并替换为实际数据路径：
% load('your_data_file.mat');

%% 数据切片（从第2000个点开始）
start_idx = 2000;
h_R = z_est.data(start_idx:end);
h_R_ref = z_cmd.data(start_idx:end);
timestamp = z_est.time(start_idx:end);

%% 计算误差指标
abs_error = abs(h_R - h_R_ref);              % 绝对误差
relative_error = abs_error ./ (abs(h_R_ref) + eps);  % 相对误差（避免除零）
rmse = sqrt(mean((h_R - h_R_ref).^2));       % 均方根误差
mae = mean(abs_error);                       % 平均绝对误差
max_abs_error = max(abs_error);              % 最大绝对误差

% 打印误差统计结果
fprintf('===== 误差统计结果 =====\n');
fprintf('RMSE               = %.4f\n', rmse);
fprintf('MAE                = %.4f\n', mae);
fprintf('Max Absolute Error = %.4f\n', max_abs_error);
fprintf('========================\n');

%% 可视化分析
% 设置全局绘图样式
set(0, 'DefaultAxesFontSize', 10, 'DefaultLineLineWidth', 1.5);

% ---- 图1：参考值、估计值与误差曲线 ----
figure('Name', 'Reference vs. Estimated with Error', 'Position', [100 100 800 600]);

% 子图1：参考值与估计值对比
plot(timestamp, h_R, 'b');
ylim([-1,0]);
hold on;
plot(timestamp, h_R_ref, 'r--');
xlabel('Time (s)');
ylabel('Value');
title('Reference vs. Estimated (h\_R)');
legend('Estimated', 'Reference', 'Location', 'best');
grid on;

%% 保存结果（可选）
% 保存误差数据到MAT文件
save('error_analysis_results.mat', 'abs_error', 'rmse', 'mae', 'max_abs_error');

% 保存图形（可选）
% saveas(gcf, 'error_plots.png');