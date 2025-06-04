%% 数据准备
% 提取实际轨迹数据（确保转换为普通数组）
x_actual = x_est.data(1000:5739);  % 切片从1000到5739
y_actual = y_est.data(1000:5739);

% 提取期望定点（兼容timeseries和普通数组）
if isa(x_cmd, 'timeseries')
    x_cmd_p = x_cmd.data(1);
    y_cmd_p = y_cmd.data(1);
else
    x_cmd_p = x_cmd(1);
    y_cmd_p = y_cmd(1);
end

%% 计算误差指标
% 1. 距离误差（每个点到定点的欧氏距离）
distance_error = sqrt((x_actual - x_cmd_p).^2 + (y_actual - y_cmd_p).^2);

% 2. 统计指标
mean_error = mean(distance_error);
max_error = max(distance_error);
std_error = std(distance_error);
rmse = sqrt(mean(distance_error.^2));

% 打印结果
fprintf('===== 定点精度分析 =====\n');
fprintf('平均误差: %.4f\n', mean_error);
fprintf('最大误差: %.4f\n', max_error);
fprintf('误差标准差: %.4f\n', std_error);
fprintf('RMSE: %.4f\n', rmse);
fprintf('=======================\n');

%% 可视化
figure('Position', [100, 100, 900, 600]);

% ---- 子图1：轨迹与定点 ----
plot(x_actual, y_actual, 'b-', 'LineWidth', 1.5); 
ylim([1.8,2.8]);
xlim([-2.8,-1.8]);
hold on;
plot(x_cmd_p, y_cmd_p, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
hold off;
xlabel('X Position'); ylabel('Y Position');
title('实际轨迹 vs 期望定点');
legend('实际轨迹', '期望定点', '起点', '终点', 'Location', 'best');
grid on; axis equal;
%% 保存结果（可选）
save('precision_analysis.mat', 'distance_error', 'mean_error', 'max_error', 'rmse');