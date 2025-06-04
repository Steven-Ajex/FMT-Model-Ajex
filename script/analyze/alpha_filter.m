% 假设原始数据已在结构体中
voltage = BATTERY.voltage.data;

% 滤波参数
alpha = 0.1;  % 越小越平滑
filter_voltage = zeros(size(voltage));  % 初始化滤波结果数组

% 初始化滤波值
filter_voltage(1) = voltage(1);

% Alpha滤波（递归实现）
for i = 2:length(voltage)
    filter_voltage(i) = alpha * voltage(i) + (1 - alpha) * filter_voltage(i-1);
end

% 绘图对比原始数据与滤波数据
figure;
plot(voltage, 'r--', 'DisplayName', '原始电压');
hold on;
plot(filter_voltage, 'b-', 'LineWidth', 1.5, 'DisplayName', '滤波后电压');
grid on;
xlabel('样本点');
ylabel('电压 (V)');
title('Alpha滤波 - 电压平滑处理');
legend show;
