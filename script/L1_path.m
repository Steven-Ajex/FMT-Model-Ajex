% 提取数据
L1_x = L1_WP_x.data;
L1_y = L1_WP_y.data;
x = INS_Out.x_R.data;
y = INS_Out.y_R.data;

% 绘图
figure;
plot( L1_y, L1_x, '-o', 'LineWidth', 2, 'MarkerSize', 6, 'Color', [0 0.447 0.741]); % L1 航点轨迹（蓝色）
hold on;
plot(y, x, '-o', 'LineWidth', 1, 'MarkerSize', 3, 'Color', [0.85 0.325 0.098]);     % 实际轨迹（细橙色）

% 添加标签和图例
xlabel('Y Position');
ylabel('X Position');
title('L1 Guidance vs Actual Path');
legend('L1 Waypoints', 'Actual Trajectory');
grid on;
axis equal;
