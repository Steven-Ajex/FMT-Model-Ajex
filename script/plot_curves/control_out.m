% 读取 timestamp
t = double(Control_Out.timestamp.Data);     % 先转成 double
u = double(Control_Out.actuator_cmd.Data);  % N×M

% 如果 timestamp 单位是 ms，可换算成秒
t = t / 1000;

% 取前4路
u1 = u(:,1);
u2 = u(:,2);
u3 = u(:,3);
u4 = u(:,4);

% 画到同一张图
figure;
plot(t, u1, 'LineWidth', 1.5); hold on;
plot(t, u2, 'LineWidth', 1.5);
plot(t, u3, 'LineWidth', 1.5);
plot(t, u4, 'LineWidth', 1.5);
grid on;

xlabel('Time (s)');
ylabel('actuator\_cmd');
title('Actuator Command');
legend('cmd1', 'cmd2', 'cmd3', 'cmd4');