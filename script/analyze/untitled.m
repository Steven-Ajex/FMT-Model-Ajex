remaining = BATTERY.remaining.data;
remaining_c = BATTERY.remaining_c.data;
remaining_v = BATTERY.remaining_v.data;

figure;
plot(remaining, 'r', 'LineWidth', 1.5); hold on;
plot(remaining_c, 'g', 'LineWidth', 1.5);
plot(remaining_v, 'b', 'LineWidth', 1.5);
hold off;

legend('Remaining', 'Remaining (Current)', 'Remaining (Voltage)');
xlabel('Sample Index');
ylabel('Battery Remaining (%)');
title('Battery Remaining Comparison');
grid on;
