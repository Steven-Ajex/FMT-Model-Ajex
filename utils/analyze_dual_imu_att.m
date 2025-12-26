function T = analyze_dual_imu_att(source, save_path)
%ANALYZE_DUAL_IMU_ATT Align and plot dual_imu_att timeseries data.
% Usage:
%   T = analyze_dual_imu_att;                        % read dual_imu_att from base workspace
%   T = analyze_dual_imu_att('file.mat');            % load struct from MAT file
%   T = analyze_dual_imu_att(dual_imu_att);          % pass struct directly
%   T = analyze_dual_imu_att(src, 'att_out.csv');    % additionally save attitude to CSV/MAT
%
% Returns a table T with a common time base (seconds) and one column per
% signal (vector signals expanded to x/y/z). Generates a quick-look figure.

if nargin < 1
    source = [];
end
if nargin < 2
    save_path = [];
end

data = resolve_source(source);

if ~isfield(data, 'timestamp_ms') || ~isa(data.timestamp_ms, 'timeseries')
    error('dual_imu_att.timestamp_ms timeseries is required.');
end

time_ms = double(data.timestamp_ms.Data(:));
if isempty(time_ms)
    error('timestamp_ms contains no samples.');
end
time_ms = time_ms - time_ms(1);
t_s = time_ms / 1000;

T = table(t_s, 'VariableNames', {'t_s'});
fields = setdiff(fieldnames(data), {'timestamp_ms'});

for i = 1:numel(fields)
    name = fields{i};
    ts = data.(name);
    if ~isa(ts, 'timeseries')
        warning('Skipping %s because it is not a timeseries.', name);
        continue;
    end

    y = squeeze(ts.Data);
    y = reshape(y, size(y, 1), []); % force 2-D

    % Align to t_s using ts.Time when available, otherwise use sample index.
    if numel(ts.Time) == size(y, 1)
        y = interp1(ts.Time, y, t_s, 'linear', 'extrap');
    else
        if size(y, 1) ~= numel(t_s)
            warning('%s length (%d) does not match timestamp length (%d); truncating/padding with NaN.', ...
                name, size(y, 1), numel(t_s));
        end
        y = resize_to_length(y, numel(t_s));
    end

    % Expand vector components to individual columns.
    axis_names = {'x','y','z','w'};
    n_cols = size(y, 2);
    for c = 1:n_cols
        if n_cols == 1
            col_name = name;
        else
            idx = min(c, numel(axis_names));
            col_name = sprintf('%s_%s', name, axis_names{idx});
        end
        T.(col_name) = y(:, c);
    end
end

T = compute_attitude(T);
if ~isempty(save_path)
    save_attitude(T, save_path);
end
plot_dual_imu(T);

end

function data = resolve_source(source)
if nargin == 0 || isempty(source)
    if evalin('base', 'exist(''dual_imu_att'', ''var'')')
        data = evalin('base', 'dual_imu_att');
    else
        error('No input provided and dual_imu_att not found in base workspace.');
    end
elseif ischar(source) || isstring(source)
    s = load(source);
    if isfield(s, 'dual_imu_att')
        data = s.dual_imu_att;
    else
        fns = fieldnames(s);
        if numel(fns) == 1
            data = s.(fns{1});
        else
            error('MAT file does not contain dual_imu_att.');
        end
    end
else
    data = source;
end
end

function y = resize_to_length(y, new_len)
old_len = size(y, 1);
if old_len == new_len
    return;
elseif old_len > new_len
    y = y(1:new_len, :);
else
    y = [y; nan(new_len - old_len, size(y, 2))];
end
end

function plot_dual_imu(T)
t = T.t_s;

figure('Name', 'dual\_imu\_att overview', 'NumberTitle', 'off');
if exist('tiledlayout', 'file')
    tl = tiledlayout(5,2, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile(tl);
    plot(t, get_var(T, 'hinge_theta'), 'LineWidth', 1);
    grid on; xlabel('t [s]'); ylabel('hinge\_theta [rad]');
    title('Hinge');

    nexttile(tl);
    plot3axis(t, T, 'acc_b_L', 'Acc B L');
    nexttile(tl);
    plot3axis(t, T, 'acc_b_R', 'Acc B R');
    nexttile(tl);
    plot3axis(t, T, 'acc_b_cg', 'Acc B CG');

    nexttile(tl);
    plot3axis(t, T, 'mag_b_L', 'Mag B L');
    nexttile(tl);
    plot3axis(t, T, 'mag_b_R', 'Mag B R');
    nexttile(tl);
    plot3axis(t, T, 'mag_b_cg', 'Mag B CG');

    nexttile(tl);
    plot3axis(t, T, 'gyr_b', 'Gyro B');
    nexttile(tl);
    plot_att(t, T);

    if exist('sgtitle', 'file')
        sgtitle(tl, 'dual\_imu\_att quick look');
    end
else
    % Fallback for older MATLAB: use subplots.
    subplot(5,2,1);
    plot(t, get_var(T, 'hinge_theta'), 'LineWidth', 1);
    grid on; xlabel('t [s]'); ylabel('hinge\_theta [rad]');
    title('Hinge');

    subplot(5,2,2); plot3axis(t, T, 'acc_b_L',  'Acc B L');
    subplot(5,2,3); plot3axis(t, T, 'acc_b_R',  'Acc B R');
    subplot(5,2,4); plot3axis(t, T, 'acc_b_cg', 'Acc B CG');
    subplot(5,2,5); plot3axis(t, T, 'mag_b_L',  'Mag B L');
    subplot(5,2,6); plot3axis(t, T, 'mag_b_R',  'Mag B R');
    subplot(5,2,7); plot3axis(t, T, 'mag_b_cg', 'Mag B CG');
    subplot(5,2,8); plot3axis(t, T, 'gyr_b',    'Gyro B');
    subplot(5,2,9); plot_att(t, T);
    if exist('suptitle', 'file')
        suptitle('dual\_imu\_att quick look');
    end
end
end

function plot3axis(t, T, prefix, ttl)
y = get_var(T, prefix);
if isempty(y)
    title(sprintf('%s (missing)', ttl));
    return;
end
plot(t, y, 'LineWidth', 1);
grid on; xlabel('t [s]');
ylabel(prefix);
legend(comp_labels(size(y,2)), 'Location', 'best');
title(ttl);
end

function y = get_var(T, prefix)
cols = startsWith(T.Properties.VariableNames, prefix);
if ~any(cols)
    y = [];
    return;
end
y = table2array(T(:, cols));
end

function labels = comp_labels(n)
axes = {'x','y','z','w'};
labels = axes(1:min(n, numel(axes)));
if n > numel(axes)
    for i = numel(axes)+1:n
        labels{end+1} = sprintf('c%d', i);
    end
end
end

function T = compute_attitude(T)
acc = get_var(T, 'acc_b_cg');
mag = get_var(T, 'mag_b_cg');
if isempty(acc) || size(acc,2) < 3 || isempty(mag) || size(mag,2) < 3
    warning('acc_b_cg or mag_b_cg missing; skip attitude computation.');
    return;
end

% Auto-detect z-down gravity (acc_z ~ -g) and flip to +g along +Z.
az_nonan = acc(:,3);
az_nonan = az_nonan(~isnan(az_nonan));
if ~isempty(az_nonan)
    med_az = median(az_nonan);
    if med_az < 0
        acc = -acc;
        warning('acc_b_cg median az=%.3f (z-down); flipping acc sign for attitude calc.', med_az);
    end
end
ax = acc(:,1); ay = acc(:,2); az = acc(:,3);
mx = mag(:,1); my = mag(:,2); mz = mag(:,3);

roll_acc = atan2(ay, az);
pitch_acc = atan2(-ax, sqrt(ay .* ay + az .* az));
mxh = mx .* cos(pitch_acc) + mz .* sin(pitch_acc);
myh = mx .* sin(roll_acc) .* sin(pitch_acc) + my .* cos(roll_acc) - mz .* sin(roll_acc) .* cos(pitch_acc);
yaw_mag = atan2(myh, mxh);

T.roll_acc = roll_acc;
T.pitch_acc = pitch_acc;
T.yaw_mag = yaw_mag;

% Complementary filter with gyro (body-frame average), if present.
gyr = get_var(T, 'gyr_b');
if isempty(gyr) || size(gyr,2) < 3
    warning('gyr_b missing; attitude uses acc/mag only.');
    T.roll = roll_acc;
    T.pitch = pitch_acc;
    T.yaw = yaw_mag;
    return;
end

p = gyr(:,1); q = gyr(:,2); r = gyr(:,3); % rad/s body rates
dt = [0; diff(T.t_s)];
n = numel(dt);
roll_cf = roll_acc;
pitch_cf = pitch_acc;
yaw_cf = yaw_mag;
alpha = 0.98; % gyro weight
for k = 2:n
    cp = cos(roll_cf(k-1)); sp = sin(roll_cf(k-1));
    ct = cos(pitch_cf(k-1)); st = sin(pitch_cf(k-1));
    if abs(ct) < 1e-3
        ct = sign(ct) * 1e-3; % avoid tan/cos blow-up
    end
    phi_dot = p(k-1) + q(k-1) * sp * st / ct + r(k-1) * cp * st / ct;
    theta_dot = q(k-1) * cp - r(k-1) * sp;
    psi_dot = q(k-1) * sp / ct + r(k-1) * cp / ct;

    phi_pred = roll_cf(k-1) + phi_dot * dt(k);
    theta_pred = pitch_cf(k-1) + theta_dot * dt(k);
    psi_pred = yaw_cf(k-1) + psi_dot * dt(k);

    roll_cf(k) = alpha * phi_pred + (1 - alpha) * roll_acc(k);
    pitch_cf(k) = alpha * theta_pred + (1 - alpha) * pitch_acc(k);
    yaw_cf(k) = alpha * psi_pred + (1 - alpha) * yaw_mag(k);
end

T.roll_cf = roll_cf;
T.pitch_cf = pitch_cf;
T.yaw_cf = yaw_cf;
T.roll = roll_cf;
T.pitch = pitch_cf;
T.yaw = yaw_cf;
end

function plot_att(t, T)
vars = T.Properties.VariableNames;
has_cf = all(ismember({'roll_cf','pitch_cf','yaw_cf'}, vars));
has_acc = all(ismember({'roll_acc','pitch_acc','yaw_mag'}, vars));
has_base = all(ismember({'roll','pitch','yaw'}, vars));
if ~has_cf && ~has_acc && ~has_base
    title('Att (missing)');
    return;
end
hold on;
if has_cf
    plot(t, table2array(T(:, {'roll_cf','pitch_cf','yaw_cf'})), 'LineWidth', 1.2);
end
if has_acc
    plot(t, table2array(T(:, {'roll_acc','pitch_acc','yaw_mag'})), '--', 'LineWidth', 1);
end
if ~has_cf && ~has_acc && has_base
    plot(t, table2array(T(:, {'roll','pitch','yaw'})), 'LineWidth', 1.2);
end
hold off;
grid on; xlabel('t [s]'); ylabel('rad');
legend_entries = {};
if has_cf
    legend_entries = [legend_entries, {'roll_cf','pitch_cf','yaw_cf'}];
end
if has_acc
    legend_entries = [legend_entries, {'roll_acc','pitch_acc','yaw_mag'}];
end
if ~isempty(legend_entries)
    legend(legend_entries, 'Location', 'best');
end
title('Att RPY (cg acc/mag + gyro CF)');
end

function save_attitude(T, path_out)
if endsWith(path_out, '.mat', 'IgnoreCase', true)
    att_table = T;
    save(path_out, 'att_table');
elseif endsWith(path_out, '.csv', 'IgnoreCase', true)
    writetable(T, path_out);
else
    warning('Unknown extension for %s. Use .csv or .mat; skipping save.', path_out);
end
end
