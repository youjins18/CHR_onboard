function CHR_Decryptor(csv_file)
%CHR_DECRYPTOR Plot arm and whole-body CHR data from one bag CSV.

if nargin < 1 || strlength(string(csv_file)) == 0
    [file, folder] = uigetfile('*.csv', 'Select a CHR bag CSV');
    if isequal(file, 0); return; end
    csv_file = fullfile(folder, file);
end

data = readtable(csv_file, 'VariableNamingRule', 'preserve');
[t, v] = array_topic(data, '/chr/data_logging_msg', 53);

state = v(:, 1);
armed = v(:, 2);
hover = v(:, 3);
torque_request = v(:, 4);
torque_enabled = v(:, 5);
custom = v(:, 6);
trajectory = v(:, 7);
dob = v(:, 8);
base = v(:, 9:11);
base_command = v(:, 13:15);
joint = v(:, 17:19);
joint_command = v(:, 20:22);
tcp = v(:, 23:25);
tcp_roll_pitch = quat_to_roll_pitch(v(:, 26:29));
target = v(:, 30:32);
target_roll_pitch = quat_to_roll_pitch(v(:, 33:36));
position_residual = v(:, 37);
tilt_residual = v(:, 38);
sigma_min = v(:, 39);
condition_number = v(:, 40);
converged = v(:, 42);
px4_offset = v(:, 43:46);
feedback_fresh = v(:, 47);
target_fresh = v(:, 48);
tracking = v(:, 49);

figure('Name', 'CHR arm control', 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact');
for index = 1:3
    nexttile;
    plot(t, rad2deg(joint(:, index)), 'LineWidth', 1.2); hold on;
    plot(t, rad2deg(joint_command(:, index)), '--', 'LineWidth', 1.2);
    if index == 2; yline(0, ':k'); yline(30, ':r'); end
    ylabel(sprintf('J%d [deg]', index)); grid on;
    legend('measured', 'command', 'Location', 'best');
end
xlabel('time [s]');

figure('Name', 'CHR whole-body and TCP', 'Color', 'w');
tiledlayout(3, 2, 'TileSpacing', 'compact');
axis_name = {'X', 'Y', 'Z'};
for index = 1:3
    nexttile;
    plot(t, tcp(:, index), 'LineWidth', 1.2); hold on;
    plot(t, target(:, index), '--', 'LineWidth', 1.2);
    ylabel(sprintf('%s [m]', axis_name{index})); grid on;
    legend('TCP', 'target', 'Location', 'best');
end
nexttile;
plot(t, rad2deg(tcp_roll_pitch(:, 1)), 'LineWidth', 1.2); hold on;
plot(t, rad2deg(target_roll_pitch(:, 1)), '--', 'LineWidth', 1.2);
ylabel('roll [deg]'); grid on; legend('TCP', 'target', 'Location', 'best');
nexttile;
plot(t, rad2deg(tcp_roll_pitch(:, 2)), 'LineWidth', 1.2); hold on;
plot(t, rad2deg(target_roll_pitch(:, 2)), '--', 'LineWidth', 1.2);
ylabel('pitch [deg]'); xlabel('time [s]'); grid on;
legend('TCP', 'target', 'Location', 'best');
nexttile;
plot(t, base - base(1, :), 'LineWidth', 1.1); hold on;
plot(t, base_command - base(1, :), '--', 'LineWidth', 1.1);
ylabel('base delta [m]'); xlabel('time [s]'); grid on;
legend('x', 'y', 'z', 'x cmd', 'y cmd', 'z cmd', 'Location', 'best');

figure('Name', 'CHR DLS diagnostics', 'Color', 'w');
tiledlayout(2, 2, 'TileSpacing', 'compact');
nexttile;
semilogy(t, max(position_residual, eps), 'LineWidth', 1.2); hold on;
semilogy(t, max(tilt_residual, eps), 'LineWidth', 1.2);
ylabel('residual'); grid on;
legend('position [m]', 'tool-z tilt [rad]', 'Location', 'best');
nexttile;
semilogy(t, max(sigma_min, eps), 'LineWidth', 1.2); hold on;
semilogy(t, max(condition_number, eps), 'LineWidth', 1.2);
ylabel('conditioning'); grid on;
legend('sigma min', 'condition number', 'Location', 'best');
nexttile;
stairs(t, [state, converged, tracking], 'LineWidth', 1.1);
ylabel('state'); xlabel('time [s]'); grid on;
legend('controller state', 'converged', 'tracking', 'Location', 'best');
nexttile;
stairs(t, [armed, hover, torque_request, torque_enabled, custom, trajectory, dob, ...
    feedback_fresh, target_fresh], 'LineWidth', 1.0);
ylabel('flags'); xlabel('time [s]'); ylim([-0.1, 1.1]); grid on;
legend('armed', 'hover', 'torque req', 'torque', 'custom', 'trajectory', ...
    'DOB', 'feedback', 'target', 'Location', 'eastoutside');

fprintf('CHR: %d samples, %.2f s, max J2 %.2f deg, max TCP error %.3f m\n', ...
    size(v, 1), t(end) - t(1), max(rad2deg(joint(:, 2))), max(position_residual));
end


function [t, values] = array_topic(data, topic, width)
if ~all(ismember({'time_s', 'topic'}, data.Properties.VariableNames))
    error('CSV must be created by data_logging/rosbag_to_csv.py.');
end
mask = string(data.topic) == topic;
if ~any(mask); error('Topic %s is absent from this CSV.', topic); end
t = data.time_s(mask);
values = zeros(sum(mask), width);
for index = 0:width-1
    column = sprintf('data[%d]', index);
    if ~ismember(column, data.Properties.VariableNames)
        error('Missing CSV column %s.', column);
    end
    values(:, index + 1) = data{mask, column};
end
end


function roll_pitch = quat_to_roll_pitch(q)
norm_q = sqrt(sum(q.^2, 2));
norm_q(norm_q < eps) = 1;
q = q ./ norm_q;
x = q(:, 1); y = q(:, 2); z = q(:, 3); w = q(:, 4);
roll = atan2(2 .* (w .* x + y .* z), 1 - 2 .* (x.^2 + y.^2));
pitch = asin(max(-1, min(1, 2 .* (w .* y - z .* x))));
roll_pitch = [roll, pitch];
end
