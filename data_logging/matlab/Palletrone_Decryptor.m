function Palletrone_Decryptor(csv_file)
%PALLETRONE_DECRYPTOR Plot the verified Palletrone controller log.

if nargin < 1 || strlength(string(csv_file)) == 0
    [file, folder] = uigetfile('*.csv', 'Select a Palletrone bag CSV');
    if isequal(file, 0); return; end
    csv_file = fullfile(folder, file);
end

data = readtable(csv_file, 'VariableNamingRule', 'preserve');
[t, v] = array_topic(data, '/data_logging_msg', 91);

position = v(:, 1:3);
position_command = v(:, 4:6);
velocity = v(:, 7:9);
velocity_command = v(:, 10:12);
attitude = v(:, 13:15);
attitude_command = v(:, 16:18);
angular_velocity = v(:, 19:21);
angular_velocity_command = v(:, 22:24);
force_command = v(:, 25:27);
torque_command = v(:, 28:30);
dob_torque = v(:, 32:34);
rotor_thrust = v(:, 35:38);
servo = v(:, 39:42);
servo_command = v(:, 43:46);
acceleration = v(:, 47:49);
acceleration_command = v(:, 50:52);
motor_percent = 100 * max(0, min(1, v(:, 69:72)));

figure('Name', 'Palletrone flight control', 'Color', 'w');
tiledlayout(3, 3, 'TileSpacing', 'compact');
axis_name = {'X', 'Y', 'Z'};
for index = 1:3
    nexttile;
    plot(t, position(:, index), 'LineWidth', 1.2); hold on;
    plot(t, position_command(:, index), '--', 'LineWidth', 1.2);
    ylabel(sprintf('%s [m]', axis_name{index})); grid on;
    legend('measured', 'command', 'Location', 'best');
end

attitude_name = {'roll', 'pitch', 'yaw'};
for index = 1:3
    nexttile;
    plot(t, rad2deg(attitude(:, index)), 'LineWidth', 1.2); hold on;
    plot(t, rad2deg(attitude_command(:, index)), '--', 'LineWidth', 1.2);
    ylabel(sprintf('%s [deg]', attitude_name{index})); grid on;
    legend('measured', 'command', 'Location', 'best');
end

nexttile;
plot(t, torque_command, 'LineWidth', 1.1); hold on;
plot(t, dob_torque, '--', 'LineWidth', 1.1);
ylabel('torque'); xlabel('time [s]'); grid on;
legend('T_x', 'T_y', 'T_z', 'DOB_x', 'DOB_y', 'DOB_z', 'Location', 'best');

nexttile;
plot(t, motor_percent, 'LineWidth', 1.1);
ylabel('motor [%]'); xlabel('time [s]'); ylim([0, 100]); grid on;
legend('M1', 'M2', 'M3', 'M4', 'Location', 'best');

nexttile;
plot(t, rad2deg(servo), 'LineWidth', 1.0); hold on;
set(gca, 'ColorOrderIndex', 1);
plot(t, rad2deg(servo_command), '--', 'LineWidth', 1.0);
ylabel('servo [deg]'); xlabel('time [s]'); grid on;
legend('S1', 'S2', 'S3', 'S4', ...
    'S1 cmd', 'S2 cmd', 'S3 cmd', 'S4 cmd', 'Location', 'eastoutside');

figure('Name', 'Palletrone motion and allocation', 'Color', 'w');
tiledlayout(4, 3, 'TileSpacing', 'compact');

for index = 1:3
    nexttile;
    plot(t, velocity(:, index), 'LineWidth', 1.2); hold on;
    plot(t, velocity_command(:, index), '--', 'LineWidth', 1.2);
    ylabel(sprintf('v_%s [m/s]', lower(axis_name{index}))); grid on;
    legend('measured', 'command', 'Location', 'best');
end

for index = 1:3
    nexttile;
    plot(t, rad2deg(angular_velocity(:, index)), 'LineWidth', 1.2); hold on;
    plot(t, rad2deg(angular_velocity_command(:, index)), '--', 'LineWidth', 1.2);
    ylabel(sprintf('%s rate [deg/s]', attitude_name{index})); grid on;
    legend('measured', 'command', 'Location', 'best');
end

for index = 1:3
    nexttile;
    plot(t, acceleration(:, index), 'LineWidth', 1.2); hold on;
    plot(t, acceleration_command(:, index), '--', 'LineWidth', 1.2);
    ylabel(sprintf('a_%s [m/s^2]', lower(axis_name{index}))); xlabel('time [s]'); grid on;
    legend('measured', 'command', 'Location', 'best');
end

nexttile;
plot(t, force_command, 'LineWidth', 1.1);
ylabel('force command [N]'); xlabel('time [s]'); grid on;
legend('F_x', 'F_y', 'F_z', 'Location', 'best');

nexttile([1, 2]);
plot(t, rotor_thrust, 'LineWidth', 1.1);
ylabel('rotor thrust [N]'); xlabel('time [s]'); grid on;
legend('f_1', 'f_2', 'f_3', 'f_4', 'Location', 'best');

fprintf('Palletrone: %d samples, %.2f s, maximum motor output %.1f %%\n', ...
    size(v, 1), t(end) - t(1), max(motor_percent, [], 'all'));

[j1_time, j1_angle] = scalar_topic(data, '/j1_angle');
[j1_command_time, j1_command] = scalar_topic(data, '/j1_angle_command');
figure('Name', 'J1 and bar', 'Color', 'w');
plot(j1_time, rad2deg(j1_angle), 'LineWidth', 1.2); hold on;
plot(j1_command_time, rad2deg(j1_command), '--', 'LineWidth', 1.2);
xlabel('time [s]'); ylabel('J1 [deg]'); grid on;
legend('measured', 'command', 'Location', 'best');
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


function [t, values] = scalar_topic(data, topic)
mask = string(data.topic) == topic;
if ~any(mask); error('Topic %s is absent from this CSV.', topic); end
t = data.time_s(mask);
values = data{mask, 'data'};
end
