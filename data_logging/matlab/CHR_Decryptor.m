function CHR_Decryptor(csv_path)
%CHR_DECRYPTOR Plot /chr/data_logging_msg exported by rosbag_to_csv.py.

if nargin < 1 || strlength(string(csv_path)) == 0
    [file, folder] = uigetfile('*.csv', 'Select chr_data_logging_msg.csv');
    if isequal(file, 0)
        return;
    end
    csv_path = fullfile(folder, file);
end

data = readtable(csv_path, 'VariableNamingRule', 'preserve');
if width(data) < 54
    error('CHR log needs timestamp plus 53 data columns; this file has %d columns.', width(data));
end

t_ns = double(data{:, 1});
t = (t_ns - t_ns(1)) * 1e-9;
v = double(data{:, 2:54});

state = v(:, 1);
armed = v(:, 2);
hover_stable = v(:, 3);
torque_request = v(:, 4);
torque_enabled = v(:, 5);
custom_mode = v(:, 6);
trajectory = v(:, 7);
dob = v(:, 8);
base = v(:, 9:11);
base_yaw = v(:, 12);
base_cmd = v(:, 13:15);
base_yaw_cmd = v(:, 16);
joint = v(:, 17:19);
joint_cmd = v(:, 20:22);
tcp = v(:, 23:25);
target = v(:, 30:32);
position_residual = v(:, 37);
orientation_residual = v(:, 38);
sigma_min = v(:, 39);
condition_number = v(:, 40);
converged = v(:, 42);
px4_offset_ned = v(:, 43:45);
yaw_offset_ned = v(:, 46);
feedback_fresh = v(:, 47);
target_fresh = v(:, 48);
tracking = v(:, 49);

figure('Name', 'CHR base tracking', 'Color', 'w');
tiledlayout(4, 1);
labels = {'x_{ENU} [m]', 'y_{ENU} [m]', 'z_{ENU} [m]'};
for axis = 1:3
    nexttile;
    plot(t, base(:, axis), 'LineWidth', 1.2); hold on;
    plot(t, base_cmd(:, axis), '--', 'LineWidth', 1.2);
    ylabel(labels{axis}); grid on;
    legend('measured', 'DLS command', 'Location', 'best');
end
nexttile;
plot(t, rad2deg(base_yaw), 'LineWidth', 1.2); hold on;
plot(t, rad2deg(base_yaw_cmd), '--', 'LineWidth', 1.2);
ylabel('yaw [deg]'); xlabel('time [s]'); grid on;
legend('measured', 'DLS command', 'Location', 'best');

figure('Name', 'CHR arm joints', 'Color', 'w');
tiledlayout(3, 1);
for joint_index = 1:3
    nexttile;
    plot(t, rad2deg(joint(:, joint_index)), 'LineWidth', 1.2); hold on;
    plot(t, rad2deg(joint_cmd(:, joint_index)), '--', 'LineWidth', 1.2);
    if joint_index == 2
        yline(0, ':k'); yline(30, ':r', 'J2 hardware limit');
    end
    ylabel(sprintf('J%d [deg]', joint_index)); grid on;
    legend('measured', 'command', 'Location', 'best');
end
xlabel('time [s]');

figure('Name', 'CHR EE tracking and DLS', 'Color', 'w');
tiledlayout(4, 1);
nexttile;
plot(t, tcp, 'LineWidth', 1.1); hold on;
set(gca, 'ColorOrderIndex', 1);
plot(t, target, '--', 'LineWidth', 1.1);
ylabel('EE xyz [m]'); grid on;
legend('x', 'y', 'z', 'x_d', 'y_d', 'z_d', 'Location', 'best');
nexttile;
semilogy(t, max(position_residual, eps), 'LineWidth', 1.2); hold on;
semilogy(t, max(orientation_residual, eps), 'LineWidth', 1.2);
ylabel('DLS residual'); grid on;
legend('position [m]', 'orientation [rad]', 'Location', 'best');
nexttile;
semilogy(t, max(sigma_min, eps), 'LineWidth', 1.2); hold on;
semilogy(t, max(condition_number, eps), 'LineWidth', 1.2);
ylabel('conditioning'); grid on;
legend('\sigma_{min}', 'condition number', 'Location', 'best');
nexttile;
stairs(t, converged, 'LineWidth', 1.2); hold on;
stairs(t, tracking, 'LineWidth', 1.2);
ylabel('boolean'); xlabel('time [s]'); ylim([-0.1 1.1]); grid on;
legend('DLS converged', 'tracking accepted', 'Location', 'best');

figure('Name', 'CHR experiment state', 'Color', 'w');
tiledlayout(3, 1);
nexttile;
stairs(t, [armed, hover_stable, torque_request, torque_enabled], 'LineWidth', 1.1);
ylim([-0.1 1.1]); grid on;
legend('armed', 'hover stable', 'torque request', 'torque enabled', 'Location', 'best');
nexttile;
stairs(t, [custom_mode, trajectory, dob, feedback_fresh, target_fresh], 'LineWidth', 1.1);
ylim([-0.1 1.1]); grid on;
legend('custom', 'trajectory', 'DOB', 'feedback fresh', 'target fresh', 'Location', 'best');
nexttile;
plot(t, px4_offset_ned, 'LineWidth', 1.1); hold on;
plot(t, yaw_offset_ned, '--k', 'LineWidth', 1.1);
ylabel('PX4 offsets'); xlabel('time [s]'); grid on;
legend('N', 'E', 'D', 'yaw', 'Location', 'best');

fprintf('Samples: %d, duration: %.2f s\n', height(data), t(end));
fprintf('State range: %d..%d, max J2: %.2f deg\n', ...
    min(state), max(state), max(rad2deg(joint(:, 2))));
end
