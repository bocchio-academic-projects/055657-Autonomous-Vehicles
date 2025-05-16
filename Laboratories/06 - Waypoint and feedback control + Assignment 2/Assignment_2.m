clc
clear variables
close all

run('connect_to_WSL.m')

waypoints = load_waypoints();


%% Waypoints publisher

pub_waypoints = rospublisher('/waypoints', 'geometry_msgs/PoseWithCovariance');
msg_pose_covariance = rosmessage(pub_waypoints);

for i = 1:size(waypoints, 1)
    
    msg_pose_covariance.Covariance((i-1)*3 + 1) = waypoints(i, 1);
    msg_pose_covariance.Covariance((i-1)*3 + 2) = waypoints(i, 2);
    msg_pose_covariance.Covariance((i-1)*3 + 3) = waypoints(i, 3);
    % pause(1);

end

send(pub_waypoints, msg_pose_covariance);


%% Data loader

bag_prop = rosbag('bags/proportional_controller.bag');
bag_lyap = rosbag('bags/lyapunov_controller.bag');

odom_prop = select(bag_prop, 'Topic', '/odom');
odom_prop_msgs = readMessages(odom_prop, 'DataFormat', 'struct');
odom_prop_time = odom_prop.MessageList.Time - odom_prop.StartTime;
odom_prop_pose = cell2mat(arrayfun(@(msg) {[msg{1}.Pose.Pose.Position.X msg{1}.Pose.Pose.Position.Y]}, odom_prop_msgs));
odom_prop_linear_velocities = cell2mat(arrayfun(@(msg) {[msg{1}.Twist.Twist.Linear.X, msg{1}.Twist.Twist.Linear.Y, msg{1}.Twist.Twist.Linear.Z]}, odom_prop_msgs));
odom_prop_angular_velocities = cell2mat(arrayfun(@(msg) {[msg{1}.Twist.Twist.Angular.X, msg{1}.Twist.Twist.Angular.Y, msg{1}.Twist.Twist.Angular.Z]}, odom_prop_msgs));

odom_lyap = select(bag_lyap, 'Topic', '/odom');
odom_lyap_msgs = readMessages(odom_lyap, 'DataFormat', 'struct');
odom_lyap_time = odom_lyap.MessageList.Time - odom_lyap.StartTime;
odom_lyap_pose = cell2mat(arrayfun(@(msg) {[msg{1}.Pose.Pose.Position.X msg{1}.Pose.Pose.Position.Y]}, odom_lyap_msgs));
odom_lyap_linear_velocities = cell2mat(arrayfun(@(msg) {[msg{1}.Twist.Twist.Linear.X, msg{1}.Twist.Twist.Linear.Y, msg{1}.Twist.Twist.Linear.Z]}, odom_lyap_msgs));
odom_lyap_angular_velocities = cell2mat(arrayfun(@(msg) {[msg{1}.Twist.Twist.Angular.X, msg{1}.Twist.Twist.Angular.Y, msg{1}.Twist.Twist.Angular.Z]}, odom_lyap_msgs));



%% Plots
reset(0)
set(0, 'DefaultFigureNumberTitle', 'off');
set(0, 'DefaultFigureWindowStyle', 'docked');
set(0, 'DefaultLineLineWidth', 1.2);
set(0, 'defaultaxesfontsize', 12);

% Waypoints visualization
figure('Name', 'Waypoints')
tiles = tiledlayout(1, 2);

waypoints_tile = nexttile(tiles);
hold on
grid on

plot_waypoints(waypoints)

title('Waypoints')


% Proportional control strategy
figure('Name', 'Proportional control strategy')
tiles = tiledlayout(2, 2);

trajectory_prop_tile = nexttile(tiles, 1, [2, 1]);
hold on
grid on

plot_waypoints(waypoints)
plot(odom_prop_pose(:, 1), odom_prop_pose(:, 2), 'k')

title('Trajectory with proportional controller')


linear_velocity_prop_tile = nexttile(tiles, 2);
hold on
grid on

plot(odom_prop_time, 1e3 * odom_prop_linear_velocities(:, 1), 'DisplayName', 'odom');

title('Linear velocity X with proportional controller')
xlabel('Time [s]')
ylabel('v_x [mm/s]')
legend()


angular_velocity_prop_tile = nexttile(tiles, 4);
hold on
grid on

plot(odom_prop_time, odom_prop_angular_velocities(:, 3), 'DisplayName', 'odom');

title('Angular velocity Z with proportional controller')
xlabel('Time [s]')
ylabel('\omega_z [rad/s]')
legend()

linkaxes([nexttile(tiles, 2) nexttile(tiles, 4)], 'x')


% Lyapunov control strategy
figure('Name', 'Lyapunov control strategy')
tiles = tiledlayout(2, 2);

trajectory_lyap_tile = nexttile(tiles, 1, [2, 1]);
hold on
grid on

plot_waypoints(waypoints)
plot(odom_lyap_pose(:, 1), odom_lyap_pose(:, 2), 'k')
% quiver(velocity)

title('Trajectory with Lyapunov controller')


linear_velocity_lyap_tile = nexttile(tiles, 2);
hold on
grid on

plot(odom_lyap_time, 1e3 * odom_lyap_linear_velocities(:, 1), 'DisplayName', 'odom');

title('Linear velocity X with Lyapunov controller')
xlabel('Time [s]')
ylabel('v_x [mm/s]')
legend()


angular_velocity_lyap_tile = nexttile(tiles, 4);
hold on
grid on

plot(odom_lyap_time, odom_lyap_angular_velocities(:, 3), 'DisplayName', 'odom');

title('Angular velocity Z with Lyapunov controller')
xlabel('Time [s]')
ylabel('\omega_z [rad/s]')
legend()

linkaxes([nexttile(tiles, 2) nexttile(tiles, 4)], 'x')


%% Exports

exportgraphics(waypoints_tile, '.latex/img/MATLAB/waypoints.pdf');
exportgraphics(trajectory_prop_tile, '.latex/img/MATLAB/trajectory_proportional.pdf');
exportgraphics(linear_velocity_prop_tile, '.latex/img/MATLAB/linear_velocity_proportional.pdf');
exportgraphics(angular_velocity_prop_tile, '.latex/img/MATLAB/angular_velocity_proportional.pdf');
exportgraphics(trajectory_lyap_tile, '.latex/img/MATLAB/trajectory_lyapunov.pdf');
exportgraphics(linear_velocity_lyap_tile, '.latex/img/MATLAB/linear_velocity_lyapunov.pdf');
exportgraphics(angular_velocity_lyap_tile, '.latex/img/MATLAB/angular_velocity_lyapunov.pdf');



%% Functions

function plot_waypoints(waypoints)

x = waypoints(:, 1);
y = waypoints(:, 2);
theta = waypoints(:, 3);

plot(x, y, 'ro', 'MarkerSize', 4);
quiver(x, y, cos(theta), sin(theta), 0.1, 'r');

text(x(1), y(1), "1/11", 'Color', 'r', 'Position', [x(1), y(1)] + 0.2);
text(x(1), y(1), "6", 'Color', 'r', 'Position', [x(1) - 0.1, y(1)] - 0.2);
for idx = 2:length(x)-1
    if (idx ~= 6)
        text(x(idx), y(idx), num2str(idx), 'Color', 'r', 'Position', [x(idx), y(idx)] + 0.2);
    end
end

% axis equal
axis padded
xlabel('X [m]')
ylabel('Y [m]')

end

