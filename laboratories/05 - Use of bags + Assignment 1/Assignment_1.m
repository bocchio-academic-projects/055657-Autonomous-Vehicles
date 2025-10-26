clc
clear variables
close all

%% Part A
% Starting from a Bag file provided, evaluate at each time:
% - Minimum distance respect to obstacles
% - An estimate of /cmd_vel sequence provided to the robot

bag = rosbag('bags/provided.bag');

scan = select(bag, 'Topic', '/scan');
scan_msgs = readMessages(scan, 'DataFormat', 'struct');
scan_time = scan.MessageList.Time - scan.StartTime;
scan_ranges = cell2mat(cellfun(@(msg) msg.Ranges(:)', scan_msgs, 'UniformOutput', false));

imu = select(bag, 'Topic', '/imu');
imu_msgs = readMessages(imu, 'DataFormat', 'struct');
imu_time = imu.MessageList.Time - imu.StartTime;
imu_linear_accelerations = cell2mat(arrayfun(@(msg) {[msg{1}.LinearAcceleration.X, msg{1}.LinearAcceleration.Y, msg{1}.LinearAcceleration.Z]}, imu_msgs));
imu_angular_velocities = cell2mat(arrayfun(@(msg) {[msg{1}.AngularVelocity.X, msg{1}.AngularVelocity.Y, msg{1}.AngularVelocity.Z]}, imu_msgs));

odom = select(bag, 'Topic', '/odom');
odom_msgs = readMessages(odom, 'DataFormat', 'struct');
odom_time = odom.MessageList.Time - odom.StartTime;
odom_pose = cell2mat(arrayfun(@(msg) {[msg{1}.Pose.Pose.Position.X msg{1}.Pose.Pose.Position.Y]}, odom_msgs));
odom_linear_velocities = cell2mat(arrayfun(@(msg) {[msg{1}.Twist.Twist.Linear.X, msg{1}.Twist.Twist.Linear.Y, msg{1}.Twist.Twist.Linear.Z]}, odom_msgs));
odom_angular_velocities = cell2mat(arrayfun(@(msg) {[msg{1}.Twist.Twist.Angular.X, msg{1}.Twist.Twist.Angular.Y, msg{1}.Twist.Twist.Angular.Z]}, odom_msgs));


%% Part B
% Evaluate how accurate it's the /cmd_vel just estimated

% MATLAB->WSL communication is tremendously time-inaccurate.
% This task has been performed entirerly in WLS directly, after exporting
% the /cmd_vel commands via python script in order to keep accurate timing.
% 
% run('connect_to_WSL.m')
% 
% pub_cmd_vel = ros.Publisher([], '/cmd_vel', 'geometry_msgs/Twist');
% msg_twist = rosmessage(pub_cmd_vel);
% 
% clear bag_replay odom_replay
% delete 'bags/obtained.bag'
% wsl('nohup rosbag record /odom /cmd_vel -O bags/obtained.bag');
% 
% dt = diff([odom_time; odom_time(end)]);
% for idx = 1:length(odom_time)/4
%     msg_twist.Linear.X = odom_linear_velocities(idx, 1);
%     msg_twist.Angular.Z = odom_angular_velocities(idx, 3);
%     send(pub_cmd_vel, msg_twist);
%     pause(dt(idx)-0.005);
% end
% wsl('pkill rosbag');

bag_replay = rosbag('bags/replayed.bag');

odom_replay = select(bag_replay, 'Topic', '/odom');
odom_msgs_replay = readMessages(odom_replay, 'DataFormat', 'struct');
odom_time_replay = odom_replay.MessageList.Time - odom_replay.StartTime;
odom_pose_replay = cell2mat(arrayfun(@(msg) {[msg{1}.Pose.Pose.Position.X msg{1}.Pose.Pose.Position.Y]}, odom_msgs_replay));
odom_linear_velocities_replay = cell2mat(arrayfun(@(msg) {[msg{1}.Twist.Twist.Linear.X, msg{1}.Twist.Twist.Linear.Y, msg{1}.Twist.Twist.Linear.Z]}, odom_msgs_replay));
odom_angular_velocities_replay = cell2mat(arrayfun(@(msg) {[msg{1}.Twist.Twist.Angular.X, msg{1}.Twist.Twist.Angular.Y, msg{1}.Twist.Twist.Angular.Z]}, odom_msgs_replay));


%% Plots
reset(0)
set(0, 'DefaultFigureNumberTitle', 'off');
set(0, 'DefaultFigureWindowStyle', 'docked');
set(0, 'DefaultLineLineWidth', 1.2);
set(0, 'defaultaxesfontsize', 12);

% Assignment part A
figure('Name', 'Assignment #1A')
tiles = tiledlayout(2, 2);

scan_tile = nexttile(tiles, 1, [2, 1]);
hold on
grid on

plot(scan_time, min(scan_ranges, [], 2) * 1e3)

axis tight
ylim('padded')

title('Minimum distance from object')
xlabel('Time [s]')
ylabel('min(distance) [mm]')


linear_velocity_tile = nexttile(tiles, 2);
hold on
grid on

% plot(imu_time, trapz(imu_linear_accelerations(:, 1)), 'DisplayName', 'imu (itegrated)');
plot(odom_time, 1e3 * odom_linear_velocities(:, 1), 'DisplayName', 'odom');

title('Linear velocity X')
xlabel('Time [s]')
ylabel('v_x [mm/s]')
legend()


angular_velocity_tile = nexttile(tiles, 4);
hold on
grid on

% plot(imu_time, imu_angular_velocities(:, 3), 'DisplayName', 'imu');
plot(odom_time, odom_angular_velocities(:, 3), 'DisplayName', 'odom');

title('Angular velocity Z')
xlabel('Time [s]')
ylabel('\omega_z [rad/s]')
legend()

linkaxes([nexttile(tiles, 1) nexttile(tiles, 2) nexttile(tiles, 4)], 'x')


% Assignment part B
figure('Name', 'Assignment #1B')
tiles = tiledlayout(2, 2);

% Trajectory comparison
trajectories_comparison_tile = nexttile(tiles, 1, [2, 1]);
hold on
grid on

plot(odom_pose(:, 1), odom_pose(:, 2), 'b-', 'DisplayName', 'Provided');
plot(odom_pose_replay(:, 1), odom_pose_replay(:, 2), 'r-', 'DisplayName', 'Estimated');

axis padded
% axis equal

title('Trajectories comparison')
xlabel('X [m]')
ylabel('Y [m]')
legend()

% Linear velocity x comparison
linear_velocity_comparison_tile = nexttile(tiles);
hold on
grid on

plot(odom_time, 1e3 * odom_linear_velocities(:, 1), 'b', 'DisplayName', 'Provided');
plot(odom_time_replay, 1e3 * odom_linear_velocities_replay(:, 1), 'r', 'DisplayName', 'Estimated');

title('Linear velocity X comparison')
xlabel('Time [s]')
ylabel('V_x [mm/s]')
legend()


% Angular velocity z comparison
angular_velocity_comparison_tile = nexttile(tiles);
hold on
grid on

plot(odom_time, odom_angular_velocities(:, 3), 'b', 'DisplayName', 'Provided');
plot(odom_time_replay, odom_angular_velocities_replay(:, 3), 'r', 'DisplayName', 'Estimated');

title('Angular velocity Z comparison')
xlabel('Time [s]')
ylabel('w_z [rad/s]')
legend()


%% Exports

exportgraphics(scan_tile, '.latex/img/MATLAB/minimum_distance.pdf');
exportgraphics(linear_velocity_tile, '.latex/img/MATLAB/linear_velocity.pdf');
exportgraphics(angular_velocity_tile, '.latex/img/MATLAB/angular_velocity.pdf');
exportgraphics(trajectories_comparison_tile, '.latex/img/MATLAB/trajectories_comparison.pdf');
exportgraphics(linear_velocity_comparison_tile, '.latex/img/MATLAB/linear_velocity_comparison.pdf');
exportgraphics(angular_velocity_comparison_tile, '.latex/img/MATLAB/angular_velocity_comparison.pdf');