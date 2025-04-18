clc
clear variables
close all

%% Bag

bag = rosbag('bags\assignment_1.1.bag');
bag = select(bag, ...
    'Topic', '/odom' ...
    );

bag_msgs = readMessages(bag, 'DataFormat', 'struct');

x = zeros(length(bag_msgs), 1);
y = zeros(length(bag_msgs), 1);
for idx = 1:length(bag_msgs)
    x(idx) = bag_msgs{idx}.Pose.Pose.Position.X;
    y(idx) = bag_msgs{idx}.Pose.Pose.Position.Y;
end


%% Plots

reset(0)
set(0, 'DefaultFigureNumberTitle', 'off');
set(0, 'DefaultFigureWindowStyle', 'docked');
set(0, 'DefaultLineLineWidth', 1.2);
set(0, 'defaultaxesfontsize', 12);

% Trajectory
figure('Name', 'Bag play results')
nexttile
hold on
grid on

plot(x, y)

title('Trajectory from /odom')
xlabel('x [m]')
ylabel('y [m]')