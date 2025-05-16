clc
clear variables
close all

run('connect_to_WSL.m')

%% Turtlebot setup

turtle = turtlebot(getenv('ROS_MASTER'));
turtle.Velocity.TopicName = '/cmd_vel';
rate = rateControl(2);
reset(rate)


%% Turtlebot control

tile = tiledlayout(1, 2);
for i = 1:20

    % Velocity command
    setVelocity(turtle, 0.1, 0.2)

    % Camera reading
    nexttile(tile, 1);
    imshow(getColorImage(turtle));
    
    % Scan reading
    nexttile(tile, 2)
    [~, scanMsg] = getLaserScan(turtle);
    plot(scanMsg)

    waitfor(rate);

end
