clc
clear variables
close all

run('connect_to_WSL.m')


%% Test connection

try

    ros_topic_list = rostopic('list');
    msg_hello_world = rostopic('echo', '/hello_world');
    disp(msg_hello_world.Data);

catch ME

    fprintf(2, [ME.message '\n']);
    return

end