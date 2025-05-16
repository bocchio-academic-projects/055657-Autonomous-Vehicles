rosshutdown
setenv('ROS_MASTER', '192.168.178.244')
setenv('ROS_IP', char(java.net.InetAddress.getLocalHost().getHostAddress()))
rosinit(getenv('ROS_MASTER'))

wsl = @(command) system(['start /b wsl bash -c "source /opt/ros/noetic/setup.bash && ' command '"']);