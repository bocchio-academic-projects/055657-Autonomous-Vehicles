rosshutdown
setenv('ROS_MASTER', '192.168.178.244')
setenv('ROS_IP', char(java.net.InetAddress.getLocalHost().getHostAddress()))
rosinit(getenv('ROS_MASTER'))