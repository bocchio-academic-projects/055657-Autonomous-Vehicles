#!/usr/bin/python3

import rosbag
from geometry_msgs.msg import Twist
import csv
import rospy

bag = rosbag.Bag('computed.bag', 'w')
with open('cmd_vel_data.csv', newline='') as csvfile:
    reader = csv.reader(csvfile)
    next(reader)  # skip header

    for row in reader:
        time_sec = float(row[0])
        linear_x = float(row[1])
        angular_z = float(row[2])

        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        timestamp = rospy.Time.from_sec(time_sec)
        bag.write('/cmd_vel', msg, t=timestamp)

bag.close()
