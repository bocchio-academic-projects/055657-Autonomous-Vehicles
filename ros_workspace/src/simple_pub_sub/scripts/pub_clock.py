#!/usr/bin/python3

import rospy
from std_msgs.msg import Float32

if __name__ == "__main__":
    pub = rospy.Publisher("clock", Float32, queue_size=10)
    rospy.init_node("pub_clock")
    rate = rospy.Rate(1)
    start = rospy.get_time()

    while not rospy.is_shutdown():
        msg = rospy.get_time() - start
        pub.publish(msg)
        rospy.logdebug(msg)
        rate.sleep()
