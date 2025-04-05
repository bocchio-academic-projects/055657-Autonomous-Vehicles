#!/usr/bin/python3

import rospy
from std_msgs.msg import String

if __name__ == "__main__":
    pub = rospy.Publisher("hello_world", String, queue_size=10)
    rospy.init_node("pub_hello_world")
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = f"hello world {rospy.get_time()}"
        pub.publish(msg)
        rospy.logdebug(msg)
        rate.sleep()
