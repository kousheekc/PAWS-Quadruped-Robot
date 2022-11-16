#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point

if __name__ == '__main__':
    rospy.init_node('dummy_joint_angle_pub_node', anonymous=True)

    lf_j_pub = rospy.Publisher("/foot/joints/lf", Point, queue_size=1)
    rf_j_pub = rospy.Publisher("/foot/joints/rf", Point, queue_size=1)
    lh_j_pub = rospy.Publisher("/foot/joints/lh", Point, queue_size=1)
    rh_j_pub = rospy.Publisher("/foot/joints/rh", Point, queue_size=1)

    rate = rospy.Rate(100)

    msg = Point()
    msg.x = 0.2
    msg.y = 0.2
    msg.z = 0.2

    try:
        while not rospy.is_shutdown():
            lf_j_pub.publish(msg)
            rf_j_pub.publish(msg)
            lh_j_pub.publish(msg)
            rh_j_pub.publish(msg)

            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass