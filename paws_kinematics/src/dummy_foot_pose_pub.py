#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point

if __name__ == '__main__':
    rospy.init_node('dummy_foot_pose_pub_node', anonymous=True)

    lf_p_pub = rospy.Publisher("/foot/pose_rel/lf", Point, queue_size=1)
    rf_p_pub = rospy.Publisher("/foot/pose_rel/rf", Point, queue_size=1)
    lh_p_pub = rospy.Publisher("/foot/pose_rel/lh", Point, queue_size=1)
    rh_p_pub = rospy.Publisher("/foot/pose_rel/rh", Point, queue_size=1)

    rate = rospy.Rate(100)

    msg1 = Point()
    msg1.x = 0.038
    msg1.y = -0.12
    msg1.z = 0.00

    msg2 = Point()
    msg2.x = -0.038
    msg2.y = -0.12
    msg2.z = 0.00

    msg3 = Point()
    msg3.x = -0.038
    msg3.y = -0.12
    msg3.z = 0.00

    msg4 = Point()
    msg4.x = 0.038
    msg4.y = -0.12
    msg4.z = 0.00

    try:
        while not rospy.is_shutdown():
            lf_p_pub.publish(msg1)
            rf_p_pub.publish(msg2)
            lh_p_pub.publish(msg3)
            rh_p_pub.publish(msg4)

            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass