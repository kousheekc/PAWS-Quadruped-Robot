#!/usr/bin/env python3
import rospy
from scipy.spatial.transform import Rotation
from paws_kinematics.msg import Phase

if __name__ == '__main__':
    rospy.init_node('dummy_phase_pub_node', anonymous=True)

    phase_pub = rospy.Publisher("/phase", Phase, queue_size=1)

    rate = rospy.Rate(5)

    try:
        while not rospy.is_shutdown():
            msg = Phase()
            phase_pub.publish(msg)
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass