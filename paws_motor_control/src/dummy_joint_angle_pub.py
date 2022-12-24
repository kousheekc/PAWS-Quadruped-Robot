#!/usr/bin/env python3
import rospy
from paws_description.msg import Joints

if __name__ == '__main__':
    rospy.init_node('dummy_joint_angle_pub_node', anonymous=True)

    joints_pub = rospy.Publisher("/joints", Joints, queue_size=1)

    rate = rospy.Rate(100)

    msg = Joints()
    msg.lf.x = 0.2
    msg.lf.y = 0.2
    msg.lf.z = 0.2
    
    msg.rf.x = 0.2
    msg.rf.y = 0.2
    msg.rf.z = 0.2
    
    msg.lh.x = 0.2
    msg.lh.y = 0.2
    msg.lh.z = 0.2
    
    msg.rh.x = 0.2
    msg.rh.y = 0.2
    msg.rh.z = 0.2

    try:
        while not rospy.is_shutdown():
            joints_pub.publish(msg)
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass