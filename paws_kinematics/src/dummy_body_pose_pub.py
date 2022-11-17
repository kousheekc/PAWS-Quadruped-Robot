#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation

if __name__ == '__main__':
    rospy.init_node('dummy_body_pose_pub_node', anonymous=True)

    body_p_pub = rospy.Publisher("/body", Pose, queue_size=1)
  
    rate = rospy.Rate(100)

    msg = Pose()
    msg.position.x = 0
    msg.position.y = 0
    msg.position.z = 0

    x = 0
    dx = 0.1

    try:
        while not rospy.is_shutdown():
            if ((x > 10) or (x < -10)):
                dx = dx * (-1)
            x += dx

            quat = Rotation.from_euler("xyz", [x, x, x], degrees=True).as_quat()
            msg.orientation.x = quat[0]
            msg.orientation.y = quat[1]
            msg.orientation.z = quat[2]
            msg.orientation.w = quat[3]
            body_p_pub.publish(msg)
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass