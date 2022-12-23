#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Point
from scipy.spatial.transform import Rotation

def generate_pose_msg(x, y, z, R, P, Y):
    msg = Pose()
    quat = Rotation.from_euler("xyz", [R, P, Y], degrees=True).as_quat()
    msg.position.x = x
    msg.position.y = y
    msg.position.z = z
    msg.orientation.x = quat[0]
    msg.orientation.y = quat[1]
    msg.orientation.z = quat[2]
    msg.orientation.w = quat[3]
    return msg

def generate_point_msg(x, y, z):
    msg = Point()
    msg.x = x
    msg.y = y
    msg.z = z
    return msg

if __name__ == '__main__':
    rospy.init_node('dummy_body_pose_pub_node', anonymous=True)

    body_p_pub = rospy.Publisher("/body", Pose, queue_size=1)
    lf_p_pub = rospy.Publisher("/foot/pose_rel/lf", Point, queue_size=1)
    rf_p_pub = rospy.Publisher("/foot/pose_rel/rf", Point, queue_size=1)
    lh_p_pub = rospy.Publisher("/foot/pose_rel/lh", Point, queue_size=1)
    rh_p_pub = rospy.Publisher("/foot/pose_rel/rh", Point, queue_size=1)
  
    rate = rospy.Rate(5)

    counter = 0

    try:
        while not rospy.is_shutdown():
            if counter <= 10:
                rospy.loginfo("published!!")
                body_msg = generate_pose_msg(0, 0.002, 0, 0, 0, 0)

                lf_msg = generate_point_msg(0, 0, 0)
                rf_msg = generate_point_msg(0, 0.002, 0)
                lh_msg = generate_point_msg(0, 0, 0)
                rh_msg = generate_point_msg(0, 0, 0)

                body_p_pub.publish(body_msg)
                lf_p_pub.publish(lf_msg)
                rf_p_pub.publish(rf_msg)
                lh_p_pub.publish(lh_msg)
                rh_p_pub.publish(rh_msg)
                counter += 1
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass