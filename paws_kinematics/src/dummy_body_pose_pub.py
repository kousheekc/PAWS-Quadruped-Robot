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
    lf_p_pub = rospy.Publisher("/foot/pose_abs/lf", Point, queue_size=1)
    rf_p_pub = rospy.Publisher("/foot/pose_abs/rf", Point, queue_size=1)
    lh_p_pub = rospy.Publisher("/foot/pose_abs/lh", Point, queue_size=1)
    rh_p_pub = rospy.Publisher("/foot/pose_abs/rh", Point, queue_size=1)
  
    rate = rospy.Rate(100)

    body_x = 0
    body_y = 0
    body_z = 0
    body_R = 0
    body_P = 0
    body_Y = 0

    body_dx = 0.0001
    body_dy = 0.0001
    body_dz = 0
    body_dR = 0
    body_dP = 0
    body_dY = 0

    lf_fx = 0.038
    lf_fy = -0.12
    lf_fz = 0

    lf_fdx = 0
    lf_fdy = 0
    lf_fdz = 0

    rf_fx = 0
    rf_fy = 0
    rf_fz = 0

    lh_fx = 0
    lh_fy = 0
    lh_fz = 0

    rh_fx = 0
    rh_fy = 0
    rh_fz = 0

    try:
        while not rospy.is_shutdown():
            body_msg = generate_pose_msg(body_dx, body_dy, body_dz, body_dR, body_dP, body_dY)

            body_x += body_dx
            body_y += body_dy
            body_z += body_dz

            if (body_x > 0.05):
                lf_fdy = 0.0001
                body_dx = 0
                body_dy = 0

            lf_msg = generate_point_msg(0, 0, 0)
            rf_msg = generate_point_msg(lf_fdx, lf_fdy, lf_fdz)
            lh_msg = generate_point_msg(lh_fx, lh_fy, lh_fz)
            rh_msg = generate_point_msg(rh_fx, rh_fy, rh_fz)

            lf_fx += lf_fdx
            lf_fy += lf_fdy
            lf_fz += lf_fdz

            body_p_pub.publish(body_msg)
            lf_p_pub.publish(lf_msg)
            rf_p_pub.publish(rf_msg)
            lh_p_pub.publish(lh_msg)
            rh_p_pub.publish(rh_msg)

            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass