#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, TransformStamped
import numpy as np

class InverseKinematics:
    def __init__(self, l1, l2, l3):
        rospy.init_node('inverse_kinematics_node', anonymous=True)

        self.lf_j_pub = rospy.Publisher("/foot/joints/lf", Point, queue_size=1)
        self.rf_j_pub = rospy.Publisher("/foot/joints/rf", Point, queue_size=1)
        self.lh_j_pub = rospy.Publisher("/foot/joints/lh", Point, queue_size=1)
        self.rh_j_pub = rospy.Publisher("/foot/joints/rh", Point, queue_size=1)
        
        self.lf_p_sub = rospy.Subscriber("/foot/pose_rel/lf", Point, self.lf_pose_callback)
        self.rf_p_sub = rospy.Subscriber("/foot/pose_rel/rf", Point, self.rf_pose_callback)
        self.lh_p_sub = rospy.Subscriber("/foot/pose_rel/lh", Point, self.lh_pose_callback)
        self.rh_p_sub = rospy.Subscriber("/foot/pose_rel/rh", Point, self.rh_pose_callback)

        self.l1 = l1
        self.l2 = l2
        self.l3 = l3

    def lf_pose_callback(self, data):
        joint_angles = self.compute_ik([data.x, data.y, data.z], "lf")
        self.publish(joint_angles, "lf")

    def rf_pose_callback(self, data):
        joint_angles = self.compute_ik([data.x, data.y, data.z], "rf")
        self.publish(joint_angles, "rf")

    def lh_pose_callback(self, data):
        joint_angles = self.compute_ik([data.x, data.y, data.z], "lh")
        self.publish(joint_angles, "lh")

    def rh_pose_callback(self, data):
        joint_angles = self.compute_ik([data.x, data.y, data.z], "rh")
        self.publish(joint_angles, "rh")

    def compute_ik(self, pose, leg):
        if (leg == "lh" or leg == "rf"):
            theta1 = -np.arctan2(pose[1], pose[0]) - np.arctan2(np.sqrt(pose[0]**2 + pose[1]**2 - self.l1**2), -self.l1)
            d = (pose[0]**2 + pose[1]**2 - self.l1**2 + pose[2]**2 - self.l2**2 - self.l3**2)/(2 * self.l2 * self.l3)
            theta3 = np.arctan2(-np.sqrt(1 - d**2), d)
            theta2 = np.arctan2(-pose[2], np.sqrt(pose[0]**2 + pose[1]**2 - self.l1**2)) - np.arctan2(self.l3 * np.sin(theta3), self.l2 + self.l3 * np.cos(theta3))

        elif (leg == "lf" or leg == "rh"):
            theta1 = -np.arctan2(pose[1], pose[0]) - np.arctan2(np.sqrt(pose[0]**2 + pose[1]**2 - self.l1**2), self.l1)
            d = (pose[0]**2 + pose[1]**2 - self.l1**2 + pose[2]**2 - self.l2**2 - self.l3**2)/(2 * self.l2 * self.l3)
            theta3 = np.arctan2(np.sqrt(1 - d**2), d)
            theta2 = np.arctan2(pose[2], np.sqrt(pose[0]**2 + pose[1]**2 - self.l1**2)) - np.arctan2(self.l3 * np.sin(theta3), self.l2 + self.l3 * np.cos(theta3))

        return [theta1, theta2, theta3]

    def publish(self, joint_angles, leg):
        joint_angles_msg = Point()

        joint_angles_msg.x = joint_angles[0]
        joint_angles_msg.y = joint_angles[1]
        joint_angles_msg.z = joint_angles[2]

        if leg == "lf":
            self.lf_j_pub.publish(joint_angles_msg)
        elif leg == "rf":
            self.rf_j_pub.publish(joint_angles_msg)
        elif leg == "lh":
            self.lh_j_pub.publish(joint_angles_msg)
        elif leg == "rh":
            self.rh_j_pub.publish(joint_angles_msg)

if __name__ == '__main__':
    inverse_kinematics = InverseKinematics(0.038, 0.08, 0.08)

    try:
        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        pass