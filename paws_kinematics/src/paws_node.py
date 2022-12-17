#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose, Point, TransformStamped
import tf2_ros
import tf_conversions
from scipy.spatial.transform import Rotation

class WholeBodyControl:
    def __init__(self, l1, l2, l3, w, l, fx, fy, fz):
        rospy.init_node('paws_node', anonymous=True)

        self.lf_j_pub = rospy.Publisher("/foot/joints/lf", Point, queue_size=1)
        self.rf_j_pub = rospy.Publisher("/foot/joints/rf", Point, queue_size=1)
        self.lh_j_pub = rospy.Publisher("/foot/joints/lh", Point, queue_size=1)
        self.rh_j_pub = rospy.Publisher("/foot/joints/rh", Point, queue_size=1)
        
        self.lf_p_sub = rospy.Subscriber("/foot/pose_rel/lf", Point, self.lf_pose_callback)
        self.rf_p_sub = rospy.Subscriber("/foot/pose_rel/rf", Point, self.rf_pose_callback)
        self.lh_p_sub = rospy.Subscriber("/foot/pose_rel/lh", Point, self.lh_pose_callback)
        self.rh_p_sub = rospy.Subscriber("/foot/pose_rel/rh", Point, self.rh_pose_callback)

        self.body_p_sub = rospy.Subscriber("/body", Pose, self.body_pose_callback)

        self.br = tf2_ros.TransformBroadcaster()

        self.l1 = l1
        self.l2 = l2
        self.l3 = l3

        self.wTb = self.vec_to_mat(0, 0, 0, 0, 0, 0)
        
        self.bTlf = self.vec_to_mat(w/2, -l/2, 0, 90, 0, 0)
        self.bTrf = self.vec_to_mat(-w/2, -l/2, 0, 90, 0, 0)
        self.bTlh = self.vec_to_mat(w/2, l/2, 0, -90, 180, 0)
        self.bTrh = self.vec_to_mat(-w/2, l/2, 0, -90, 180, 0)

        self.lfTlff = self.vec_to_mat(fx, fy, fz, 0, 0, 0)
        self.rfTrff = self.vec_to_mat(-fx, fy, fz, 0, 0, 0)
        self.lhTlhf = self.vec_to_mat(-fx, fy, fz, 0, 0, 0)
        self.rhTrhf = self.vec_to_mat(fx, fy, fz, 0, 0, 0)

        self.bTlf_inv = np.linalg.inv(self.bTlf)
        self.bTrf_inv = np.linalg.inv(self.bTrf)
        self.bTlh_inv = np.linalg.inv(self.bTlh)
        self.bTrh_inv = np.linalg.inv(self.bTrh)

        self.rel_leg_poses = np.zeros((4, 3))

    def publish_tf(self, parent, child, mat):
        msg_q = Rotation.from_matrix(mat[:3, :3]).as_quat()

        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = mat[0, 3]
        msg.transform.translation.y = mat[1, 3]
        msg.transform.translation.z = mat[2, 3]
        msg.transform.rotation.x = msg_q[0]
        msg.transform.rotation.y = msg_q[1]
        msg.transform.rotation.z = msg_q[2]
        msg.transform.rotation.w = msg_q[3]

        self.br.sendTransform(msg)

    def vec_to_mat(self, x, y, z, R, P, Y):
        mat = np.eye(4)
        mat[:3, :3] = Rotation.from_euler("xyz", [R, P, Y], degrees=True).as_matrix()
        mat[0, 3] = x
        mat[1, 3] = y
        mat[2, 3] = z
        return mat

    def vec_to_msg(self, vec):
        msg = Point()
        msg.x = vec[0]
        msg.y = vec[1]
        msg.z = vec[2]
        return msg

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

    def compute_whole_body_control(self, body_pose, leg_poses):
        prevbTb = self.vec_to_mat(body_pose[0], body_pose[1], body_pose[2], body_pose[3], body_pose[4], body_pose[5])
        prevbTb_inv = np.linalg.inv(prevbTb)

        self.lfTlff = self.bTlf_inv @ prevbTb_inv @ self.bTlf @ self.lfTlff
        self.rfTrff = self.bTrf_inv @ prevbTb_inv @ self.bTrf @ self.rfTrff
        self.lhTlhf = self.bTlh_inv @ prevbTb_inv @ self.bTlh @ self.lhTlhf
        self.rhTrhf = self.bTrh_inv @ prevbTb_inv @ self.bTrh @ self.rhTrhf

        self.lfTlff[:3, 3] += leg_poses[0]
        self.rfTrff[:3, 3] += leg_poses[1]
        self.lhTlhf[:3, 3] += leg_poses[2]
        self.rhTrhf[:3, 3] += leg_poses[3]

    def lf_pose_callback(self, data):
        rospy.loginfo("lf callback")
        self.rel_leg_poses[0] = [data.x, data.y, data.z]

    def rf_pose_callback(self, data):
        rospy.loginfo("rf callback")
        self.rel_leg_poses[1] = [data.x, data.y, data.z]

    def lh_pose_callback(self, data):
        rospy.loginfo("lh callback")
        self.rel_leg_poses[2] = [data.x, data.y, data.z]  

    def rh_pose_callback(self, data):
        rospy.loginfo("rh callback")
        self.rel_leg_poses[3] = [data.x, data.y, data.z]  

    def body_pose_callback(self, data):
        rospy.loginfo("body callback")
        eul = Rotation.from_quat([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]).as_euler('xyz', degrees=True)
        body_pose = [data.position.x, data.position.y, data.position.z, eul[0], eul[1], eul[2]]
        
        self.compute_whole_body_control(body_pose, self.rel_leg_poses)

        lf_j = self.compute_ik([self.lfTlff[0, 3], self.lfTlff[1, 3], self.lfTlff[2, 3]], 'lf')
        rf_j = self.compute_ik([self.rfTrff[0, 3], self.rfTrff[1, 3], self.rfTrff[2, 3]], 'rf')
        lh_j = self.compute_ik([self.lhTlhf[0, 3], self.lhTlhf[1, 3], self.lhTlhf[2, 3]], 'lh')
        rh_j = self.compute_ik([self.rhTrhf[0, 3], self.rhTrhf[1, 3], self.rhTrhf[2, 3]], 'rh')

        self.publish_tf("base_link", "base", self.wTb)

        self.publish_tf("base", "lf", self.bTlf)
        self.publish_tf("base", "rf", self.bTrf)
        self.publish_tf("base", "lh", self.bTlh)
        self.publish_tf("base", "rh", self.bTrh)

        self.publish_tf("lf", "lff", self.lfTlff)
        self.publish_tf("rf", "rff", self.rfTrff)
        self.publish_tf("lh", "lhf", self.lhTlhf)
        self.publish_tf("rh", "rhf", self.rhTrhf)


        lf_j_msg = self.vec_to_msg(lf_j)
        rf_j_msg = self.vec_to_msg(rf_j)
        lh_j_msg = self.vec_to_msg(lh_j)
        rh_j_msg = self.vec_to_msg(rh_j)

        self.lf_j_pub.publish(lf_j_msg)
        self.rf_j_pub.publish(rf_j_msg)
        self.lh_j_pub.publish(lh_j_msg)
        self.rh_j_pub.publish(rh_j_msg)

if __name__ == '__main__':
    wbc = WholeBodyControl(0.038, 0.08, 0.08, 0.08, 0.2, 0, -0.12, 0)

    try:
        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        pass