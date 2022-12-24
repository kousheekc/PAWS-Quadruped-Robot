#!/usr/bin/env python3
import rospy
import numpy as np
from paws_description.msg import Phase, Joints
from geometry_msgs.msg import TransformStamped
import tf2_ros
from scipy.spatial.transform import Rotation

class WholeBodyControl:
    def __init__(self, l1, l2, l3, w, l, fx, fy, fz):
        rospy.init_node('whole_body_control_node', anonymous=True)

        self.joints_pub = rospy.Publisher("/joints", Joints, queue_size=1)
        
        self.phase_sub = rospy.Subscriber("/phase", Phase, self.phase_callback)
        
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

    def pose_to_msg(self, lf, rf, lh, rh):
        msg = Joints()

        msg.lf.x = lf[0]
        msg.lf.y = lf[1]
        msg.lf.z = lf[2]
        
        msg.rf.x = rf[0]
        msg.rf.y = rf[1]
        msg.rf.z = rf[2]
        
        msg.lh.x = lh[0]
        msg.lh.y = lh[1]
        msg.lh.z = lh[2]
        
        msg.rh.x = rh[0]
        msg.rh.y = rh[1]
        msg.rh.z = rh[2]

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

    def compute_whole_body_control(self, body_pose, lf_pose, rf_pose, lh_pose, rh_pose):
        prevbTb = self.vec_to_mat(body_pose[0], body_pose[1], body_pose[2], body_pose[3], body_pose[4], body_pose[5])
        prevbTb_inv = np.linalg.inv(prevbTb)

        self.lfTlff = self.bTlf_inv @ prevbTb_inv @ self.bTlf @ self.lfTlff
        self.rfTrff = self.bTrf_inv @ prevbTb_inv @ self.bTrf @ self.rfTrff
        self.lhTlhf = self.bTlh_inv @ prevbTb_inv @ self.bTlh @ self.lhTlhf
        self.rhTrhf = self.bTrh_inv @ prevbTb_inv @ self.bTrh @ self.rhTrhf

        self.lfTlff[:3, 3] += lf_pose
        self.rfTrff[:3, 3] += rf_pose
        self.lhTlhf[:3, 3] += lh_pose
        self.rhTrhf[:3, 3] += rh_pose

        return [self.lfTlff, self.rfTrff, self.lhTlhf, self.rhTrhf]

    def phase_callback(self, data):
        rospy.loginfo("phase callback")
        eul = Rotation.from_quat([data.body.orientation.x, data.body.orientation.y, data.body.orientation.z, data.body.orientation.w]).as_euler('xyz', degrees=True)
        body_pose = [data.body.position.x, data.body.position.y, data.body.position.z, eul[0], eul[1], eul[2]]

        lf_pose = [data.lf.x, data.lf.y, data.lf.z]
        rf_pose = [data.rf.x, data.rf.y, data.rf.z]
        lh_pose = [data.lh.x, data.lh.y, data.lh.z]
        rh_pose = [data.rh.x, data.rh.y, data.rh.z]
        
        leg_poses = self.compute_whole_body_control(body_pose, lf_pose, rf_pose, lh_pose, rh_pose)

        lf_j = self.compute_ik(leg_poses[0][:3, 3].T, 'lf')
        rf_j = self.compute_ik(leg_poses[1][:3, 3].T, 'rf')
        lh_j = self.compute_ik(leg_poses[2][:3, 3].T, 'lh')
        rh_j = self.compute_ik(leg_poses[3][:3, 3].T, 'rh')

        self.publish_tf("base_link", "base", self.wTb)

        self.publish_tf("base", "lf", self.bTlf)
        self.publish_tf("base", "rf", self.bTrf)
        self.publish_tf("base", "lh", self.bTlh)
        self.publish_tf("base", "rh", self.bTrh)

        self.publish_tf("lf", "lff", self.lfTlff)
        self.publish_tf("rf", "rff", self.rfTrff)
        self.publish_tf("lh", "lhf", self.lhTlhf)
        self.publish_tf("rh", "rhf", self.rhTrhf)

        joints_msg = self.pose_to_msg(lf_j, rf_j, lh_j, rh_j)
        self.joints_pub.publish(joints_msg)

if __name__ == '__main__':
    wbc = WholeBodyControl(0.038, 0.08, 0.08, 0.08, 0.2, 0.038, -0.12, 0)

    try:
        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        pass