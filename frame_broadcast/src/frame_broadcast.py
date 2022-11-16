#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf_conversions
import numpy as np
from scipy.spatial.transform import Rotation

class FrameBroadcast:
    def __init__(self):
        rospy.init_node('frame_broadcast_node', anonymous=True)

        self.x = 0
        self.y = 0
        self.z = 0
        self.R = 0
        self.P = 0
        self.Y = 10

        self.l = 0.2
        self.w = 0.08
        
        self.lf = [0.038, -0.12, 0]
        self.rf = [-0.038, -0.12, 0]
        self.lh = [-0.038, -0.12, 0]
        self.rh = [0.038, -0.12, 0]

        self.br = tf2_ros.TransformBroadcaster()

    def compute(self):
        wTb = self.vec_to_mat(self.x, self.y, self.z, self.R, self.P, self.Y)
        
        bTlf = self.vec_to_mat(self.w/2, -self.l/2, 0, 90, 0, 0)
        bTrf = self.vec_to_mat(-self.w/2, -self.l/2, 0, 90, 0, 0)
        bTlh = self.vec_to_mat(self.w/2, self.l/2, -90, 180, 0, 0)
        bTrh = self.vec_to_mat(-self.w/2, self.l/2, -90, 180, 0, 0)
        
        lfTlff = self.vec_to_mat(self.lf[0], self.lf[1], self.lf[2], 0, 0, 0)
        rfTrff = self.vec_to_mat(self.rf[0], self.rf[1], self.rf[2], 0, 0, 0)
        lhTlhf = self.vec_to_mat(self.lh[0], self.lh[1], self.lh[2], 0, 0, 0)
        rhTrhf = self.vec_to_mat(self.rh[0], self.rh[1], self.rh[2], 0, 0, 0)

        lfnTlffn = np.dot(np.linalg.inv(bTlf), np.dot(np.linalg.inv(wTb), np.dot(bTlf, lfTlff)))
        rfnTrffn = np.dot(np.linalg.inv(bTrf), np.dot(np.linalg.inv(wTb), np.dot(bTrf, rfTrff)))
        lhnTlhfn = np.dot(np.linalg.inv(bTlh), np.dot(np.linalg.inv(wTb), np.dot(bTlh, lhTlhf)))
        rhnTrhfn = np.dot(np.linalg.inv(bTrh), np.dot(np.linalg.inv(wTb), np.dot(bTrh, rhTrhf)))

        lffn_eul = Rotation.from_matrix(lfnTlffn[:3, :3]).as_euler('xyz', degrees=True)
        rffn_eul = Rotation.from_matrix(rfnTrffn[:3, :3]).as_euler('xyz', degrees=True)
        lhfn_eul = Rotation.from_matrix(lhnTlhfn[:3, :3]).as_euler('xyz', degrees=True)
        rhfn_eul = Rotation.from_matrix(rhnTrhfn[:3, :3]).as_euler('xyz', degrees=True)

        self.publish("base_link", "basen", self.x, self.y, self.z + 0.16, self.R, self.P, self.Y)
        self.publish("basen", "lfn", self.w/2, -self.l/2, 0, 90, 0, 0)
        self.publish("basen", "rfn", -self.w/2, -self.l/2, 0, 90, 0, 0)
        self.publish("basen", "lhn", self.w/2, self.l/2, 0, -90, 180, 0)
        self.publish("basen", "rhn", -self.w/2, self.l/2, 0, -90, 180, 0)
        self.publish("lfn", "lffn", lfnTlffn[0, 3], lfnTlffn[1, 3], lfnTlffn[2, 3], lffn_eul[0], lffn_eul[1], lffn_eul[2])
        self.publish("rfn", "rffn", rfnTrffn[0, 3], rfnTrffn[1, 3], rfnTrffn[2, 3], rffn_eul[0], rffn_eul[1], rffn_eul[2])
        self.publish("lhn", "lhfn", lhnTlhfn[0, 3], lhnTlhfn[1, 3], lhnTlhfn[2, 3], lhfn_eul[0], lhfn_eul[1], lhfn_eul[2])
        self.publish("rhn", "rhfn", rhnTrhfn[0, 3], rhnTrhfn[1, 3], rhnTrhfn[2, 3], rhfn_eul[0], rhfn_eul[1], rhfn_eul[2])

    def vec_to_mat(self, x, y, z, R, P, Y):
        mat = np.eye(4)
        mat[:3, :3] = Rotation.from_euler("xyz", [R, P, Y], degrees=True).as_matrix()
        mat[0, 3] = x
        mat[1, 3] = y
        mat[2, 3] = z
        return mat

    def publish(self, parent, child, x, y, z, R, P, Y):
        R_rad = np.deg2rad(R)
        P_rad = np.deg2rad(P)
        Y_rad = np.deg2rad(Y)

        msg = TransformStamped()
        msg_q = tf_conversions.transformations.quaternion_from_euler(R_rad, P_rad, Y_rad)
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = x
        msg.transform.translation.y = y
        msg.transform.translation.z = z
        msg.transform.rotation.x = msg_q[0]
        msg.transform.rotation.y = msg_q[1]
        msg.transform.rotation.z = msg_q[2]
        msg.transform.rotation.w = msg_q[3]

        self.br.sendTransform(msg)

    def static(self):
        self.publish("base_link", "base", 0, 0, 0.16, 0, 0, 0)
        self.publish("base", "lf", self.w/2, -self.l/2, 0, 90, 0, 0)
        self.publish("base", "rf", -self.w/2, -self.l/2, 0, 90, 0, 0)
        self.publish("base", "lh", self.w/2, self.l/2, 0, -90, 180, 0)
        self.publish("base", "rh", -self.w/2, self.l/2, 0, -90, 180, 0)
        self.publish("lf", "lff", self.lf[0], self.lf[1], self.lf[2], 0, 0, 0)
        self.publish("rf", "rff", self.rf[0], self.rf[1], self.rf[2], 0, 0, 0)
        self.publish("lh", "lhf", self.lh[0], self.lh[1], self.lh[2], 0, 0, 0)
        self.publish("rh", "rhf", self.rh[0], self.rh[1], self.rh[2], 0, 0, 0)


if __name__ == '__main__':
    fb = FrameBroadcast()

    try:
        while not rospy.is_shutdown():
            fb.static()
            fb.compute()

    except rospy.ROSInterruptException:
        pass