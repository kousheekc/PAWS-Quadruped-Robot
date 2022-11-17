#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped, Point
import tf2_ros
import tf_conversions
import numpy as np
from scipy.spatial.transform import Rotation

class FrameBroadcast:
    def __init__(self):
        rospy.init_node('frame_broadcast_node', anonymous=True)

        self.lf_p_pub = rospy.Publisher("/foot/pose_rel/lf", Point, queue_size=1)
        self.rf_p_pub = rospy.Publisher("/foot/pose_rel/rf", Point, queue_size=1)
        self.lh_p_pub = rospy.Publisher("/foot/pose_rel/lh", Point, queue_size=1)
        self.rh_p_pub = rospy.Publisher("/foot/pose_rel/rh", Point, queue_size=1)

        self.x = 0
        self.y = 0
        self.z = 0
        self.R = 0
        self.P = 0
        self.Y = 20

        self.l = 0.2
        self.w = 0.08
        
        self.lf = [0.038, -0.12, 0]
        self.rf = [-0.038, -0.12, 0]
        self.lh = [-0.038, -0.12, 0]
        self.rh = [0.038, -0.12, 0]

        self.br = tf2_ros.TransformBroadcaster()
        self.rate = rospy.Rate(100)
       
    def vec_to_mat(self, x, y, z, R, P, Y):
        mat = np.eye(4)
        mat[:3, :3] = Rotation.from_euler("xyz", [R, P, Y], degrees=True).as_matrix()
        mat[0, 3] = x
        mat[1, 3] = y
        mat[2, 3] = z
        return mat

    # def vec_to_mat(self, x, y, z, R, P, Y):
    #     R_rad = np.deg2rad(R)
    #     P_rad = np.deg2rad(P)
    #     Y_rad = np.deg2rad(Y)
    #     mat = tf_conversions.transformations.euler_matrix(R_rad, P_rad, Y_rad)
    #     mat[0, 3] = x
    #     mat[1, 3] = y
    #     mat[2, 3] = z
    #     return mat

    def mat_to_vec(self, mat):
        eul = Rotation.from_matrix(mat[:3, :3]).as_euler('xyz', degrees=True)
        return [mat[0, 3], mat[1, 3], mat[2, 3], eul[0], eul[1], eul[2]]

    # def mat_to_vec(self, mat):
    #     eul = tf_conversions.transformations.euler_from_matrix(mat)
    #     return [mat[0, 3], mat[1, 3], mat[2, 3], np.rad2deg(eul[0]), np.rad2deg(eul[1]), np.rad2deg(eul[2])]

    def vec_to_msg(self, vec):
        msg = Point()
        msg.x = vec[0]
        msg.y = vec[1]
        msg.z = vec[2]
        return msg

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


    def move(self):
        wTb = self.vec_to_mat(self.x, self.y, self.z, self.R, self.P, self.Y)

        bTlf = self.vec_to_mat(self.w/2, -self.l/2, 0, 90, 0, 0)
        bTrf = self.vec_to_mat(-self.w/2, -self.l/2, 0, 90, 0, 0)
        bTlh = self.vec_to_mat(self.w/2, self.l/2, 0, -90, 180, 0)
        bTrh = self.vec_to_mat(-self.w/2, self.l/2, 0, -90, 180, 0)
        
        lfTlff = self.vec_to_mat(self.lf[0], self.lf[1], self.lf[2], 0, 0, 0)
        rfTrff = self.vec_to_mat(self.rf[0], self.rf[1], self.rf[2], 0, 0, 0)
        lhTlhf = self.vec_to_mat(self.lh[0], self.lh[1], self.lh[2], 0, 0, 0)
        rhTrhf = self.vec_to_mat(self.rh[0], self.rh[1], self.rh[2], 0, 0, 0)

        lfnTlffn = np.linalg.inv(bTlf) @ np.linalg.inv(wTb) @ bTlf @ lfTlff
        rfnTrffn = np.linalg.inv(bTrf) @ np.linalg.inv(wTb) @ bTrf @ rfTrff
        lhnTlhfn = np.linalg.inv(bTlh) @ np.linalg.inv(wTb) @ bTlh @ lhTlhf
        rhnTrhfn = np.linalg.inv(bTrh) @ np.linalg.inv(wTb) @ bTrh @ rhTrhf

        lffn = self.mat_to_vec(lfnTlffn)
        rffn = self.mat_to_vec(rfnTrffn)
        lhfn = self.mat_to_vec(lhnTlhfn)
        rhfn = self.mat_to_vec(rhnTrhfn)

        self.publish("base_link", "basen", self.x, self.y, self.z + 0.16, self.R, self.P, self.Y)
        
        self.publish("basen", "lfn", self.w/2, -self.l/2, 0, 90, 0, 0)
        self.publish("basen", "rfn", -self.w/2, -self.l/2, 0, 90, 0, 0)
        self.publish("basen", "lhn", self.w/2, self.l/2, 0, -90, 180, 0)
        self.publish("basen", "rhn", -self.w/2, self.l/2, 0, -90, 180, 0)
        
        self.publish("lfn", "lffn", lffn[0], lffn[1], lffn[2], lffn[3], lffn[4], lffn[5])
        self.publish("rfn", "rffn", rffn[0], rffn[1], rffn[2], rffn[3], rffn[4], rffn[5])
        self.publish("lhn", "lhfn", lhfn[0], lhfn[1], lhfn[2], lhfn[3], lhfn[4], lhfn[5])
        self.publish("rhn", "rhfn", rhfn[0], rhfn[1], rhfn[2], rhfn[3], rhfn[4], rhfn[5])

        lf_msg = self.vec_to_msg(lffn)
        rf_msg = self.vec_to_msg(rffn)
        lh_msg = self.vec_to_msg(lhfn)
        rh_msg = self.vec_to_msg(rhfn)

        self.lf_p_pub.publish(lf_msg)
        self.rf_p_pub.publish(rf_msg)
        self.lh_p_pub.publish(lh_msg)
        self.rh_p_pub.publish(rh_msg)

        self.rate.sleep()

if __name__ == '__main__':
    fb = FrameBroadcast()

    try:
        while not rospy.is_shutdown():
            fb.static()
            fb.move()

    except rospy.ROSInterruptException:
        pass