#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Point
import numpy as np
from scipy.spatial.transform import Rotation

class WholeBodyControl:
    def __init__(self, w, l, fx, fy, fz):
        rospy.init_node('whole_body_control_node', anonymous=True)

        self.lf_p_pub = rospy.Publisher("/foot/pose_rel/lf", Point, queue_size=1)
        self.rf_p_pub = rospy.Publisher("/foot/pose_rel/rf", Point, queue_size=1)
        self.lh_p_pub = rospy.Publisher("/foot/pose_rel/lh", Point, queue_size=1)
        self.rh_p_pub = rospy.Publisher("/foot/pose_rel/rh", Point, queue_size=1)
        
        self.body_p_sub = rospy.Subscriber("/body", Pose, self.body_pose_callback)

        self.bTlf = self.vec_to_mat(w/2, -l/2, 0, 90, 0, 0)
        self.bTrf = self.vec_to_mat(-w/2, -l/2, 0, 90, 0, 0)
        self.bTlh = self.vec_to_mat(w/2, l/2, 0, -90, 180, 0)
        self.bTrh = self.vec_to_mat(-w/2, l/2, 0, -90, 180, 0)

        self.bTlf_inv = np.linalg.inv(self.bTlf)
        self.bTrf_inv = np.linalg.inv(self.bTrf)
        self.bTlh_inv = np.linalg.inv(self.bTlh)
        self.bTrh_inv = np.linalg.inv(self.bTrh)
        
        self.lfTlff = self.vec_to_mat(fx, fy, fz, 0, 0, 0)
        self.rfTrff = self.vec_to_mat(-fx, fy, fz, 0, 0, 0)
        self.lhTlhf = self.vec_to_mat(-fx, fy, fz, 0, 0, 0)
        self.rhTrhf = self.vec_to_mat(fx, fy, fz, 0, 0, 0)

    def vec_to_mat(self, x, y, z, R, P, Y):
        mat = np.eye(4)
        mat[:3, :3] = Rotation.from_euler("xyz", [R, P, Y], degrees=True).as_matrix()
        mat[0, 3] = x
        mat[1, 3] = y
        mat[2, 3] = z
        return mat

    def mat_to_msg(self, mat):
        msg = Point()
        msg.x = mat[0, 3]
        msg.y = mat[1, 3]
        msg.z = mat[2, 3]
        return msg

    def body_pose_callback(self, data):
        eul = Rotation.from_quat([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]).as_euler('xyz', degrees=True)
        
        wTb = self.vec_to_mat(data.position.x, data.position.y, data.position.z, eul[0], eul[1], eul[2])
        wTb_inv = np.linalg.inv(wTb)

        lfnTlffn = self.bTlf_inv @ wTb_inv @ self.bTlf @ self.lfTlff
        rfnTrffn = self.bTrf_inv @ wTb_inv @ self.bTrf @ self.rfTrff
        lhnTlhfn = self.bTlh_inv @ wTb_inv @ self.bTlh @ self.lhTlhf
        rhnTrhfn = self.bTrh_inv @ wTb_inv @ self.bTrh @ self.rhTrhf

        lffn = self.mat_to_msg(lfnTlffn)
        rffn = self.mat_to_msg(rfnTrffn)
        lhfn = self.mat_to_msg(lhnTlhfn)
        rhfn = self.mat_to_msg(rhnTrhfn)

        self.lf_p_pub.publish(lffn)
        self.rf_p_pub.publish(rffn)
        self.lh_p_pub.publish(lhfn)
        self.rh_p_pub.publish(rhfn)


if __name__ == '__main__':
    wbc = WholeBodyControl(0.08, 0.2, 0.038, -0.12, 0)

    try:
        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        pass