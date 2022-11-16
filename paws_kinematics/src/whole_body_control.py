#!/usr/bin/env python3
# import rospy
# from geometry_msgs.msg import TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation

class WholeBodyControl:
    def __init__(self):
        # rospy.init_node('whole_body_control_node', anonymous=True)
        self.x = 1
        self.y = 2
        self.z = 3
        self.R = 0
        self.P = 0
        self.Y = 60

        self.l = 0.2
        self.w = 0.08
        
        self.lf = [0.038, -0.12, 0]
        self.rf = [-0.038, -0.12, 0]
        self.lh = [-0.038, -0.12, 0]
        self.rh = [0.038, -0.12, 0]

    def compute(self):
        wTb = np.eye(4)
        wTb[:3, :3] = Rotation.from_euler("zyx", [self.R, self.P, self.Y], degrees=True).as_matrix()
        wTb[0, 3] = self.x
        wTb[1, 3] = self.y
        wTb[2, 3] = self.z

        bTlf = np.eye(4)
        bTlf[:3, :3] = Rotation.from_euler("zyx", [90, 0, 0], degrees=True).as_matrix()
        bTlf[0, 3] = self.w/2
        bTlf[1, 3] = -self.l/2
        bTlf[2, 3] = 0

        lfTlfl = np.eye(4)
        lfTlfl[:3, :3] = Rotation.from_euler("zyx", [0, 0, 0], degrees=True).as_matrix()
        lfTlfl[0, 3] = self.lf[0]
        lfTlfl[1, 3] = self.lf[1]
        lfTlfl[2, 3] = self.lf[2]

        print(wTb)

if __name__ == '__main__':
    wbc = WholeBodyControl()
    wbc.compute()

    # try:
    #     while not rospy.is_shutdown():
    #         fb.publish()
    #         fb.compute()

    # except rospy.ROSInterruptException:
    #     pass