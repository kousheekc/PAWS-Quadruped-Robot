#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Point

class MotorController:
    def __init__(self):
        rospy.init_node('motor_control_gazebo_node', anonymous=True)

        lf1 = rospy.Publisher("/paws/lf_j1_position_controller/command", Float64, queue_size=1)
        lf2 = rospy.Publisher("/paws/lf_j2_position_controller/command", Float64, queue_size=1)
        lf3 = rospy.Publisher("/paws/lf_j3_position_controller/command", Float64, queue_size=1)
        rf1 = rospy.Publisher("/paws/rf_j1_position_controller/command", Float64, queue_size=1)
        rf2 = rospy.Publisher("/paws/rf_j2_position_controller/command", Float64, queue_size=1)
        rf3 = rospy.Publisher("/paws/rf_j3_position_controller/command", Float64, queue_size=1)
        lh1 = rospy.Publisher("/paws/lh_j1_position_controller/command", Float64, queue_size=1)
        lh2 = rospy.Publisher("/paws/lh_j2_position_controller/command", Float64, queue_size=1)
        lh3 = rospy.Publisher("/paws/lh_j3_position_controller/command", Float64, queue_size=1)
        rh1 = rospy.Publisher("/paws/rh_j1_position_controller/command", Float64, queue_size=1)
        rh2 = rospy.Publisher("/paws/rh_j2_position_controller/command", Float64, queue_size=1)
        rh3 = rospy.Publisher("/paws/rh_j3_position_controller/command", Float64, queue_size=1)
        self.pubs = [lf1, lf2, lf3, rf1, rf2, rf3, lh1, lh2, lh3, rh1, rh2, rh3]

        self.lf_j_sub = rospy.Subscriber("/foot/joints/lf", Point, self.lf_j_callback)
        self.rf_j_sub = rospy.Subscriber("/foot/joints/rf", Point, self.rf_j_callback)
        self.lh_j_sub = rospy.Subscriber("/foot/joints/lh", Point, self.lh_j_callback)
        self.rh_j_sub = rospy.Subscriber("/foot/joints/rh", Point, self.rh_j_callback)

        self.joint_angles = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    def lf_j_callback(self, data):
        self.joint_angles[0] = data.x
        self.joint_angles[1] = data.y
        self.joint_angles[2] = data.z

        self.publish()

    def rf_j_callback(self, data):
        self.joint_angles[3] = data.x
        self.joint_angles[4] = data.y
        self.joint_angles[5] = data.z

        self.publish()

    def lh_j_callback(self, data):
        self.joint_angles[6] = data.x
        self.joint_angles[7] = data.y
        self.joint_angles[8] = data.z

        self.publish()

    def rh_j_callback(self, data):
        self.joint_angles[9] = data.x
        self.joint_angles[10] = data.y
        self.joint_angles[11] = data.z

        self.publish()

    def publish(self):
        msg = Float64()

        for i, pub in enumerate(self.pubs):
            msg.data = self.joint_angles[i]
            pub.publish(msg)

if __name__ == '__main__':
    motor_controller = MotorController()

    try:
        while not rospy.is_shutdown():
            rospy.spin()
            
    except rospy.ROSInterruptException:
        pass