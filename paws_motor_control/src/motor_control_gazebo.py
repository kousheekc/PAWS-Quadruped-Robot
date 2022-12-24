#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from paws_description.msg import Joints

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

        self.joints_sub = rospy.Subscriber("/joints", Joints, self.joints_callback)

    def joints_callback(self, data):
        joints = [data.lf.x, data.lf.y, data.lf.z, data.rf.x, data.rf.y, data.rf.z, data.lh.x, data.lh.y, data.lh.z, data.rh.x, data.rh.y, data.rh.z]
        
        msg = Float64()

        for i in range(len(joints)):
            msg.data = joints[i]
            self.pubs[i].publish(msg)

if __name__ == '__main__':
    motor_controller = MotorController()

    try:
        while not rospy.is_shutdown():
            rospy.spin()
            
    except rospy.ROSInterruptException:
        pass