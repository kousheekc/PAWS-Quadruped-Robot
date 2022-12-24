#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from paws_description.msg import Joints

class MotorController:
    def __init__(self):
        rospy.init_node('motor_control_rviz_node', anonymous=True)

        self.joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)

        self.joints_sub = rospy.Subscriber("/joints", Joints, self.joints_callback)

    def joints_callback(self, data):
        joints = [data.lf.x, data.lf.y, data.lf.z, data.rf.x, data.rf.y, data.rf.z, data.lh.x, data.lh.y, data.lh.z, data.rh.x, data.rh.y, data.rh.z]
        
        msg = JointState()

        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.name = ['lf_j1', 'lf_j2', 'lf_j3', 'rf_j1', 'rf_j2', 'rf_j3', 'lh_j1', 'lh_j2', 'lh_j3', 'rh_j1', 'rh_j2', 'rh_j3']
        msg.position = joints
        msg.velocity = []
        msg.effort = []
        
        self.joint_pub.publish(msg)

    def publish(self):
        msg = JointState()
        
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.name = ['lf_j1', 'lf_j2', 'lf_j3', 'rf_j1', 'rf_j2', 'rf_j3', 'lh_j1', 'lh_j2', 'lh_j3', 'rh_j1', 'rh_j2', 'rh_j3']
        msg.position = self.joint_angles
        msg.velocity = []
        msg.effort = []
        
        self.joint_pub.publish(msg)

if __name__ == '__main__':
    motor_controller = MotorController()

    try:
        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        pass