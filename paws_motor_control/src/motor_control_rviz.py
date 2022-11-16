#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

class MotorController:
    def __init__(self):
        rospy.init_node('motor_control_rviz_node', anonymous=True)

        self.joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)
        
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