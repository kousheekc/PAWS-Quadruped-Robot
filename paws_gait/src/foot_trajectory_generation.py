import rospy
import numpy as np
from scipy.spatial.transform import Rotation
from paws_description.msg import Phase
import matplotlib.pyplot as plt

class FootTrajectoryGeneration:
    def __init__(self, phase_len, swing_start, swing_len, step_len, step_height, step_angle):
        rospy.init_node('walk_node', anonymous=True)
        self.phase_pub = rospy.Publisher("/phase", Phase, queue_size=1)

        self.phase_len = phase_len
        self.swing_start = swing_start
        self.swing_len = swing_len
        self.step_len = step_len
        self.step_height = step_height
        self.step_angles = step_angle
        
    def bezier(self):
        pass

    def get_stance(self, time):
        swing_stance_state = [0, 0, 0, 0]
        time = time % self.phase_len
        for i in range(4):
            if self.swing_start[i]/100*self.phase_len <= time <= self.swing_start[i]/100*self.phase_len + self.swing_len[i]/100*self.phase_len:
                swing_stance_state[i] = 1
            else:
                swing_stance_state[i] = 0

        return swing_stance_state

    def get_foot_displacements(self, time, swing_stance_state):
        foot_displacements = np.zeros((4, 4))
        time = time % self.phase_len
        for i in range(4):
            if swing_stance_state[i] == 1:
                if self.swing_start[i]/100*self.phase_len < time < self.swing_start[i]/100*self.phase_len + (self.swing_len[i]/100*self.phase_len)/2:
                    foot_displacements[i][1] = 0.0001
                elif self.swing_start[i]/100*self.phase_len + (self.swing_len[i]/100*self.phase_len)/2 <= time <= self.swing_start[i]/100*self.phase_len + self.swing_len[i]/100*self.phase_len:
                    foot_displacements[i][1] = -0.0001
                if i == 0 or i == 1:
                    foot_displacements[i][2] = 0.00028
                else:
                    foot_displacements[i][2] = -0.00028
            elif swing_stance_state[i] == 0:
                if i == 0 or i == 1:
                    foot_displacements[i][2] = -0.00004
                else:
                    foot_displacements[i][2] = 0.00004

        return foot_displacements

    def generate_phase_msg(self, body, legs):
        msg = Phase()
        quat = Rotation.from_euler("xyz", [body[3], body[4], body[5]], degrees=True).as_quat()
        
        msg.body.position.x = body[0]
        msg.body.position.y = body[1]
        msg.body.position.z = body[2]
        msg.body.orientation.x = quat[0]
        msg.body.orientation.y = quat[1]
        msg.body.orientation.z = quat[2]
        msg.body.orientation.w = quat[3]

        msg.lf.x = legs[0][0]
        msg.lf.y = legs[0][1]
        msg.lf.z = legs[0][2]

        msg.rf.x = legs[1][0]
        msg.rf.y = legs[1][1]
        msg.rf.z = legs[1][2]

        msg.lh.x = legs[2][0]
        msg.lh.y = legs[2][1]
        msg.lh.z = legs[2][2]

        msg.rh.x = legs[3][0]
        msg.rh.y = legs[3][1]
        msg.rh.z = legs[3][2]

        return msg

    def publish(self, time):
        swing_stance_state = self.get_stance(time)
        foot_displacements = self.get_foot_displacements(time, swing_stance_state)
        foot_msg = self.generate_phase_msg([0, 0, 0, 0, 0, 0], foot_displacements)
        self.phase_pub.publish(foot_msg)


if __name__ == '__main__':
    ftg = FootTrajectoryGeneration(2, [62.5, 12.5, 37.5, 87.5], [12.5, 12.5, 12.5, 12.5], 0.2, 0.2, 0)

    rate = rospy.Rate(1000)
    start_time = rospy.get_time()

    init_time = 10

    try:
        while not rospy.is_shutdown():
            time = rospy.get_time() - start_time
            if time > init_time:
                rospy.loginfo(ftg.publish(time))
            rate.sleep()
        

    except rospy.ROSInterruptException:
        pass