#!/usr/bin/env python3
import rospy
from scipy.spatial.transform import Rotation
from paws_description.msg import Phase

def generate_phase_msg(body, lf, rf, lh, rh):
    msg = Phase()
    quat = Rotation.from_euler("xyz", [body[3], body[4], body[5]], degrees=True).as_quat()
    
    msg.body.position.x = body[0]
    msg.body.position.y = body[1]
    msg.body.position.z = body[2]
    msg.body.orientation.x = quat[0]
    msg.body.orientation.y = quat[1]
    msg.body.orientation.z = quat[2]
    msg.body.orientation.w = quat[3]

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

if __name__ == '__main__':
    rospy.init_node('dummy_phase_pub_node', anonymous=True)

    phase_pub = rospy.Publisher("/phase", Phase, queue_size=100)

    rate = rospy.Rate(10)

    looped_once = False
    start_time = rospy.get_time()
    init_duration = 10

    try:
        while not rospy.is_shutdown():
            time = rospy.get_time() - start_time
            if time < init_duration:
                rospy.loginfo("looped, not moving")
                msg = generate_phase_msg([0, 0, 0, 0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0])
                phase_pub.publish(msg)
            elif time >= init_duration and looped_once == False:
                rospy.loginfo("looped, moving")
                msg = generate_phase_msg([0, 0.02, 0, 0, 0, 0], [0, 0.02, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0])
                phase_pub.publish(msg)
                looped_once = True
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass