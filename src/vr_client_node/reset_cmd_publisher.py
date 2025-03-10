#!/usr/bin/env python3
import rospy
from hdas_msg.msg import motor_control
from std_msgs.msg import Empty
import time
import subprocess

def publish_motion_control():
    rospy.init_node('motion_control_publisher', anonymous=True)
    pub_right = rospy.Publisher('/motion_control/control_arm_right', motor_control, queue_size=10)
    pub_left = rospy.Publisher('/motion_control/control_arm_left', motor_control, queue_size=10)
    pub_complete = rospy.Publisher('/relaxed_ik/completed_signal', Empty, queue_size=10)
    
    rospy.sleep(5.0)
    
    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.get_time()
    
    while not rospy.is_shutdown():
        current_time = rospy.get_time()
        if current_time - start_time >= 8:
            break  
        
        msg_right = motor_control()
        msg_right.header.seq = 0
        msg_right.header.stamp = rospy.Time.now()
        # msg_right.header.stamp.nsecs = 0
        msg_right.header.frame_id = ''
        msg_right.name = ''
        msg_right.p_des = [-1.57, 2.75, -2.3, 0.00, 0.00, 0.00]
        msg_right.v_des = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
        msg_right.kp = [0]
        msg_right.kd = [0]
        msg_right.t_ff = [0.8, 0.8, 0.8, 0.8, 0.8, 0.8]
        msg_right.mode = 0
        
        msg_left = motor_control()
        msg_left.header.seq = 0
        msg_left.header.stamp = rospy.Time.now()
        # msg_left.header.stamp.nsecs = 0
        msg_left.header.frame_id = ''
        msg_left.name = ''
        msg_left.p_des = [1.57, 2.75, -2.3, 0.00, 0.00, 0.00]
        msg_left.v_des = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
        msg_left.kp = [0]
        msg_left.kd = [0]
        msg_left.t_ff = [0.8, 0.8, 0.8, 0.8, 0.8, 0.8]
        msg_left.mode = 0

        rospy.loginfo("Publishing motion control command to control arm...")

        pub_right.publish(msg_right)
        pub_left.publish(msg_left)
        rate.sleep()

    rospy.loginfo("Publishing completion signal...")
    pub_complete.publish(Empty())
    

if __name__ == '__main__':
    try:
        publish_motion_control()
    except Exception:
        pass
