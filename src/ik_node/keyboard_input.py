#!/usr/bin/python3

import readchar
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose, Twist
from std_msgs.msg import Bool
from relaxed_ik_ros1.msg import EEPoseGoals, EEVelGoals
import transformations as T
from robot import Robot

path_to_src = rospkg.RosPack().get_path('relaxed_ik_ros1') + '/relaxed_ik_core'

class KeyboardInput:
    def __init__(self):
        deault_setting_file_path = path_to_src + '/configs/settings.yaml'

        setting_file_path = rospy.get_param('setting_file_path')
        if setting_file_path == '':
            setting_file_path = deault_setting_file_path

        self.robot = Robot(setting_file_path)

        self.ee_vel_goals_pub = rospy.Publisher('relaxed_ik/ee_vel_goals', EEVelGoals, queue_size=5)

        self.pos_stride = 0.01
        self.rot_stride = 0.05

        self.seq = 1
        
        self.linear = [0, 0, 0]
        self.angular = [0, 0, 0]

        rospy.Timer(rospy.Duration(0.033), self.timer_callback)

        # Start listening for keyboard inputs in a non-blocking way
        self.listen_for_keys()

    def listen_for_keys(self):
        while not rospy.is_shutdown():
            key = readchar.readchar()  # Block until a key is pressed
            self.on_press(key)

    def on_press(self, key):
        self.linear = [0, 0, 0]
        self.angular = [0, 0, 0]

        if key == 'w':
            self.linear[0] += self.pos_stride
        elif key == 'x':
            self.linear[0] -= self.pos_stride
        elif key == 'a':
            self.linear[1] += self.pos_stride
        elif key == 'd':
            self.linear[1] -= self.pos_stride
        elif key == 'q':
            self.linear[2] += self.pos_stride
        elif key == 'z':
            self.linear[2] -= self.pos_stride
        elif key == '1':
            self.angular[0] += self.rot_stride
        elif key == '2':
            self.angular[0] -= self.rot_stride
        elif key == '3':
            self.angular[1] += self.rot_stride
        elif key == '4':
            self.angular[1] -= self.rot_stride
        elif key == '5':
            self.angular[2] += self.rot_stride
        elif key == '6':
            self.angular[2] -= self.rot_stride
        elif key == 'c':
            rospy.signal_shutdown()

        print("   Linear Vel: {}, Angular Vel: {}".format(self.linear, self.angular))

    def timer_callback(self, event):
        msg = EEVelGoals()

        for i in range(self.robot.num_chain):
            twist = Twist()
            twist.linear.x = self.linear[0]
            twist.linear.y = self.linear[1]
            twist.linear.z = self.linear[2]

            twist.angular.x = self.angular[0]
            twist.angular.y = self.angular[1]
            twist.angular.z = self.angular[2]

            tolerance = Twist()
            tolerance.linear.x = 0.0
            tolerance.linear.y = 0.0
            tolerance.linear.z = 0.0
            tolerance.angular.x = 0.0
            tolerance.angular.y = 0.0
            tolerance.angular.z = 0.0

            msg.ee_vels.append(twist)
            msg.tolerances.append(tolerance)

        self.seq += 1
        self.ee_vel_goals_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('keyboard_input')

    keyboard = KeyboardInput()
    rospy.spin()

