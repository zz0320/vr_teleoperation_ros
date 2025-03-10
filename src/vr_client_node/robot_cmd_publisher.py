#!/usr/bin/env python3

import rospy
import time
import numpy as np
from pyquaternion import Quaternion
from vr_teleoperation_msg.msg import VrPose
from vr_ros_adapter import ManipVrRosAdapter
from std_msgs.msg import Empty


ARM_LENGTH = 0.7608  # hardcoded, meters
HUMAN_LENGTH = 0.61  # hardcoded, meters
LENGTH_RATIO = ARM_LENGTH / HUMAN_LENGTH
CHASSIS_LINEAR_VEL_RATIO = 0.2
CHASSIS_ANGULAR_VEL_RATIO = 0.4

class VrPoseProcessor:

    def __init__(self, left_flag: bool, ros_adapter:ManipVrRosAdapter, pub_breakpoint: bool):
        self.left_flag = left_flag
        self.sub_topic_name = "vr_pose"
        self.arm_name = "left_arm" if self.left_flag else "right_arm"

        self.ros_adapter = ros_adapter

        self.init_vr_call = True
        self.gripper_close_status = False
        self.gripper_last_update_time = None
        self.disconnect_status = False
        self.disconnect_last_update_time = None

        self.unity_to_robot_mat = np.array(
            [[0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0]]
        )
        self.unity_to_robot_mat_inv = np.linalg.inv(self.unity_to_robot_mat)
        self.callback_times = 0

        self.last_vrstick_pos_robot_coord = None  # vr stick position in last time step, in robot coordination
        self.init_vrstick_pos_robot_coord = None  # initial vr stick position
        
        self.last_vrstick_quat_unity_coord = None
        self.init_vrstick_quat_unity_coord = None

        self.last_target_position = None
        self.last_target_quaternion = None
        self.init_mode_zero = True
        self.last_vr_msg_time = None
        self.pub_breakpoint = pub_breakpoint
        self.vr_pose_subscriber = None
        self.last_mode = 0
    
    def start_listen(self):
        self.ros_adapter.listen_init_pose_from_ros()
        init_pos, init_quat_xyzw = self.ros_adapter.get_init_pose()
        if self.left_flag:
            ...
            init_pos=np.array([0.5,  0.238891402, -0.10208433])
            init_quat_xyzw=np.array([0, 0, 0,1])
        else:
            ...
            init_pos=np.array( [ 0.5, -0.230083289, -0.103303090] )
            init_quat_xyzw=np.array([0, 0, 0,1])
        
        self.init_position = init_pos
        self.init_quaternion = Quaternion(init_quat_xyzw[3], init_quat_xyzw[0], init_quat_xyzw[1], init_quat_xyzw[2])
        self.vr_pose_subscriber = rospy.Subscriber(self.sub_topic_name, VrPose, self.callback)
        print(f"{self.arm_name} starts listening")
    
    def stop_listen(self):
        self.vr_pose_subscriber.unregister()
        self.vr_pose_subscriber = None
        print(f"{self.arm_name} stops listening")
    
    def process_first_vr_call(self, stick_pos, stick_quat):
        self.last_vrstick_pos_robot_coord = stick_pos 
        self.init_vrstick_pos_robot_coord = stick_pos

        self.last_vrstick_quat_unity_coord = stick_quat
        self.init_vrstick_quat_unity_coord = stick_quat
        self.gripper_last_update_time = time.time()
        self.disconnect_last_update_time = time.time()

        self.init_vr_call = False
    
    def parse_torso(self, cmd_left, cmd_right, center, left, right):
        if (not cmd_right and not cmd_left):
            result_direction = center
        elif (not cmd_right and cmd_left):
            result_direction = left
        elif (cmd_right and not cmd_left):
            result_direction = right
        else:
            result_direction = center
        return result_direction
        
    def init_mode_check(self, control_mode_vr):
        if (self.init_mode_zero and not (control_mode_vr == 0)):
            control_mode = 0
            rospy.logwarn("THE INITIAL MODE IS NOT ZERO! PLEASE PRESS 2 STICKS TO ZERO IT!")
        elif (self.init_mode_zero and control_mode_vr == 0):
            self.init_mode_zero = False
            control_mode = control_mode_vr
        else:
            control_mode = control_mode_vr
        return control_mode
    
    def parse_vrpose_data(self, msg_data: VrPose):
        if self.last_vr_msg_time is None:
            self.last_vr_msg_time = time.time()
        else:
            current_time = time.time()
            spent_time = current_time - self.last_vr_msg_time
            self.last_vr_msg_time = current_time
            if spent_time > 0.1:
                print("abnormal vr interval time: ", spent_time)
        
        control_mode_vr = msg_data.control_mode
        control_mode = self.init_mode_check(control_mode_vr)
        breakpoint = False
        exception = False
        
        if control_mode == 0:
            if self.left_flag:
                stick_pos = np.array(msg_data.left_position) - np.array(msg_data.head_position)
                stick_quat = Quaternion(msg_data.left_quaternion[3], msg_data.left_quaternion[0], msg_data.left_quaternion[1], msg_data.left_quaternion[2])
                gripper_pressed = msg_data.left_gripper_close
                disconnect_flag = msg_data.left_disconnect_pressed
            else:
                stick_pos = np.array(msg_data.right_position) - np.array(msg_data.head_position)
                stick_quat = Quaternion(msg_data.right_quaternion[3], msg_data.right_quaternion[0], msg_data.right_quaternion[1], msg_data.right_quaternion[2])
                gripper_pressed = msg_data.right_gripper_close
                disconnect_flag = msg_data.right_disconnect_pressed
            waist_direction = 0
            pitch_direction = 0
            vertical_direction = 0
            chassis_x_vel = 0
            chassis_y_vel = 0
            chassis_angular_vel = 0

        elif control_mode == 1:
            stick_pos = [] 
            stick_quat = [] 
            gripper_pressed = 0
            disconnect_flag = 0
            chassis_x_vel = 0
            chassis_y_vel = 0
            chassis_angular_vel = 0
            waist_direction = self.parse_torso(msg_data.waist_left, msg_data.waist_right, 0, 1, 2)
            pitch_direction = self.parse_torso(msg_data.tilt_forward, msg_data.tilt_backward, 0, 2, 1)
            vertical_direction = self.parse_torso(msg_data.torso_up, msg_data.torso_down, 0, 0, 0)
            print(f"control mode vr:{control_mode_vr}, control mode: control mode: {control_mode}")
        elif control_mode == 2:
            stick_pos = []
            stick_quat = []
            gripper_pressed = 0
            disconnect_flag = 0
            waist_direction = 0
            pitch_direction = 0
            vertical_direction = 0
            chassis_x_vel = CHASSIS_LINEAR_VEL_RATIO * msg_data.x_vel
            chassis_y_vel = CHASSIS_LINEAR_VEL_RATIO * msg_data.y_vel
            chassis_angular_vel = CHASSIS_ANGULAR_VEL_RATIO * msg_data.angular_vel
            print(f"control mode vr:{control_mode_vr}, control mode: control mode: {control_mode}")
        breakpoint = msg_data.breakpoint
        exception = msg_data.exception
        return stick_pos, stick_quat, gripper_pressed, disconnect_flag, control_mode, waist_direction, pitch_direction, vertical_direction, chassis_x_vel, chassis_y_vel, chassis_angular_vel, breakpoint, exception

    def process_vrstick_position(self, stick_pos_robot_coord):
        # if self.disconnect_status:
        #     smoothed_new_position = self.last_vrstick_pos_robot_coord
        # else:
        smoothed_new_position = 0.5 * np.array(stick_pos_robot_coord) + 0.5 * self.last_vrstick_pos_robot_coord
        movement = LENGTH_RATIO * (smoothed_new_position - self.init_vrstick_pos_robot_coord)
        target_robot_position = movement + self.init_position

        self.last_vrstick_pos_robot_coord = smoothed_new_position
        return target_robot_position
    
    def process_vrstick_quaternion(self, stick_quat_unity_coord:Quaternion):
        # if self.disconnect_status:
        #     smoothed_new_quat = self.last_vrstick_quat_unity_coord
        # else:
        smoothed_new_quat = Quaternion.slerp(self.last_vrstick_quat_unity_coord, stick_quat_unity_coord, 0.5)
        current_quaternion_change = smoothed_new_quat * self.init_vrstick_quat_unity_coord.inverse
        rel_rot_in_robot = np.dot(np.dot(self.unity_to_robot_mat, current_quaternion_change.rotation_matrix), self.unity_to_robot_mat_inv)
        self.last_vrstick_quat_unity_coord = smoothed_new_quat

        current_quaternion = Quaternion(matrix=rel_rot_in_robot) * self.init_quaternion
        return current_quaternion

    def process_gripper_pressed(self, gripper_pressed):
        curr_time = time.time()
        if gripper_pressed and curr_time - self.gripper_last_update_time > 0.3:
            self.gripper_close_status = not self.gripper_close_status
            self.gripper_last_update_time = curr_time
    
    def process_disconnect_pressed(self, disconnect_pressed):
        curr_time = time.time()
        if disconnect_pressed and curr_time - self.disconnect_last_update_time > 0.3:
            self.disconnect_status = not self.disconnect_status
            self.disconnect_last_update_time = curr_time
            if disconnect_pressed:
                print(f"disconnect pressed, current disconnect status {self.disconnect_status}")

    def callback(self, msg_data: VrPose):
        stick_pos, stick_quat, gripper_pressed, disconnect_flag, control_mode, waist_direction, pitch_direction, vertical_direction, x_vel, y_vel, angular_vel, breakpoint, exception = self.parse_vrpose_data(msg_data)
        torso_mode_jump = False    
        if control_mode == 0:
            
            stick_pos_robot_coord = np.dot(self.unity_to_robot_mat, stick_pos)

            if self.init_vr_call:
                self.process_first_vr_call(stick_pos_robot_coord, stick_quat)
            
            self.process_gripper_pressed(gripper_pressed)
            self.process_disconnect_pressed(disconnect_flag)
            target_robot_position = self.process_vrstick_position(stick_pos_robot_coord)
            target_robot_quaternion = self.process_vrstick_quaternion(stick_quat)
            self.last_target_position = target_robot_position
            self.last_target_quaternion = target_robot_quaternion 
            self.ros_adapter.publish_arm_command(target_robot_position, target_robot_quaternion, self.gripper_close_status)
            self.callback_times += 1
            self.ros_adapter.publish_chassis_command(0, 0, 0)

            self.ros_adapter.publish_torso_command(0, 0, 0, torso_mode_jump)
        elif control_mode == 1:
            self.ros_adapter.publish_chassis_command(0, 0, 0)
            if self.last_target_position is not None:
                target_arm_position = self.last_target_position
            else:
                target_arm_position = self.init_position
            if self.last_target_quaternion is not None:
                target_arm_quaternion = self.last_target_quaternion
            else:
                target_arm_quaternion = self.init_quaternion    
            self.ros_adapter.publish_arm_command(target_arm_position, target_arm_quaternion, self.gripper_close_status)

            if self.last_mode != 1:
                torso_mode_jump = True

            self.ros_adapter.publish_torso_command_v2(msg_data.torso_up, msg_data.torso_down)
        elif control_mode == 2:
            if self.last_target_position is not None:
                target_arm_position = self.last_target_position
            else:
                target_arm_position = self.init_position
            if self.last_target_quaternion is not None:
                target_arm_quaternion = self.last_target_quaternion
            else:
                target_arm_quaternion = self.init_quaternion 
            self.ros_adapter.publish_arm_command(target_arm_position, target_arm_quaternion, self.gripper_close_status)
            
            self.ros_adapter.publish_torso_command(0, 0, 0, torso_mode_jump)  # todo(kevinkedong) 健宁负责此处
            self.ros_adapter.publish_chassis_command(x_vel, y_vel, angular_vel)
        if self.pub_breakpoint:  # arm_name == "left_arm":
            self.ros_adapter.publish_data_collection_command(breakpoint, exception)
        self.last_mode = control_mode



class VrServerNode():

    def __init__(self, use_left=True, use_right=True) -> None:
        self.use_left = use_left
        self.use_right = use_right
        
        if self.use_left:
            self.left_ros_adapter = ManipVrRosAdapter(True)
            self.left_vr_pose_processor = VrPoseProcessor(True, self.left_ros_adapter, True)
        if self.use_right:
            self.right_ros_adapter = ManipVrRosAdapter(False)
            if self.use_left:
                self.right_vr_pose_processor = VrPoseProcessor(False, self.right_ros_adapter, False)
            else:
                self.right_vr_pose_processor = VrPoseProcessor(False, self.right_ros_adapter, True)

    def start_config(self):
        if self.use_right:
            self.right_ros_adapter.start_command()
            time.sleep(2)
        if self.use_left:
            self.left_ros_adapter.start_command()
            time.sleep(2)
    
    def start_listen(self):
        if self.use_left:
            self.left_vr_pose_processor.start_listen()
        if self.use_right:
            self.right_vr_pose_processor.start_listen()
    
    def stop_listen(self):
        if self.use_left:
            self.left_vr_pose_processor.stop_listen()
        if self.use_right:
            self.right_vr_pose_processor.stop_listen()
            
    def callback_complete(self, msg):
        rospy.loginfo("/relaxed_ik/completed_signal starting to listen...")
        self.start_listen()
    
    def run(self):
        rospy.init_node("real_vr_publisher")
        rate = rospy.Rate(1000)
        rospy.Subscriber("/relaxed_ik/completed_signal", Empty, self.callback_complete)
        rospy.loginfo("Waiting for /completed_signal...")
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    server = VrServerNode()
    server.run()
