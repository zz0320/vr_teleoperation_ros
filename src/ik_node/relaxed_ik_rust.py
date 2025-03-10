#! /usr/bin/env python3

import ctypes
import numpy as np
import os
import rospkg
import rospy
import sys
# import src.ik_node.transformations as T
import yaml

from vr_teleoperation_msg.srv import IKPose, IKPoseResponse
from vr_teleoperation_msg.msg import EEPoseGoals, EEVelGoals
from hdas_msg.msg import motor_control
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from urdf_parser_py.urdf import URDF

package_dir = os.path.dirname(os.path.abspath(__file__)) + '/../../'
sys.path.append(package_dir)
print(sys.path)
from src.ik_node import custom_kdl_parser
import PyKDL as kdl
from src.ik_node.robot import Robot

# path_to_src = rospkg.RosPack().get_path('vr_teleoperation_ros') + '/../../modules/relaxed_ik_core'
print(rospkg.RosPack().get_path('vr_teleoperation_ros'))
sys.path.append(package_dir + '/wrappers')
from python_wrapper import RelaxedIKRust


class RelaxedIK:
    def __init__(self, left, right):
        rospy.sleep(1)
        arm_name = ''
        if left == 1:
            arm_name = 'left'
            self.last_pos = [1.57, 2.75, -2.3, 0.00, 0.00, 0.00]
            self.control_topic = '/motion_control/control_arm_left'
            self.sub_pose_topic = '/motion_target/target_pose_arm_left'
            self.sub_joint_topic = '/hdas/feedback_arm_left'
            self.current_ee_tf_topic = '/relaxed_ik/motion_control/pose_ee_arm_left'
            self.alg_angle_topic = 'relaxed_ik/joint_angle_solutions_left'
            self.ee_pose_topic = 'left_ee'
        if right == 1:
            arm_name = 'right'
            self.last_pos = [-1.57, 2.75, -2.3, 0.00, 0.00, 0.00]
            self.control_topic = '/motion_control/control_arm_right'
            self.sub_pose_topic = '/motion_target/target_pose_arm_right'
            self.sub_joint_topic = '/hdas/feedback_arm_right'
            self.current_ee_tf_topic = '/relaxed_ik/motion_control/pose_ee_arm_right'
            self.alg_angle_topic = 'relaxed_ik/joint_angle_solutions_right'
            self.ee_pose_topic = 'right_ee'
        default_setting_file_path = package_dir + '/configs/settings_' + arm_name + '.yaml'
        self.arm_name = arm_name
        setting_file_path = ""
        try:
            setting_file_path = rospy.get_param('setting_file_path')
        except:
            pass

        if setting_file_path == "":
            print("Relaxed IK Rust: no setting file path is given, using the default setting file -- {}".format(default_setting_file_path))
            setting_file_path = default_setting_file_path

        try:
            self.use_visualization = rospy.get_param('~use_visualization')
        except:
            self.use_visualization = False

        os.chdir(package_dir)

        # Load the infomation

        print("setting_file_path: ", setting_file_path)
        setting_file = open(setting_file_path, 'r')
        settings = yaml.load(setting_file, Loader=yaml.FullLoader)

        urdf_file = open(package_dir + '/configs/urdfs/' + settings["urdf"], 'r')
        urdf_string = urdf_file.read()
        rospy.set_param('robot_description', urdf_string)

        self.relaxed_ik = RelaxedIKRust(setting_file_path)

        # Services
        self.ik_pose_service = rospy.Service('relaxed_ik/solve_pose', IKPose, self.handle_ik_pose)

        self.pose_stamped_pub = rospy.Publisher(f'relaxed_ik/ik_ee_pose_{self.arm_name}', PoseStamped, queue_size=1)
        self.current_pose_stamped_pub = rospy.Publisher(self.current_ee_tf_topic, PoseStamped, queue_size=1)
        self.target_pose_stamped_pub = rospy.Publisher(f'relaxed_ik/target_ee_pose_{self.arm_name}', PoseStamped, queue_size=1)
        
        self.motor_pub = rospy.Publisher(self.control_topic, motor_control, queue_size=1)


        # Publishers
        self.angles_pub = rospy.Publisher(self.alg_angle_topic, JointState, queue_size=1)
        if self.use_visualization:
            self.vis_ee_pub = rospy.Publisher('relaxed_ik/vis_ee_poses', EEPoseGoals, queue_size=1)

        self.robot = Robot(setting_file_path)

        self.js_msg = JointState()
        self.js_msg.name = self.robot.articulated_joint_names
        self.js_msg.position = []

        if 'starting_config' not in settings:
            settings['starting_config'] = [0.0] * len(self.js_msg.name)
        else:
            assert len(settings['starting_config']) == len(self.js_msg.name), \
                    "Starting config length does not match the number of joints"
            for i in range(len(self.js_msg.name)):
                self.js_msg.position.append( settings['starting_config'][i] )

        # Subscribers
        self.current_pose_msg = PoseStamped()
        rospy.Subscriber('/relaxed_ik/ee_pose_goals', EEPoseGoals, self.pose_goals_cb)
        rospy.Subscriber('/relaxed_ik/ee_vel_goals', EEVelGoals, self.pose_vels_cb)
        rospy.Subscriber('/relaxed_ik/reset', JointState, self.reset_cb)
        rospy.Subscriber(self.sub_joint_topic, JointState, self.fk_callback)
        rospy.Subscriber(self.sub_pose_topic, PoseStamped, self.pose_callback)

        self.last_time = rospy.Time.now()
        
        self.protected_value = 0.3
        
        print('protected value = ', self.protected_value)
        print('\n')

        print("\nSolver RelaxedIK initialized!!!!!!!!!\n")

    def get_ee_pose(self):
        ee_poses = self.relaxed_ik.get_ee_positions()
        ee_poses = np.array(ee_poses)
        ee_poses = ee_poses.reshape((len(ee_poses)//6, 6))
        ee_poses = ee_poses.tolist()
        return ee_poses

    
    def list_norm(self, list1, list2):
        # 将列表转换为NumPy数组
        arr1 = np.array(list1)
        arr2 = np.array(list2)
        
        # 计算两个数组的差
        diff = arr1 - arr2
        
        # 计算差的欧几里得范数
        norm = np.linalg.norm(diff)
    
        return norm
    
    def pose_callback(self, msg):
        positions = []
        orientations = []
        tolerances = []

        # Extract position data
        positions.append(msg.pose.position.x)
        positions.append(msg.pose.position.y)
        positions.append(msg.pose.position.z)
        
        # if self.list_norm(positions, [self.current_pose_msg.pose.position.x, self.current_pose_msg.pose.position.y, self.current_pose_msg.pose.position.z]) > self.protected_value:
        #     print("input far from current position!\n")
        #     return

        # Extract orientation data
        orientations.append(msg.pose.orientation.x)
        orientations.append(msg.pose.orientation.y)
        orientations.append(msg.pose.orientation.z)
        orientations.append(msg.pose.orientation.w)

        # Set default tolerances if not provided
        for _ in range(6):
            tolerances.append(0.001)

        target_pose_stamped = PoseStamped()
        target_pose_stamped.header.stamp = rospy.Time.now()
        target_pose_stamped.header.frame_id = "base_link"

        target_pose_stamped.pose.position.x = msg.pose.position.x
        target_pose_stamped.pose.position.y = msg.pose.position.y
        target_pose_stamped.pose.position.z = msg.pose.position.z

        target_pose_stamped.pose.orientation.x = msg.pose.orientation.x
        target_pose_stamped.pose.orientation.y = msg.pose.orientation.y
        target_pose_stamped.pose.orientation.z = msg.pose.orientation.z
        target_pose_stamped.pose.orientation.w = msg.pose.orientation.w

        self.target_pose_stamped_pub.publish(target_pose_stamped)

        ik_solution = self.relaxed_ik.solve_position(positions, orientations, tolerances)

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "base_link"

        ik_pose = self.robot.fk(ik_solution)[0]

        pose_stamped.pose.position.x = ik_pose.position.x
        pose_stamped.pose.position.y = ik_pose.position.y
        pose_stamped.pose.position.z = ik_pose.position.z

        pose_stamped.pose.orientation.x = ik_pose.orientation.x
        pose_stamped.pose.orientation.y = ik_pose.orientation.y
        pose_stamped.pose.orientation.z = ik_pose.orientation.z
        pose_stamped.pose.orientation.w = ik_pose.orientation.w


        self.pose_stamped_pub.publish(pose_stamped)
        # print('current ee pose', self.robot.fk(ik_solution)[0].position.x)

        self.js_msg.header.stamp = rospy.Time.now()
        self.js_msg.position = ik_solution
        

        dist = np.linalg.norm([msg.pose.position.x - self.current_pose_msg.pose.position.x, msg.pose.position.y - self.current_pose_msg.pose.position.y, msg.pose.position.z - self.current_pose_msg.pose.position.z])
        alg_dist = np.linalg.norm([msg.pose.position.x - ik_pose.position.x, msg.pose.position.y - ik_pose.position.y, msg.pose.position.z - ik_pose.position.z])
        # vel = dist/ (rospy.Time.now() - self.last_time).to_sec()
        
        
        q_vel = [(cuurent_q - last_q) / (rospy.Time.now() - self.last_time).to_sec() for cuurent_q, last_q in zip(ik_solution, self.last_pos)]
        self.last_pos = ik_solution
        self.last_time = rospy.Time.now()
        self.js_msg.velocity = q_vel
        self.js_msg.effort = [0, 0]
        self.js_msg.effort[0] = dist
        self.js_msg.effort[1] = alg_dist
        self.angles_pub.publish(self.js_msg)

        motor_msg = motor_control()
        motor_msg.p_des = ik_solution
        motor_msg.kp = [0]
        motor_msg.kd = [0]
        motor_msg.v_des = [12.0, 12.0, 12.0, 12.0, 12.0, 12.0]
        motor_msg.t_ff =  [0.8, 0.8, 0.8, 0.8, 0.8, 0.8]
        self.motor_pub.publish(motor_msg)
        
    
    def handle_ik_pose(self, req):
        positions = []
        orientations = []
        tolerances = []
        for i in range(len(req.ee_poses)):
            positions.append(req.ee_poses[i].position.x)
            positions.append(req.ee_poses[i].position.y)
            positions.append(req.ee_poses[i].position.z)
            orientations.append(req.ee_poses[i].orientation.x)
            orientations.append(req.ee_poses[i].orientation.y)
            orientations.append(req.ee_poses[i].orientation.z)
            orientations.append(req.ee_poses[i].orientation.w)
            if i < len(req.tolerances):
                tolerances.append(req.tolerances[i].linear.x)
                tolerances.append(req.tolerances[i].linear.y)
                tolerances.append(req.tolerances[i].linear.z)
                tolerances.append(req.tolerances[i].angular.x)
                tolerances.append(req.tolerances[i].angular.y)
                tolerances.append(req.tolerances[i].angular.z)
            else:
                for j in range(6):
                    tolerances.append(0.0)
        print('orientations', orientations)
        if self.use_visualization:
            vis_msg = EEPoseGoals()
            vis_msg.ee_poses = req.ee_poses
            vis_msg.tolerances = req.tolerances
            self.vis_ee_pub.publish(vis_msg)

        target_pose_stamped = PoseStamped()
        target_pose_stamped.header.stamp = rospy.Time.now()
        target_pose_stamped.header.frame_id = "base_link"

        target_pose_stamped.pose.position.x = req.ee_poses[0].position.x
        target_pose_stamped.pose.position.y = req.ee_poses[0].position.y
        target_pose_stamped.pose.position.z = req.ee_poses[0].position.z

        target_pose_stamped.pose.orientation.x = req.ee_poses[0].orientation.x
        target_pose_stamped.pose.orientation.y = req.ee_poses[0].orientation.y
        target_pose_stamped.pose.orientation.z = req.ee_poses[0].orientation.z
        target_pose_stamped.pose.orientation.w = req.ee_poses[0].orientation.w

        self.target_pose_stamped_pub.publish(target_pose_stamped)

        ik_solution = self.relaxed_ik.solve_position(positions, orientations, tolerances)

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "base_link"

        ik_pose = self.robot.fk(ik_solution)[0]

        pose_stamped.pose.position.x = ik_pose.position.x
        pose_stamped.pose.position.y = ik_pose.position.y
        pose_stamped.pose.position.z = ik_pose.position.z

        pose_stamped.pose.orientation.x = ik_pose.orientation.x
        pose_stamped.pose.orientation.y = ik_pose.orientation.y
        pose_stamped.pose.orientation.z = ik_pose.orientation.z
        pose_stamped.pose.orientation.w = ik_pose.orientation.w


        # error_dist = [ik_pose.position.x - req.ee_poses[0].position.x, ik_pose.position.y - req.ee_poses[0].position.y, 
        #               ik_pose.position.z - req.ee_poses[0].position.z]
        # error_norm = np.linalg.norm(error_dist)

        # if(error_norm > 0.02):
        #     current_pose = self.robot.fk(self.js_msg.position)[0]
        #     orientations = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        #     ik_solution = self.relaxed_ik.solve_position(positions, orientations, tolerances)

        self.pose_stamped_pub.publish(pose_stamped)
        # print('current ee pose', self.robot.fk(ik_solution)[0].position.x)

        self.js_msg.header.stamp = rospy.Time.now()
        self.js_msg.position = ik_solution
        self.angles_pub.publish(self.js_msg)
        
        motor_msg = motor_control()
        motor_msg.p_des = ik_solution
        motor_msg.kp = [0]
        motor_msg.kd = [0]
        motor_msg.v_des = [12.0, 12.0, 12.0, 12.0, 12.0, 12.0]
        motor_msg.t_ff =  [2.5, 2.5, 2.5, 2.5, 2.5, 2.5]
        self.motor_pub.publish(motor_msg)
        
        res = IKPoseResponse()
        res.joint_state = ik_solution

        return res

    def reset_cb(self, msg):
        n = len(msg.position)
        x = (ctypes.c_double * n)()
        for i in range(n):
            x[i] = msg.position[i]
        self.relaxed_ik.reset(x)

    def pose_goals_cb(self, msg):
        positions = []
        orientations = []
        tolerances = []
        for i in range(len(msg.ee_poses)):
            positions.append(msg.ee_poses[i].position.x)
            positions.append(msg.ee_poses[i].position.y)
            positions.append(msg.ee_poses[i].position.z)
            orientations.append(msg.ee_poses[i].orientation.x)
            orientations.append(msg.ee_poses[i].orientation.y)
            orientations.append(msg.ee_poses[i].orientation.z)
            orientations.append(msg.ee_poses[i].orientation.w)
            if i < len(msg.tolerances):
                tolerances.append(msg.tolerances[i].linear.x)
                tolerances.append(msg.tolerances[i].linear.y)
                tolerances.append(msg.tolerances[i].linear.z)
                tolerances.append(msg.tolerances[i].angular.x)
                tolerances.append(msg.tolerances[i].angular.y)
                tolerances.append(msg.tolerances[i].angular.z)
            else:
                for j in range(6):
                    tolerances.append(0.0)

        ik_solution = self.relaxed_ik.solve_position(positions, orientations, tolerances)

        # Publish the joint angle solution
        self.js_msg.header.stamp = rospy.Time.now()
        self.js_msg.position = ik_solution
        self.angles_pub.publish(self.js_msg)

    def pose_vels_cb(self, msg):
        linear_vels = []
        angular_vels = []
        tolerances = []
        for i in range(len(msg.ee_vels)):
            linear_vels.append(msg.ee_vels[i].linear.x)
            linear_vels.append(msg.ee_vels[i].linear.y)
            linear_vels.append(msg.ee_vels[i].linear.z)
            angular_vels.append(msg.ee_vels[i].angular.x)
            angular_vels.append(msg.ee_vels[i].angular.y)
            angular_vels.append(msg.ee_vels[i].angular.z)
            if i < len(msg.tolerances):
                tolerances.append(msg.tolerances[i].linear.x)
                tolerances.append(msg.tolerances[i].linear.y)
                tolerances.append(msg.tolerances[i].linear.z)
                tolerances.append(msg.tolerances[i].angular.x)
                tolerances.append(msg.tolerances[i].angular.y)
                tolerances.append(msg.tolerances[i].angular.z)
            else:
                for j in range(6):
                    tolerances.append(0.0)

        ik_solution = self.relaxed_ik.solve_velocity(linear_vels, angular_vels, tolerances)

        assert len(ik_solution) == len(self.robot.articulated_joint_names)

        # Publish the joint angle solution
        self.js_msg.header.stamp = rospy.Time.now()
        self.js_msg.position = ik_solution
        self.angles_pub.publish(self.js_msg)

    def fk_callback(self, msg):
        current_pose = self.robot.fk(msg.position)[0]
        self.current_pose_msg.header.stamp = rospy.Time.now()
        self.current_pose_msg.header.frame_id = self.ee_pose_topic 
        self.current_pose_msg.pose.position.x = current_pose.position.x
        self.current_pose_msg.pose.position.y = current_pose.position.y
        self.current_pose_msg.pose.position.z = current_pose.position.z

        self.current_pose_msg.pose.orientation.x = current_pose.orientation.x
        self.current_pose_msg.pose.orientation.y = current_pose.orientation.y
        self.current_pose_msg.pose.orientation.z = current_pose.orientation.z
        self.current_pose_msg.pose.orientation.w = current_pose.orientation.w


        # error_dist = [ik_pose.position.x - req.ee_poses[0].position.x, ik_pose.position.y - req.ee_poses[0].position.y, 
        #               ik_pose.position.z - req.ee_poses[0].position.z]
        # error_norm = np.linalg.norm(error_dist)

        # if(error_norm > 0.02):
        #     current_pose = self.robot.fk(self.js_msg.position)[0]
        #     orientations = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        #     ik_solution = self.relaxed_ik.solve_position(positions, orientations, tolerances)

        self.current_pose_stamped_pub.publish(self.current_pose_msg)

if __name__ == '__main__':
    import sys
    use_left = (sys.argv[1] == "1")
    use_right = (sys.argv[2] == "1")
    print("use_left", use_left)
    print("use_right", use_right)
    rospy.init_node('relaxed_ik')
    relaxed_ik = RelaxedIK(use_left, use_right)
    rospy.spin()
