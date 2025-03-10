#!/usr/bin/env python3

import time
import rospy
import copy
import numpy as np
import os
import subprocess
import threading
import signal

from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PoseStamped, Twist, Pose
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from pyquaternion import Quaternion as pyQuaternion
from hdas_msg.msg import motor_control


class ManipVrRosAdapter:

    def __init__(self, is_left_arm=False):
        self.gripper_pub_topic = "/motion_control/position_control_gripper_left"
        self.gripper_sub_topic = "/hdas/feedback_gripper_left"
        self.arm_ee_pub_topic = "/motion_target/target_pose_arm_left"
        self.arm_ee_sub_topic = "/relaxed_ik/motion_control/pose_ee_arm_left"
        self.torso_sub_topic = "/hdas/feedback_torso"
        self.torso_pub_topic = "/motion_target/target_joint_state_torso"
        self.torse_pub_topic_direct = '/motion_control/control_torso'
        self.chassis_sub_topic = "/hdas/feedback_chassis"
        self.chassis_pub_topic = "/motion_target/target_speed_chassis"
        self.breakpoint_topic = "/breakpoint"
        self.exception_topic = "/exception"
        self.arm_name = "left" if is_left_arm else "right"
        self.gripper_open_val = 103.0
        self.gripper_close_val = -3.0 # 0.0
        self.gripper_open_clamp_val = 100.0
        self.gripper_close_clamp_val = 0.0
        self.gripper_cmd_offset = 20.0
        self.is_left_arm = is_left_arm
        self._subscribed_ee_pose = None
        self._subscribed_gripper_pos = None
        self._subscribed_torso_pose = None
        self._baseline_torso_pose = None
        # self.control_rate = 15.0
        if not self.is_left_arm:
            self._replace_arm_name()

        self._gripper_pub = rospy.Publisher(self.gripper_pub_topic, Float32, queue_size=10)
        self._arm_ee_pub = rospy.Publisher(self.arm_ee_pub_topic, PoseStamped, queue_size=10)
        self._torso_sub = rospy.Subscriber(self.torso_sub_topic, JointState, self._torso_callback)
        self._torso_pub = rospy.Publisher(self.torso_pub_topic, JointState, queue_size=10)
        self._gripper_sub = rospy.Subscriber(self.gripper_sub_topic, JointState, self._gripper_callback)
        self._chassis_pub = rospy.Publisher(self.chassis_pub_topic, Twist, queue_size=10)
        self._torso_direct_pub = rospy.Publisher(self.torse_pub_topic_direct, motor_control, queue_size=10)
        
        # 极简音频播放设置
        self._play_lock = threading.Lock()  # 音频播放锁
        self._last_play = 0  # 上次播放时间
        
        # 设置音频文件目录
        self._voice_dir = "/home/nvidia/vr_workspace/install/lib/vr_teleoperation_ros/voice"
        
        # 确保ROS节点已就绪
        time.sleep(0.2)
        
        if not self.is_left_arm:
            self._replace_arm_name()

        self._gripper_pub = rospy.Publisher(self.gripper_pub_topic, Float32, queue_size=10)
        self._arm_ee_pub = rospy.Publisher(self.arm_ee_pub_topic, PoseStamped, queue_size=10)
        self._torso_sub = rospy.Subscriber(self.torso_sub_topic, JointState, self._torso_callback)
        self._torso_pub = rospy.Publisher(self.torso_pub_topic, JointState, queue_size=10)
        self._gripper_sub = rospy.Subscriber(self.gripper_sub_topic, JointState, self._gripper_callback)
        self._chassis_pub = rospy.Publisher(self.chassis_pub_topic, Twist, queue_size=10)
        self._torso_direct_pub = rospy.Publisher(self.torse_pub_topic_direct, motor_control, queue_size=10)
        
        # 测试播放功能
        try:
            # 采用更简单的系统调用直接播放
            rospy.loginfo("系统启动，测试音频系统...")
            test_file = os.path.join(self._voice_dir, "start.mp3")
            if os.path.exists(test_file):
                os.system(f"mpg123 -q {test_file} &")
                rospy.loginfo("播放测试音频")
        except Exception as e:
            rospy.logerr(f"音频系统测试失败: {e}")
        
        time.sleep(0.2)
        # for link 6
        if self.is_left_arm:
            self.init_pos = np.array([0.04542, 0.2321, -0.4747])
            self.init_quat = np.array([0.9737, 0.01459, 0.0339, 0.22475])  # x, y, z, w
        else:
            self.init_pos = np.array([0.04446, -0.2252, -0.4747])
            self.init_quat = np.array([0.9725, 0.007612, 0.03506, -0.2297])  # x, y, z, w
        time.sleep(1.0)
        rospy.loginfo(f"{self.arm_name} ros adapter created")

    """
    使用顺序：
        1. 先使用start_command控制机械臂到指定的末端位姿
        2. 等到机械臂到达指定位姿，并停止后，调用listen_init_pose_from_ros函数更新init_pose
        3. 使用get_init_pose获取当前位姿
    """
    def listen_init_pose_from_ros(self):
        self._arm_ee_sub = rospy.Subscriber(self.arm_ee_sub_topic, PoseStamped, self._ee_pose_callback)
        time.sleep(0.5)
        if self._subscribed_ee_pose is not None:
            print(self._subscribed_ee_pose)
            self.init_pos = np.array([self._subscribed_ee_pose.position.x, self._subscribed_ee_pose.position.y, self._subscribed_ee_pose.position.z])
            self.init_quat = np.array([self._subscribed_ee_pose.orientation.x, self._subscribed_ee_pose.orientation.y,
                                       self._subscribed_ee_pose.orientation.z, self._subscribed_ee_pose.orientation.w])
            rospy.loginfo(f"{self.arm_name} receives initial pose from {self.arm_ee_sub_topic} init position {self.init_pos} init quat {self.init_quat}")

    def _gripper_callback(self, msg:JointState):
        self._subscribed_gripper_pos = msg.position[0]

    def get_init_pose(self):
        return copy.copy(self.init_pos), copy.copy(self.init_quat)

    def _replace_arm_name(self):
        self.gripper_pub_topic = self.gripper_pub_topic.replace("left", "right")
        self.gripper_sub_topic = self.gripper_sub_topic.replace("left", "right")
        self.arm_ee_pub_topic = self.arm_ee_pub_topic.replace("left", "right")
        self.arm_ee_sub_topic = self.arm_ee_sub_topic.replace("left", "right")
    
    def _ee_pose_callback(self, msg:PoseStamped):
        self._subscribed_ee_pose = msg.pose

    def _torso_callback(self,msg:JointState):
        self._subscribed_torso_pose = msg.position # it should be a list
    
    def start_command(self):
        self.publish_arm_command(self.init_pos, 
                                 pyQuaternion(self.init_quat[3], self.init_quat[0], self.init_quat[1], self.init_quat[2]),
                                 False)
        rospy.loginfo(f"{self.arm_name} publishes start command")
    
    def publish_arm_command(self, position, quat_wxyz:pyQuaternion, gripper_close:bool):
        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = position[0]
        pose_stamped.pose.position.y = position[1]
        pose_stamped.pose.position.z = position[2]

        pose_stamped.pose.orientation.x = quat_wxyz.x
        pose_stamped.pose.orientation.y = quat_wxyz.y
        pose_stamped.pose.orientation.z = quat_wxyz.z
        pose_stamped.pose.orientation.w = quat_wxyz.w

        self._arm_ee_pub.publish(pose_stamped)
        rospy.logdebug(f"{self.arm_name} publish arm ee command")

        gripper_pos = (self.gripper_close_val if gripper_close else self.gripper_open_val)
        clamped_gripper_pos = self.gripper_soft_clamp(gripper_pos)
        gripper_cmd = Float32()
        gripper_cmd.data = clamped_gripper_pos

        self._gripper_pub.publish(gripper_cmd)
        rospy.logdebug(f"{self.arm_name} publish gripper command {gripper_pos:.2f}")

    def form_twist_msg(self, direction):
        if direction == 1:
            angular_increment = 1
        elif direction == 2:
            angular_increment = -1
        elif direction == 0:
            angular_increment = 0
        return angular_increment
    
    def publish_torso_command_v2(self, up, down):
        if down:
            msg = motor_control()
            msg.header.seq = 0
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = ''
            msg.name = ''
            msg.p_des = [0.7, -1.4, -1.0, 0.0]
            msg.v_des = [0.2, 0.4, 0.3, 0.3]
            msg.kp = [0]
            msg.kd = [0]
            msg.t_ff = [0]
            msg.mode = 0
            self._torso_direct_pub.publish(msg)
        if up:
            msg = motor_control()
            msg.header.seq = 0
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = ''
            msg.name = ''
            msg.p_des = [0.0, 0.0, 0.0, 0.0]
            msg.v_des = [0.2, 0.3, 0.2, 0.2]
            msg.kp = [0]
            msg.kd = [0]
            msg.t_ff = [0]
            msg.mode = 0
            self._torso_direct_pub.publish(msg)
        
    def publish_torso_command(self, waist_direction, pitch_direction, vertical_direction, jump):
        if jump:
            self._baseline_torso_pose = self._subscribed_torso_pose
            cur_position = self._baseline_torso_pose #
            torso_cmd = JointState()
            torso_cmd.position = list(cur_position) # next_position
            torso_cmd.velocity = [0.001, 0.001, 0.001, 0.001]
            self._torso_pub.publish(torso_cmd)
        else:
            if self._baseline_torso_pose is None: # handle the case where you start from non zero mode, and this None serves as the jump
                self._baseline_torso_pose = self._subscribed_torso_pose
            torso_cmd = JointState()
            waist_speed = 0.2
            pitch_speed = 0.1
            vertical_speed = 0.1
            angular_y_increment = self.form_twist_msg(waist_direction)
            angular_x_increment = self.form_twist_msg(pitch_direction)
            linear_y_increment = self.form_twist_msg(vertical_direction)
            
            cur_position = self._baseline_torso_pose
            cur_observation = self._subscribed_torso_pose
            next_position = list(cur_position) 
            angle_y_0 = (angular_y_increment == 0)
            angle_x_0 = (angular_x_increment == 0)
            linear_y_0 = (linear_y_increment == 0)
            # if (not angle_y_0) and angle_x_0 and linear_y_0: # joint 4, the waist turning left/right
            next_position[3] = cur_observation[3]
            next_position[3] += angular_y_increment * waist_speed
            # torso_cmd.velocity = [0.1, 0.1, 0.1, waist_speed]
            # elif (not angle_x_0) and angle_y_0 and linear_y_0:  # joint 3, the waist turning forward/backward
            next_position[2] = cur_observation[2]
            next_position[2] += angular_x_increment * pitch_speed
            # torso_cmd.velocity = [0.1, 0.1, pitch_speed, 0.1]
            torso_cmd.velocity = [0.1, 0.1, pitch_speed, waist_speed]
            # elif (not angle_x_0) and (not angle_y_0) and linear_y_0:  #
            #     rospy.logwarn(f"Please move only one joint at a time!")
            #     torso_cmd.velocity = [0.1, 0.1, 0.1, 0.1]
            # else:
            #     torso_cmd.velocity = [0.1, 0.1, 0.1, 0.1]
            torso_cmd.position = next_position
            self._torso_pub.publish(torso_cmd)
        rospy.logdebug("publish torso command")

    def gripper_soft_clamp(self, gripper_target_pos):
        sign = 1.0
        temp_current_position = copy.copy(self._subscribed_gripper_pos) # 复制当前抓手位置，避免回调函数持续修改
        if gripper_target_pos < temp_current_position:
            sign = -1.0
        
        gripper_position = temp_current_position + sign * self.gripper_cmd_offset
        clamped_value = np.clip(gripper_position, self.gripper_close_clamp_val, self.gripper_open_clamp_val)
        return clamped_value
    def publish_chassis_command(self, x, y, z):
        chassis_cmd=Twist()
        chassis_cmd.linear.x=x
        chassis_cmd.linear.y=y
        chassis_cmd.linear.z=0
        chassis_cmd.angular.x=0
        chassis_cmd.angular.y=0
        chassis_cmd.angular.z=z
        self._chassis_pub.publish(chassis_cmd)
        rospy.logdebug("publish chassis command")

    def publish_data_collection_command(self, breakpoint, exception):
        # 添加状态跟踪，避免重复触发
        current_state = (breakpoint, exception)
        
        # 检查是否有状态属性，如果没有，创建它
        if not hasattr(self, '_last_collection_state'):
            self._last_collection_state = None
            self._state_change_time = 0
            
        # 如果状态没有变化，不执行任何操作
        if self._last_collection_state == current_state:
            return
            
        # 添加时间限制，确保状态变化不会太频繁（至少0.5秒）
        current_time = time.time()
        if current_time - self._state_change_time < 0.5:
            rospy.loginfo(f"忽略频繁的状态变化: {current_state}, 时间间隔太短")
            return
        
        # 更新状态和时间
        self._last_collection_state = current_state
        self._state_change_time = current_time
        
        # 按照新逻辑处理：先调用服务，成功后再播放提示音
        if exception and not breakpoint:
            rospy.loginfo("正在停止数据采集...")
            
            try:
                # 先调用服务
                service_data = rospy.ServiceProxy('/stop_data_collection', Trigger)
                response = service_data()
                
                # 服务调用成功后，再播放提示音
                if response.success:
                    rospy.loginfo(f"停止数据采集成功: {response.message}")
                    # 播放结束录制的提示音
                    self._play_audio("end")
                else:
                    rospy.logwarn(f"停止数据采集失败: {response.message}")
            except rospy.ServiceException as e:
                rospy.logerr(f"服务调用失败: {e}")
                
        elif breakpoint and not exception:
            rospy.loginfo("正在开始数据采集...")
            
            try:
                # 先调用服务
                service_data = rospy.ServiceProxy('/start_data_collection', Trigger)
                response = service_data()
                
                # 服务调用成功后，再播放提示音
                if response.success:
                    rospy.loginfo(f"开始数据采集成功: {response.message}")
                    # 播放开始录制的提示音
                    self._play_audio("start")
                else:
                    rospy.logwarn(f"开始数据采集失败: {response.message}")
            except rospy.ServiceException as e:
                rospy.logerr(f"服务调用失败: {e}")

    # 完整播放音频的函数
    def _play_audio(self, audio_name):
        """
        播放音频并确保完整播放
        
        Args:
            audio_name: 音频文件名称（如 "start" 或 "end"）
        """
        with self._play_lock:
            try:
                # 确保文件名正确
                if audio_name.endswith('.mp3'):
                    audio_key = audio_name[:-4]
                else:
                    audio_key = audio_name
                    audio_name = f"{audio_name}.mp3"
                
                # 构建音频文件完整路径
                audio_file = os.path.join(self._voice_dir, audio_name)
                if not os.path.exists(audio_file):
                    rospy.logwarn(f"音频文件不存在: {audio_file}")
                    return False
                
                # 检查播放间隔，避免重复播放
                now = time.time()
                if now - self._last_play < 1.0:
                    rospy.loginfo(f"跳过播放 {audio_key}: 距上次播放时间太短")
                    return False
                
                # 更新播放时间
                self._last_play = now
                
                # 确保杀掉所有之前的播放进程
                try:
                    subprocess.call(["pkill", "-9", "mpg123"], 
                                  stdout=subprocess.DEVNULL, 
                                  stderr=subprocess.DEVNULL)
                    # 给一点时间让系统清理
                    time.sleep(0.1)
                except:
                    pass
                
                # 直接播放并强制等待完成
                rospy.loginfo(f"开始播放音频: {audio_key}")
                
                # 使用更可靠的播放参数
                # -q 安静模式：减少输出
                # --no-control 禁用控制接口，避免被外部干扰
                # -b 1024：增大缓冲区，减少中断
                # -o pulse：使用 pulseaudio 输出，通常更稳定
                # -o alsa：如果 pulse 不可用，使用 alsa
                try:
                    # 先尝试 pulseaudio
                    cmd = ["mpg123", "-q", "--no-control", "-b", "1024", "-o", "pulse", audio_file]
                    completed = subprocess.run(
                        cmd,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        timeout=2  # 设置超时，防止卡住
                    )
                    if completed.returncode != 0:
                        # 如果失败，尝试 alsa
                        cmd = ["mpg123", "-q", "--no-control", "-b", "1024", "-o", "alsa", audio_file]
                        subprocess.run(
                            cmd,
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                            timeout=2
                        )
                except subprocess.TimeoutExpired:
                    rospy.logwarn(f"音频播放超时: {audio_key}")
                    # 尝试终止超时的播放进程
                    subprocess.call(["pkill", "-9", "mpg123"])
                
                rospy.loginfo(f"音频播放完成: {audio_key}")
                return True
                
            except Exception as e:
                rospy.logerr(f"音频播放错误: {e}")
                # 尝试清理任何残留的播放进程
                subprocess.call(["pkill", "-9", "mpg123"])
                return False

if __name__ == "__main__":
    rospy.init_node("ros_adapter_test_node")
    ros_adapter = ManipVrRosAdapter(True)
    ros_adapter.start_command()
    ros_adapter.listen_init_pose_from_ros()
