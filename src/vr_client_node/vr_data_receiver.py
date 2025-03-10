#!/usr/bin/env python3

import json
import argparse
import asyncio
import traceback
import websockets
import time
import rospy
from vr_teleoperation_msg.msg import VrPose

# 初始化ROS节点
rospy.init_node("vr_data_receiver", anonymous=True)

# 创建ROS发布者
pose_pub = rospy.Publisher("vr_pose", VrPose, queue_size=10)
# LOCAL_SERVER_IP = "192.168.110.48"

# 添加固定频率发布相关变量
latest_pose_msg = None  # 存储最新的姿态数据
PUBLISH_RATE = 50  # 定义发布频率为50Hz

# 定时器回调函数，以固定频率发布最新的姿态数据
def timer_callback(event):
    global latest_pose_msg
    if latest_pose_msg is not None:
        # 更新时间戳
        latest_pose_msg.header.stamp = rospy.Time.now()
        # 发布消息
        pose_pub.publish(latest_pose_msg)

# 创建定时器，以PUBLISH_RATE设定的频率调用timer_callback
timer = rospy.Timer(rospy.Duration(1.0/PUBLISH_RATE), timer_callback)

def generate_j4j6_cmd(arm_name, up, down, left, right, front, side):
    j4_cmd_value = 0
    if right:
        j4_cmd_value = 1
        print(f"{arm_name} j4 increase value")
    elif left:
        j4_cmd_value = -1
        print(f"{arm_name} j4 decrease value")

    j5_cmd_value = 0
    if up:
        j5_cmd_value = 1
        print(f"{arm_name} j5 increase value")
    elif down:
        j5_cmd_value = -1
        print(f"{arm_name} j5 decrease value")

    j6_cmd_value = 0
    if front:
        j6_cmd_value = 1
        print(f"{arm_name} j6 increase value")
    elif side:
        j6_cmd_value = -1
        print(f"{arm_name} j6 decrease value")

    return [j4_cmd_value, j5_cmd_value, j6_cmd_value]


import numpy as np
# Example functions for quaternion inversion and multiplication.
# These need to be defined or imported from a library.
def quaternion_inverse(q):
    # Assuming q is in [w, x, y, z] format
    w, x, y, z = q
    norm_sq = w*w + x*x + y*y + z*z
    return np.array([w/norm_sq, -x/norm_sq, -y/norm_sq, -z/norm_sq])

def quaternion_multiply(q1, q2):
    # Multiply two quaternions q1 and q2 (both in [w, x, y, z] format)
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([w, x, y, z])

class AccumulateControlManager():
    def __init__(self):
        ...
        self.init_left_pos =np.array([-0.33883524 , 0.89370185 , 0.34637436])# np.zeros(3,dtype=float)
        self.init_left_quat =np.array( [-0.9879697 ,  0.0937537 ,  0.12227403, -0.01323522])# np.array([1.,0.,0.,0.])
        self.init_rigt_pos = np.array([0.30822945, 0.8884089,  0.37185025])# np.zeros(3,dtype=float)
        self.init_rigt_quat =np.array([-0.99207497,  0.03173578, -0.11934622 ,-0.02316332]) #np.array([1.,0.,0.,0.])

        self.last_left_pos = np.zeros(3,dtype=float)
        self.last_left_quat = np.array([1.,0.,0.,0.])
        self.last_rigt_pos =  np.zeros(3,dtype=float)
        self.last_rigt_quat = np.array([1.,0.,0.,0.])

        self.accu_left_pos = np.zeros(3,dtype=float)
        self.accu_left_quat = np.array([1.,0.,0.,0.])
        self.accu_rigt_pos =  np.zeros(3,dtype=float)
        self.accu_rigt_quat = np.array([1.,0.,0.,0.])

        self.reset()

        # self.left_ctrl_stop_cnt=0
        # self.rigt_ctrl_stop_cnt=0

        self.inited=False

        self.head_pose=None
        self.trig_move_l=False
        self.trig_move_r=False
        self.cool_down_l=False
        self.cool_down_r=False

        self.cooldown=0.3

        self.last_press_time=0
        self.last_press_time2=0

    
    def step(self,data):
        left_pos = data["p_leftControllerPosition"]
        left_quat = data["p_leftControllerRotation"]
        right_pos = data["p_rightControllerPosition"]
        right_quat = data["p_rightControllerRotation"]

        l_p = np.array([left_pos["x"], left_pos["y"], left_pos["z"]])
        r_p = np.array([right_pos["x"], right_pos["y"], right_pos["z"]])
        l_q = np.array([
            left_quat["w"],
            left_quat["x"],
            left_quat["y"],
            left_quat["z"],
            
        ])
        r_q = np.array([
            right_quat["w"],
            right_quat["x"],
            right_quat["y"],
            right_quat["z"],
            
        ])

        # print("l_p:",l_p,"r_p:",r_p)
        
        if self.inited:
            self.inited=True
            self.last_left_pos[:]=l_p
            self.last_left_quat[:]=l_q
            self.last_rigt_pos[:]=r_p
            self.last_rigt_quat[:]=r_q
        
        deta_l_p=l_p-self.last_left_pos
        deta_l_q = quaternion_multiply(l_q, quaternion_inverse(self.last_left_quat))

        deta_r_p=r_p-self.last_rigt_pos
        deta_r_q = quaternion_multiply(r_q, quaternion_inverse(self.last_rigt_quat))


        self.last_left_pos[:]=l_p
        self.last_left_quat[:]=l_q
        self.last_rigt_pos[:]=r_p
        self.last_rigt_quat[:]=r_q

        # current_time = time.time()
        # if current_time - self.last_press_time >= self.cooldown and data["p_left_Side_Trigger"]>0.8:
        #     self.last_press_time = current_time
        #     # self.trigger_event()
        #     self.trig_move_l=not self.trig_move_l
        # else:
        #     print("Cooldown active, ignoring button press.")
        
        # if current_time - self.last_press_time2 >= self.cooldown and data["p_right_Side_Trigger"]>0.8:
        #     self.last_press_time2 = current_time
        #     # self.trigger_event()
        #     self.trig_move_r=not self.trig_move_r
        # else:
        #     print("Cooldown active, ignoring button press.")



        # if data["p_right_Side_Trigger"]>0.8 and not self.cool_down_r:
        #     self.trig_move_r=not self.trig_move_r
        #     self.cool_down_r=True
    

        trig_rset_l=data["p_leftHand_Y"]
        trig_rset_r=data["p_rightHand_B"]



        if trig_rset_l:
            print("cur l",l_p,l_q)
            # left reset
            self.accu_left_pos[:] = self.init_left_pos
            self.accu_left_quat[:] = self.init_left_quat

            print(self.accu_left_pos,
              self.accu_left_quat,
              self.accu_rigt_pos,
              self.accu_rigt_quat)
            
        if trig_rset_r:
            print("cur r:",r_p,r_q)
            # right reset
            self.accu_rigt_pos[:] = self.init_rigt_pos
            self.accu_rigt_quat[:] = self.init_rigt_quat
            print(self.accu_left_pos,
              self.accu_left_quat,
              self.accu_rigt_pos,
              self.accu_rigt_quat)

        if data["p_left_Side_Trigger"]>0.8: #self.trig_move_l:

            # run left
            self.accu_left_pos=self.accu_left_pos+deta_l_p
            self.accu_left_quat=quaternion_multiply(deta_l_q, self.accu_left_quat)

            print(self.accu_left_pos,
              self.accu_left_quat,)

        if data["p_right_Side_Trigger"]>0.8:# self.trig_move_r:
            # run left
            self.accu_rigt_pos=self.accu_rigt_pos+deta_r_p
            self.accu_rigt_quat=quaternion_multiply(deta_r_q, self.accu_rigt_quat)
            print(
              self.accu_rigt_pos,
              self.accu_rigt_quat)

        # handle left controller stop
        # counte 15 round
        # if self.last_left_pos== l_p and self.last_left_quat==l_q:
        #     self.left_ctrl_stop_cnt+=1
        # else:
        #     self.left_ctrl_stop_cnt=0

        # if self.left_ctrl_stop_cnt>15:
        #     self.last_left_pos=l_p
        #     self.last_left_quat=l_q
        
        # if self.last_rigt_pos== r_p and self.last_rigt_quat==r_q:
        #     self.rigt_ctrl_stop_cnt+=1
        # else:
        #     self.rigt_ctrl_stop_cnt=0

        # if self.rigt_ctrl_stop_cnt>15:
        #     self.last_rigt_pos=r_p
        #     self.last_rigt_quat=r_q
     

        return self.accu_left_pos,self.accu_left_quat[[1,2,3,0]],self.accu_rigt_pos,self.accu_rigt_quat[[1,2,3,0]]

        ...
    
    def reset(self):
        self.accu_left_pos[:] = self.init_left_pos
        self.accu_left_quat[:] = self.init_left_quat
        self.accu_rigt_pos[:] = self.init_rigt_pos
        self.accu_rigt_quat[:] = self.init_rigt_quat

acm= AccumulateControlManager()


# 处理客户端连接
async def handler(websocket, path):
    global latest_pose_msg  # 使用全局变量存储最新姿态
    # 向客户端发送欢迎消息
    await websocket.send("Welcome to the WebSocket server!")
    print(f"Client connected from {websocket.remote_address}")
    last_trigger_time_left_front = 0
    last_trigger_time_left_side = 0
    try:
        async for message in websocket:
            # print("=================== pose ====================")
            data = json.loads(message)
            # 构造PoseStamped消息
            pose_msg = VrPose()
            pose_msg.header.stamp = rospy.Time.now()
            control_mode = data['p_control_mode']
            pose_msg.control_mode = control_mode
            current_time = time.time()
            if control_mode == 0:
                try:
                    l_p,l_q,r_p,r_q=acm.step(data)
                except Exception as e:
                    print(e)
                    traceback.print_exc()
                head_pos = data["p_headPosition"]
                # left_pos = data["p_leftControllerPosition"]
                # left_quat = data["p_leftControllerRotation"]
                # right_pos = data["p_rightControllerPosition"]
                # right_quat = data["p_rightControllerRotation"]


                

                if acm.head_pose is None:
                    pose_msg.head_position = [head_pos["x"], head_pos["y"], head_pos["z"]]
                    acm.head_pose=np.zeros(3,dtype=float)
                    acm.head_pose[:]=np.array(pose_msg.head_position)
                pose_msg.head_position=    acm.head_pose

                # pose_msg.left_position = [left_pos["x"], left_pos["y"], left_pos["z"]]
                # pose_msg.right_position = [right_pos["x"], right_pos["y"], right_pos["z"]]
                # pose_msg.left_quaternion = [
                #     left_quat["x"],
                #     left_quat["y"],
                #     left_quat["z"],
                #     left_quat["w"],
                # ]
                # pose_msg.left_disconnect_pressed = data["p_leftHand_Y"]
                # pose_msg.right_disconnect_pressed = data["p_rightHand_B"]
                # pose_msg.right_quaternion = [
                #     right_quat["x"],
                #     right_quat["y"],
                #     right_quat["z"],
                #     right_quat["w"],
                # ]

                pose_msg.left_position = l_p
                pose_msg.right_position = r_p
                pose_msg.left_quaternion = l_q
                pose_msg.right_quaternion = r_q



                pose_msg.left_gripper_close = data["p_leftHand_X"]
                if data["p_leftHand_X"]:
                    print("left gripper close")
                pose_msg.right_gripper_close = data["p_rightHand_A"]
                if data["p_rightHand_A"]:
                    print("right gripper close")

                # left_j4j6_cmd = generate_j4j6_cmd(
                #     "left",
                #     data["p_left_Thumbstick_Up"],
                #     data["p_left_Thumbstick_Down"],
                #     data["p_left_Thumbstick_Left"],
                #     data["p_left_Thumbstick_Right"],
                #     data["p_is_left_Front_Closed"],
                #     data["p_is_left_Side_Closed"],
                # )
                # right_j4j6_cmd = generate_j4j6_cmd(
                #     "right",
                #     data["p_right_Thumbstick_Up"],
                #     data["p_right_Thumbstick_Down"],
                #     data["p_right_Thumbstick_Left"],
                #     data["p_right_Thumbstick_Right"],
                #     data["p_is_right_Front_Closed"],
                #     data["p_is_right_Side_Closed"],
                # )
                # pose_msg.left_j4j6_cmd = left_j4j6_cmd
                # pose_msg.right_j4j6_cmd = right_j4j6_cmd
            elif control_mode == 1:
                pose_msg.waist_right = data["p_is_right_Front_Closed"]
                pose_msg.waist_left =  data["p_is_right_Side_Closed"]
                pose_msg.torso_down =  data["p_left_Thumbstick_Up"]
                pose_msg.torso_up =  data["p_left_Thumbstick_Down"]
                pose_msg.tilt_forward =  data["p_right_Thumbstick_Up"]
                pose_msg.tilt_backward =  data["p_right_Thumbstick_Down"]
            elif control_mode==2:
                pose_msg.y_vel = data["p_right_Thumbstick_Left"] - data["p_right_Thumbstick_Right"]
                pose_msg.x_vel = data["p_left_Thumbstick_Up"] - data["p_left_Thumbstick_Down"]
                pose_msg.angular_vel = data["p_is_right_Side_Closed"] - data["p_is_right_Front_Closed"]
            if data["p_left_trigger_Front_state"] > 0.8 and (current_time - last_trigger_time_left_front) > 2.0:
                print("left front pressed")
                pose_msg.breakpoint = True
                last_trigger_time_left_front = current_time
            if data["p_right_trigger_Front_state"] > 0.8 and (current_time - last_trigger_time_left_side) > 2.0:
                pose_msg.exception = True
                print("left side pressed")
                last_trigger_time_left_side = current_time
            
            # 更新全局变量，存储最新姿态数据
            latest_pose_msg = pose_msg
            
            # 注意：不需要在这里直接发布消息，定时器会以固定频率发布
            # pose_pub.publish(pose_msg)

            # 向客户端回显消息
            the_send = "Server already received " + str(data["p_msg_id"])

            await websocket.send(the_send)
    except websockets.exceptions.ConnectionClosed as e:
        print(f"Connection closed: {e}")


# 启动WebSocket服务器
async def main(local_server_ip):
    server = await websockets.serve(handler, local_server_ip, 8765, ping_interval=None)
    print(f"WebSocket server started at ws://{local_server_ip}:8765")
    print(f"Publishing vr_pose topic at {PUBLISH_RATE}Hz")

    await server.wait_closed()

if __name__ == "__main__":
    import os
    ros_ip = os.environ.get('ROS_IP')
    local_server_ip = os.environ.get('R1_IP', ros_ip) 
    # 运行服务器
    asyncio.run(main(local_server_ip))
