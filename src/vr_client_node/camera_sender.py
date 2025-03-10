#!/usr/bin/env python3
import sys
import os
import cv2
import rospy
import socket
import numpy as np
from cv_bridge import CvBridge

# 设置UDP参数
# UDP_IP = "192.168.110.58"  # 目标IP地址
MAX_DGRAM_SIZE = 65507  # UDP数据报的最大大小

from sensor_msgs.msg import CompressedImage

# 初始化ZED相机

class ZedResender():

    def __init__(self, camera_topic_name, udp_ip, udp_port, image_width=320, image_height=240, quality=100):
        rospy.init_node(f"port_{udp_port}_resender_node")
        self.br = CvBridge()
        self.camera_topic_name = camera_topic_name
        self.udp_ip = udp_ip  # UDP_IP
        self.udp_port = udp_port
        self.sub_camera = rospy.Subscriber(camera_topic_name, CompressedImage, self._head_camera_callback)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.image_w = image_width
        self.image_h = image_height
        self.image_quality = quality
        print(f"start resending topic {camera_topic_name} to ip {self.udp_ip} port {udp_port} with image quality {quality}")

    def _head_camera_callback(self, msg:CompressedImage):
        img_camera = self.br.compressed_imgmsg_to_cv2(msg)
        img_camera = np.array(cv2.resize(img_camera, (self.image_w, self.image_h)))
        self.send_data(img_camera)

    def send_data(self, frame_resized):
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.image_quality]  # 调整质量参数（0-100）
        _, frame_encoded = cv2.imencode(".jpg", frame_resized, encode_param)
        data = frame_encoded.tobytes()

        # 分片发送
        for i in range(0, len(data), MAX_DGRAM_SIZE):
            header = f"{i // MAX_DGRAM_SIZE:06}{(len(data) + MAX_DGRAM_SIZE - 1) // MAX_DGRAM_SIZE:04}"
            self.sock.sendto(
                header.encode() + data[i : i + MAX_DGRAM_SIZE], (self.udp_ip, self.udp_port)
            )
    
    def run(self):
        rospy.spin()


if __name__ == "__main__":
    topic_name = '/hdas/camera_head/left_raw/image_raw_color/compressed'
    udp_ip = os.environ.get('VR_IP')
    udp_port = 5005
    width = 320
    height = 240
    quality = 70
    sender = ZedResender(topic_name, udp_ip, udp_port, width, height, quality)
    sender.run()
    