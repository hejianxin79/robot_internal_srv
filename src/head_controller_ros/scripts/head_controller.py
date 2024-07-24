#! /usr/bin/env python3
import os
import sys
import time
import threading
import rospy
import json
import random
from std_msgs.msg import Header, String, Float32MultiArray
from sensor_msgs.msg import JointState

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from utils.color_msg import ColorMsg

class HeadController:
    def __init__(self) -> None:
        self.topic = rospy.get_param("~topic", "/head_hand_cmd")
        self.sub = rospy.Subscriber(self.topic, String, self.command, queue_size=10)
        self.head_loop = False
        self.stop_thread = threading.Event()
        self.loop_thread = None
        ColorMsg(msg="头部控制准备完毕", color="green")

    def command(self, msg):
        data = json.loads(msg.data)
        if data["method"] == "head_motion":  # 头部移动到x,y指定位置
            params = data.get("params", {})
            action = params.get("action", None)
            h_x = params.get("x", None)
            h_y = params.get("y", None)
            try:
                if action != "" and action is not None:
                    if action == "up":
                        # 抬头
                        self.head_start_action(x=0.0, y=0.1, loop=False)
                        ColorMsg(msg="抬头", color="yellow")
                    elif action == "down":
                        # 低头
                        self.head_start_action(x=0.0, y=-0.1, loop=False)
                        ColorMsg(msg="低头", color="yellow")
                    elif action == "left":
                        # 左看
                        self.head_start_action(x=0.5, y=0.0, loop=False)
                        ColorMsg(msg="左看", color="yellow")
                    elif action == "right":
                        # 右看
                        self.head_start_action(x=-0.5, y=0.0, loop=False)
                        ColorMsg(msg="右看", color="yellow")
                    elif action == "loop":
                        # 头部循环移动
                        self.head_start_action(loop=True)
                        ColorMsg(msg="头部循环随机运动", color="yellow")
                    elif action == "stop":
                        self.stop_head_loop()
                    else:
                        ColorMsg(msg="没有相应头部动作", color="red")
                else:
                    x = float(h_x)
                    y = float(h_y)
                    self.head_start_action(x=x, y=y, loop=False)
                    ColorMsg(msg=f"头部运动x:{x} y:{y}", color="yellow")
            except:
                ColorMsg(msg="头部运动参数错误", color="red")
        if data["method"] == "start_speak_motion":  # 开始说话命令
            self.head_start_action(loop=True)
        if data["method"] == "stop_speak_motion":  # 停止说话命令
            self.stop_head_loop()

    # 头部运动
    def head_start_action(self, x=0.0, y=0.0, loop=False):
        pub = rospy.Publisher("/cb_body_control_cmd", JointState, queue_size=10)
        self.head_loop = loop
        if self.head_loop == False:
            count = 0  # 计数器
            while count <= 3:
                pos = [x, y, 0.0,0.0,0.0]
                self.pub_joint(pos=pos,pub=pub)
                time.sleep(0.1)
                count += 1
        else:
            self.stop_thread.clear()
            self.loop_thread = threading.Thread(target=self.head_loop_action, args=(pub,))
            self.loop_thread.start()

    def head_loop_action(self, pub):
        while not self.stop_thread.is_set():
            x = round(random.uniform(-0.5, 0.5), 1)
            y = round(random.uniform(-0.3, 0.1), 1)
            count = 0  # 计数器
            while count <= 3 and not self.stop_thread.is_set():
                pos = [x, y, 0.0,0.0,0.0]
                self.pub_joint(pos=pos,pub=pub)
                time.sleep(0.1)
                count += 1
            if not self.stop_thread.is_set():
                pos = [x, y, 0.0,0.0,0.0]
                self.pub_joint(pos=pos,pub=pub)
            time.sleep(3)
            ColorMsg(msg=f"头部随机运动中{self.head_loop}", color="green")

    def stop_head_loop(self):
        self.stop_thread.set()
        if self.loop_thread is not None:
            self.loop_thread.join()
        self.head_loop = False
        pos = [0.0, 0.0, 0.0,0.0,0.0]
        pub = rospy.Publisher("/cb_body_control_cmd", JointState, queue_size=10)
        self.pub_joint(pos=pos,pub=pub)
        ColorMsg(msg="头部运动停止", color="yellow")

    def pub_joint(self,pos=[],pub=None):
        joint_names = ["joint101","joint102","joint103","joint104","joint105"]
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.frame_id = "world"
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = joint_names
        joint_state.position = pos
        #joint_state.velocity = [2.0, 2.0, 2.0, 2.0, 2.0, 7.168999671936035, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
        joint_state.effort = []
        pub.publish(joint_state)
        ColorMsg(msg="正在播放录制的txt机械臂动作", color="green")
        rospy.loginfo("Publishing: %s", pos)
        time.sleep(0.01)

if __name__ == "__main__":
    rospy.init_node("head_listener", anonymous=True)
    h = HeadController()
    rospy.spin()