#! /usr/bin/env python3
import os,sys,time,rospy, json,ast, threading
import numpy as np
from trajectory_msgs.msg import *
from std_msgs.msg import Header, String
from sensor_msgs.msg import JointState
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from utils.color_msg import ColorMsg
'''
手部与机械臂运动控制
执行录制动作 rostopic pub /head_hand_cmd std_msgs/String "data: '{\"method\":\"head_arm_hand_cmd\",\"params\":{\"action_name\":\"hello_test\"}}'" 
'''
class HandArmController:
    def __init__(self) -> None:
        self.head_arm_topic = rospy.get_param("~head_arm_sub", "/head_hand_cmd")
        self.joint_cmd_topic = rospy.get_param("~joint_cmd_pub", "/cb_joint_cmd")
        self.head_arm_sub = rospy.Subscriber(self.head_arm_topic,String,self.command,queue_size=10) # 监听手部头部运行命令话题
        self.joint_cmd_pub = rospy.Publisher(self.joint_cmd_topic, JointState, queue_size=10) # 发布机械臂与手部运动命令话题
        self.head_hand_pub = rospy.Publisher("/head_hand_cmd", String,queue_size=10)
        self.head_loop = False # 是否循环执行动作
        self.stop_thread = threading.Event() # 循环动作线程
        self.loop_thread = None # 是否在循环
        self.group_name = "leftarm"  # 机械臂组名称
        # 获取当前脚本的目录  
        self.current_dir = os.path.dirname(os.path.abspath(__file__))  
        # 构造上一级目录的路径  
        self.parent_dir = os.path.abspath(os.path.join(self.current_dir, '..'))
        ColorMsg(msg="机械臂与机械手控制准备完毕", color="green")

    def hand_arm_loop_action(self):
        while not self.stop_thread.is_set():
            self.hand_start_action()
            ColorMsg(msg=f"静默呼吸动作持续运行中.....", color="green")
    
    def stop_hand_arm_loop_action(self):
        self.stop_thread.set()
        if self.loop_thread is not None:
            self.loop_thread.join()
        self.head_loop = False
        ColorMsg(msg="静默呼吸动作停止", color="yellow")

    def command(self, msg):
        data = json.loads(msg.data)
        if data["method"] == "head_arm_hand_cmd":
            try:
                params = data["params"]
            except:
                ColorMsg(msg="缺少参数",color="red")
            cmd = params["action_name"]
            self.hand_start_action(file_name=cmd)
        if data["method"] == "silent":
            self.hand_arm_loop_action()
        if data["method"] == "start_speak_motion":
            params = data.get("params", {})
            self.group_name = params.get("group_name", None)
            ColorMsg(msg="开始机械臂运动", color="green")
            if self.group_name == None:
                ColorMsg(msg="没有指定group_name，禁止运行", color="red")
            else:
                ac = self.hand_start_action()
                if ac == True:
                    time.sleep(0.5)
                    self.arm_init_point()
                    d = {
                        "method": "stop_speak_motion",
                    }
                    s = String()
                    s.data = json.dumps(d)
                    self.head_hand_pub.publish(s)

        if data["method"] == "arm_to_origin":
            ColorMsg(msg="手臂回归原点", color="yellow")
            self.arm_init_point()
            
    
    # 手臂读取文本动作序列进行运动
    def hand_start_action(self,file_name="hello_test"):
        file_path = self.parent_dir+"/"+file_name+".txt"
        # 打开文件进行读取操作
        tmp_l= []
        with open(file_path, 'r') as file:
            for line in file:
                # 去掉行尾的换行符和多余的空格
                line = line.strip()
                # 将字符串转换为浮点数列表
                joint_position = [float(x) for x in line.strip('[]').split(', ')]
                if file_name=="hello_test":
                    tmp_l.append(joint_position[:12])
                else:
                    tmp_l.append(joint_position)
        for pos in tmp_l:
            self.publish_cb_joint_cmd(pos=pos,group_name="leftarm")
        ColorMsg(msg="机械臂动作执行完毕", color="red")
        return True
            

    # 发布机械臂消息, 这里只发布机械臂的关节角度数据
    def publish_cb_joint_cmd(self,pos=[],group_name="leftarm"):
        if len(pos) != 12:
            ColorMsg(msg="机械臂数据列表元素个数不是12，数据格式不正确", color="red")
            print(pos)
            return
        # 定义关节名称和动作序列
        joint_names = ["joint11","joint12","joint13","joint14","joint15","joint16","joint21","joint22","joint23","joint24","joint25","joint26"]
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.frame_id = "world"
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = joint_names
        joint_state.position = pos
        #joint_state.velocity = [2.0, 2.0, 2.0, 2.0, 2.0, 7.168999671936035, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
        joint_state.effort = []
        self.joint_cmd_pub.publish(joint_state)
        ColorMsg(msg="正在播放录制的txt机械臂动作", color="green")
        rospy.loginfo("Publishing: %s", pos)
        time.sleep(0.02)

    # 机械臂回归原点
    def arm_init_point(self):            
        p_name = ["joint11","joint12","joint13","joint14","joint15","joint16","joint21","joint22","joint23","joint24","joint25","joint26"]
        positions = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.frame_id = "world"
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = p_name
        joint_state.position = positions
        joint_state.velocity = [2.0, 2.0, 2.0, 2.0, 2.0, 7.168999671936035, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
        joint_state.effort = []
        self.joint_cmd_pub.publish(joint_state)

if __name__ == "__main__":
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("hand_listener", anonymous=True)
    #3.实例化 订阅者 对象
    h = HandArmController()
    #4.处理订阅的消息(回调函数)
    #5.设置循环调用回调函数
    rospy.spin()