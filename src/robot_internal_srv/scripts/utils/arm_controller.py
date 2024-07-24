#! /usr/bin/env python3
import rospy, tf, h5py, ast, rospkg
from datetime import date
import subprocess
import random
import numpy as np
import moveit_commander
import signal, sys, os, json, time
from trajectory_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import Header, String, Float32MultiArray, Float32
from geometry_msgs.msg import TransformStamped, PointStamped
from sensor_msgs.msg import JointState, Image, CameraInfo
from moveit_commander import RobotCommander, MoveGroupCommander
import moveit_commander
from moveit_msgs.msg import PlanningScene, DisplayTrajectory
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_from_euler
from moveit_msgs.msg import RobotState, Constraints, JointConstraint, PositionConstraint, OrientationConstraint

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from color_msg import ColorMsg
# 获取当前脚本文件的目录
current_dir = os.path.dirname(os.path.abspath(__file__))
# 获取上级目录
parent_dir = os.path.dirname(current_dir)
# 将上级目录添加到 sys.path
sys.path.append(parent_dir)

from config import constants

'''


头部移动到指定位置 rosservice call /robot_internal_srv "req: '{\"method\": \"head_motion\", \"id\": 121212, \"params\": {\"action\":\"up|down|left|right|loop|stop\",\"x\":\"-0.5\",\"y\":\"0.1\" }}'"  x:-0.5~0.5   y:-0.3~0.1


头部与手臂同时运动  rosservice call /robot_internal_srv "req: '{\"method\": \"start_speak_motion\", \"id\": 121212, \"params\": {\"loop\":\"True\",\"group_name\":\"leftarm\"}}'"
头部与手臂同时停止  rosservice call /robot_internal_srv "req: '{\"method\": \"stop_speak_motion\", \"id\": 121212, \"params\": {\"loop\":\"True\",\"group_name\":\"leftarm\",\"x\":\"-0.5\",\"y\":\"0.1\" }}'"


rosservice call /robot_internal_srv "req: '{\"method\": \"silent\", \"id\": 121212, \"params\": {}}'"


手臂回归原点  rosservice call /robot_internal_srv "req: '{\"method\": \"arm_to_origin\", \"id\": 121212, \"params\": {}}'"

大数据机械臂动作采集 rosservice call /robot_internal_srv "req: '{\"method\": \"start_collection\", \"id\": 121212, \"params\": {\"file_name\": \"hello\",\"image\":\"True\",\"task_id\": \"asdfasdf\", \"action_id\": \"4\"}}'"

停止大数据采集 rosservice call /robot_internal_srv "req: '{\"method\": \"stop_collection\", \"id\": 121212, \"params\": {\"save_txt\": \"1\",\"file_name\":\"hello\"}}'"


 * 开始录制指定双臂 or 头和腰 or 双手 or 所有关节的动作序列到txt序列文本中 rosservice call /robot_internal_srv "req: '{\"method\": \"start_record_action_to_txt\", \"id\": 121212, \"params\": {\"joint\": [\"arm\",\"body\",\"left_hand\",\"right_hand\"]}}'"  # 说明:arm->双臂 body->头和腰部 left_hand->左手 right_hand->右手

 * 停止录制指定双臂 or 头和腰 or 双手 or 所有关节的动作序列到txt序列文本中 rosservice call /robot_internal_srv "req: '{\"method\": \"stop_record_action_to_txt\", \"id\": 121212, \"params\": {\"file_name\":\"hello\"}}'"   file_name:要保存的文件名称

 * 播放录制指定双臂 or 头和腰 or 双手 or 所有关节的动作序列到txt序列文本 rosservice call /robot_internal_srv "req: '{\"method\": \"play_record_action_to_txt\", \"id\": 121212, \"params\": {\"file_name\":\"hello\"}}'"   file_name:要播放的动作文件的文件名称

 
'''


class RosBagController:
    def __init__(self):
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path("robot_internal_srv")
        self.is_recording = False
        self.recorder_process = None
        self.is_playing = False
        self.player_process = None
        self.head_loop = False  # 头部循环运动控制
        self.joint_sub = None  # 采集训练数据机械臂话题
        self.img_sub = None  # 采集训练数据图像话题
        self.tmp_pos = []
        self.data_dict = {  # 数据采集集合
            '/observations/qpos': [],
            '/observations/qvel': [],
            '/observations/images/top': [],
            '/action': [],
        }
        self.joint_cmd_pub = rospy.Publisher('/cb_joint_cmd', JointState, queue_size=10)
        self.episode_idx = 0
        self.num_episodes = constants.CONFIGS["num_episodes"] # 数据采集次数 默认50
        self.episode_len = constants.CONFIGS["episode_len"]  # 数据文件长度 默认400个数据切片
        self.camera_names = constants.CONFIGS["camera_names"]  # 相机列表
        self.count = 0
        self.arm_sub = None
        self.body_sub = None
        self.l_hand_sub = None
        self.r_hand_sub = None
        self.actions_data = {
            "arm":{
                "name":[],
                "position":[],
                "velocity":[],
                "effort":[]
            },
            "body":{
                "name":[],
                "position":[],
                "velocity":[],
                "effort":[]
            },
            "left_hand":{
                "name":[],
                "position":[],
                "velocity":[],
                "effort":[]
            },
            "right_hand":{
                "name":[],
                "position":[],
                "velocity":[],
                "effort":[]
            }
        } # 用于存储采集到的各个设备关节数据
    ''' ------------------数据采集---------------------- '''
    def get_hdf5(self,action_name="hello_test"):
        # 先播放机械臂动作，仅用作测试
        action_params = {
            "method":"head_arm_hand_cmd",
            "params":{
                "action_name":"hello_test"
            }
        }
        p = rospy.Publisher("/head_hand_cmd", String, queue_size=1)
        data = String()
        data.data = json.dumps(action_params)
        p.publish(data)
        time.sleep(0.1)
        # 开始采集
        self.start_collection(image=True)
        
    ''' ---------------------------------------- '''
    # 设置电机速度
    def set_motor_speed(self, val=0.0):
        value = Float32()
        value.data = val
        pub = rospy.Publisher("/cb_vel_control_cmd",Float32,queue_size=1)
        pub.publish(value)
        ColorMsg(msg="电机速度设置为:"+str(val), color="yellow")


    # 同时采集机械臂与视频数据
    def start_collection(self, image=True):
        if image == True:
            print("h5格式采集数据")
            self.joint_sub = Subscriber('/cb_joint_cmd', JointState)  # 采集时机械臂话题
            self.status_sub = Subscriber('/joint_states', JointState)  # 
            self.img_sub = Subscriber('/camera/color/image_raw', Image)  # 采集时图像话题
            ats = ApproximateTimeSynchronizer([self.joint_sub,self.status_sub, self.img_sub], queue_size=1, slop=0.5)
            ats.registerCallback(self.collection_callback3)
        else:
            print("文本格式数据采集")
            self.joint_sub = Subscriber('/cb_joint_cmd', JointState)  # 采集时机械臂话题
            self.status_sub = Subscriber('/joint_states', JointState)  #
            ats = ApproximateTimeSynchronizer([self.joint_sub,self.status_sub], queue_size=1, slop=3)
            ats.registerCallback(self.collection_callback2)
            #self.joint_sub = rospy.Subscriber("/cb_joint_cmd", JointState, self.collection_callback2, queue_size=10)

    def collection_callback3(self, arm_data, arm_status, img_data):
        print("-----------------------")
        if self.count == self.episode_len:
            print("到了最大采集切片数量，不在进行数据采集")
            return
        print(self.count)
        self.count = self.count + 1
    # 文本采集
    def collection_callback2(self, arm_data,arm_status):
        arm_name_list = arm_data.name
        arm_qvel_list = list(arm_status.position)
        arm_qpos_list = list(arm_data.position)
        self.data_dict['/action'].append(arm_qpos_list)  # TODO::这里应该是一次正确的动作序列，先使用采集数据用于验证
        self.tmp_pos.append(arm_qpos_list)
        self.data_dict['/observations/qpos'].append(arm_qpos_list) # arm_qpos_list所有电机数据
        self.data_dict['/observations/qvel'].append(arm_qvel_list) # arm_qpos_list所有电机数据
        ColorMsg(msg="文本格式数据正在采集中.....", color="green")
    '''  --------------------------1.1.1版本更新后新增的方法，对应columbus-1.1.1版本---------------------------------------  '''
    # * 2024-7-23 底层改版后新增
    # 开始录制指定双臂 or 头和腰 or 双手 or 所有关节的动作序列到txt序列文本中
    def start_record_action_to_txt(self, joint=[]):
        # [arm,body,left_hand,right_hand]
        sub_list = []
        for item in joint:
            if item == "arm":
                self.arm_sub = Subscriber("/cb_arm_control_cmd", JointState)
                sub_list.append(self.arm_sub)
            if item == "body":
                self.body_sub = Subscriber("/cb_body_control_cmd", JointState)
                sub_list.append(self.body_sub)
            if item == "left_hand":
                self.l_hand_sub = Subscriber("/cb_left_hand_control_cmd", JointState)
                sub_list.append(self.l_hand_sub)
            if item == "right_hand":
                self.r_hand_sub = Subscriber("/cb_right_hand_control_cmd", JointState)
                sub_list.append(self.r_hand_sub)
        if len(sub_list) > 0:
            ats = ApproximateTimeSynchronizer(sub_list, queue_size=1, slop=0.3)
            ats.registerCallback(self.joints_record)
        else:
            ColorMsg(msg="要录制的话题列表是空的", color="red")
    def joints_record(self, arm_msg=None,body_msg=None,left_hand_msg=None,right_hand_msg=None):
        tmp = []
        # 手臂数据
        if arm_msg != None: # 双臂数据
            self.actions_data["arm"]["name"].append(arm_msg.name)
            self.actions_data["arm"]["position"].append(arm_msg.position)
            self.actions_data["arm"]["velocity"].append(arm_msg.velocity)
            self.actions_data["arm"]["effort"].append(arm_msg.effort)
            tmp.append("arm")
        if body_msg != None: # 腰部与头部数据
            self.actions_data["body"]["name"].append(body_msg.name)
            self.actions_data["body"]["position"].append(body_msg.position)
            self.actions_data["body"]["velocity"].append(body_msg.velocity)
            self.actions_data["body"]["effort"].append(body_msg.effort)
            tmp.append("body")
        if left_hand_msg != None: # 左手数据
            self.actions_data["left_hand"]["name"].append(left_hand_msg.name)
            self.actions_data["left_hand"]["position"].append(left_hand_msg.position)
            self.actions_data["left_hand"]["velocity"].append(left_hand_msg.velocity)
            self.actions_data["left_hand"]["effort"].append(left_hand_msg.effort)
            tmp.append("left_hand")
        if right_hand_msg != None: # 右手数据
            self.actions_data["right_hand"]["name"].append(right_hand_msg.name)
            self.actions_data["right_hand"]["position"].append(right_hand_msg.position)
            self.actions_data["right_hand"]["velocity"].append(right_hand_msg.velocity)
            self.actions_data["right_hand"]["effort"].append(right_hand_msg.effort)
            tmp.append("right_hand")
        ColorMsg(msg="正在采集"+str(tmp)+"关节数据", color="green")
        time.sleep(0.03)
        

    def stop_record_action_to_txt(self, file_name=""):
        if self.arm_sub != None:
            self.arm_sub.unregister()
        if self.body_sub != None:
            self.body_sub.unregister()
        if self.l_hand_sub != None:
            self.l_hand_sub.unregister()
        if self.r_hand_sub != None:
            self.r_hand_sub.unregister()
        self.save_text(data=self.actions_data, file_name=file_name)
        self.actions_data = {
            "arm":{
                "name":[],
                "position":[],
                "velocity":[],
                "effort":[]
            },
            "body":{
                "name":[],
                "position":[],
                "velocity":[],
                "effort":[]
            },
            "left_hand":{
                "name":[],
                "position":[],
                "velocity":[],
                "effort":[]
            },
            "right_hand":{
                "name":[],
                "position":[],
                "velocity":[],
                "effort":[]
            }
        } # 重置数据
        print("停止录制啦")
    # 最新采集数据保存方法
    def save_text(self, data=None, file_name=""):
        if data!=None:
            today = date.today()
            current_date = today.strftime("%Y-%m-%d")
            dataset_dir = "/home/nx/ROS/robot_internal_srv/action_files/"
            # dataset_dir = "/home/nx/ROS/robot_internal_srv/collection_data/"+str(current_date)
            if file_name != "":
                file_txt = file_name+".txt"
            else:
                #dataset_dir = self.package_path + "/collection_data/"
                file_txt = "pose_"+str(random.randint(1, 999))+".txt"
            dataset_path = os.path.join(dataset_dir, file_txt)
            # 打开文件进行写入操作
            with open(dataset_path, "w") as file:
                json.dump(data, file, indent=4)
            ColorMsg(msg=f"{dataset_path}写入完毕", color="yellow")
    def play_record_action_to_txt(self, file_name=""):
        ColorMsg(msg=f"准备播放{file_name}动作文件", color="yellow")
        # 假设文件名为data.json，并且其内容是一个有效的JSON字符串  
        filename = "/home/nx/ROS/robot_internal_srv/action_files/"+file_name+".txt"
        # 创建两个Publisher，分别发布到不同的话题
        count = 0
        # 读取文件全部内容  
        with open(filename, 'r') as file:  
            file_content = file.read()
            try:  
                data_dict = json.loads(file_content)
                if "arm" in data_dict:
                    arm = data_dict["arm"]
                    if len(data_dict["arm"]["position"]) > 0:
                        count = len(data_dict["arm"]["position"])
                        arm_pub = rospy.Publisher('/cb_arm_control_cmd', JointState, queue_size=1)
                    else:
                        arm = None
                else:
                    arm = None
                if "body" in data_dict:
                    body = data_dict["body"]
                    if len(data_dict["body"]["position"]) > 0:
                        count = len(data_dict["body"]["position"])
                        body_pub = rospy.Publisher('/cb_body_control_cmd', JointState, queue_size=1)
                    else:
                         body = None
                else:
                    body = None
                if "left_hand" in data_dict:
                    left_hand = data_dict["left_hand"]
                    if len(data_dict["left_hand"]["position"]) > 0:
                        count = len(data_dict["left_hand"]["position"])
                        left_hand_pub = rospy.Publisher('/cb_left_hand_control_cmd', JointState, queue_size=1)
                    else:
                        left_hand = None
                else:
                    left_hand = None
                if "right_hand" in data_dict:
                    right_hand = data_dict["right_hand"]
                    if len(data_dict["right_hand"]["position"]) > 0:
                        count = len(data_dict["right_hand"]["position"])
                        right_hand_pub = rospy.Publisher('/cb_right_hand_control_cmd', JointState, queue_size=1)
                    else:
                        right_hand = None
                else:
                    right_hand = None
                
            except json.JSONDecodeError as e:  
                print(f"JSON解析错误: {e}")
            if count > 0:
                print(f"需要循环{count-1}次")
                for i in range(count-1):
                    if arm != None:
                        if i <= len(arm["position"]):
                            self.pub_joint(name=arm["name"][i],pos=arm["position"][i],pub=arm_pub)
                    if body != None:
                        if i <= len(body["position"]):
                            self.pub_joint(name=body["name"][i],pos=body["position"][i],pub=body_pub)
                    if left_hand != None:
                        if i <= len(left_hand["position"]):
                            self.pub_joint(name=left_hand["name"][i],pos=left_hand["position"][i],pub=left_hand_pub)
                    if right_hand != None:
                        if i <= len(right_hand["position"]):
                            self.pub_joint(name=right_hand["name"][i],pos=right_hand["position"][i],pub=right_hand_pub)



    def pub_joint(self,name=[],pos=[],pub=None):
        joint_names = name
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
    '''  --------------------------1.1.1版本更新后新增的方法，对应columbus-1.1.1版本---------------------------------------  '''
    # h5py采集
    def collection_callback(self, arm_data, arm_status, img_data):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")
        arm_name_list = arm_data.name
        tmp_ac_list = list(arm_data.position)
        action = []
        # 左臂
        action.append(tmp_ac_list[0])
        action.append(tmp_ac_list[1])
        action.append(tmp_ac_list[2])
        action.append(tmp_ac_list[3])
        action.append(tmp_ac_list[4])
        action.append(tmp_ac_list[5])
        # 左夹爪
        action.append(tmp_ac_list[12])
        # 右臂
        action.append(tmp_ac_list[6])
        action.append(tmp_ac_list[7])
        action.append(tmp_ac_list[8])
        action.append(tmp_ac_list[9])
        action.append(tmp_ac_list[10])
        action.append(tmp_ac_list[11])
        # 右夹爪
        action.append(tmp_ac_list[18])
        arm_qpos_list = arm_status.position
        arm_qvel_list = arm_status.velocity
        vel = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        ColorMsg(msg="hdf5数据正在采集中....."+str(action), color="green")
        self.data_dict['/action'].append(action)  # 采集前14个电机
        self.tmp_pos.append(action)
        self.data_dict['/observations/qpos'].append(action)
        self.data_dict['/observations/qvel'].append(vel)
        self.data_dict['/observations/images/top'].append(cv_image)
        time.sleep(0.01)


    # 注销话题
    def stop_collection(self, save_txt=0, file_name=""):
        try:
            self.joint_sub.unregister()
            self.img_sub.unregister()
        except:
            pass
        if int(save_txt) == 1:
            self.save_hdf5()
        else:
            self.save_txt(file_name=file_name)

    # 手臂读取文本动作序列进行运动
    def hand_start_action(self, file_name=""):
        if file_name == "":
            timestamp = time.time()
            file_path = self.package_path + "/collection_data/pose_0.txt"
        else:
            file_path = self.package_path + "/collection_data/" + file_name + ".txt"
        # 打开文件进行读取操作
        tmp_l = []
        with open(file_path, 'r') as file:
            for line in file:
                # 去掉行尾的换行符和多余的空格
                line = line.strip()
                # 将字符串转换为浮点数列表
                joint_position = [float(x) for x in line.strip('[]').split(', ')]
                tmp_l.append(joint_position)
        '''
        with open(file_path, "r") as file:
            print(file)
            return
            lines = file.readlines()
            for line in lines:
                tmp = line.strip()
                tmp_l.append(tmp)  # 移除每行的换行符并添加到列表
        # 将字符串转换为列表
        #pose_list = [ast.literal_eval(item) for item in tmp_l]
        '''
        for pos in tmp_l:
            self.publish_cb_joint_cmd(pos=pos, group_name="leftarm")

    # 头部手部同时运动
    def head_hand_start_action(self):
        x = random.uniform(-0.5, 0.5)
        y = random.uniform(-0.3, 0.1)
        self.head_start_action(x=x, y=y, loop=True)  # 头部开始运动
        time.sleep(0.1)
        self.hand_start_action()  # 同时手部开始运动

    # 头部运动
    def head_start_action(self, x=0.0, y=0.0, loop=False):
        data = Float32MultiArray()
        pub = rospy.Publisher("/cb_body_control_cmd", Float32MultiArray, queue_size=10)
        self.head_loop = loop
        if self.head_loop == False:
            count = 0  #计数器
            while True:
                if count > 2:
                    break
                data.data = [0.0, x, y]
                pub.publish(data)
                time.sleep(0.1)
                count += 1
        else:
            while self.head_loop:
                x = random.uniform(-0.5, 0.5)
                y = random.uniform(-0.3, 0.1)
                count = 0  #计数器
                while True:
                    if count > 2:
                        break
                    data.data = [0.0, x, y]
                    pub.publish(data)
                    time.sleep(0.1)
                    count += 1
                time.sleep(1)

    # 将采集到的机械臂动作存储到文本中
    def save_txt(self, file_name=""):
        today = date.today()
        current_date = today.strftime("%Y-%m-%d")
        dataset_dir = "/home/nx/ROS/robot_internal_srv/action_files/"
        # dataset_dir = "/home/nx/ROS/robot_internal_srv/collection_data/"+str(current_date)
        if file_name != "":
            file_txt = file_name+".txt"
        else:
            #dataset_dir = self.package_path + "/collection_data/"
            file_txt = f'pose_{self.episode_idx}.txt'
        dataset_path = os.path.join(dataset_dir, file_txt)
        # 打开文件进行写入操作
        with open(dataset_path, "w") as file:
            for item in self.tmp_pos:
                file.write(f"{item}\n")  # 每个元素写入一行
            ColorMsg(msg=f"{dataset_path}写入完毕", color="yellow")
        self.tmp_pos = []

    def save_hdf5(self):
        max_timesteps = len(self.data_dict['/observations/qpos'])
        t0 = time.time()
        # 获取当前日期
        today = date.today()
        # 格式化输出
        current_date = today.strftime("%Y-%m-%d")
        #dataset_dir = self.package_path + "/collection_data/" + str(current_date)
        dataset_dir = "/home/nx/ROS/robot_internal_srv/collection_data/" + str(current_date)
        dataset_path = os.path.join(dataset_dir, f'episode_{self.episode_idx}')
        with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024 ** 2 * 2) as root:
            root.attrs['sim'] = False
            obs = root.create_group('observations')
            image = obs.create_group('images')
            for cam_name in self.camera_names:
                _ = image.create_dataset(cam_name, (max_timesteps, 360, 640, 3), dtype='uint8',
                                         chunks=(1, 360, 640, 3), )
            qpos = obs.create_dataset('qpos', (max_timesteps, 14))
            qvel = obs.create_dataset('qvel', (max_timesteps, 14))
            action = root.create_dataset('action', (max_timesteps, 14))

            for name, array in self.data_dict.items():
                root[name][...] = array
        self.episode_idx = self.episode_idx + 1
        print(f'Saving: {time.time() - t0:.1f} secs\n')
        print(f"max_timesteps len {max_timesteps}")
        self.data_dict = {  # 重置数据采集集合
            '/observations/qpos': [],
            '/observations/qvel': [],
            '/observations/images/top': [],
            '/action': [],
        }

    # 将采集到的数据存储到.hdf5文件中 暂时放弃
    def save_hdf5_2(self):
        max_timesteps = len(self.data_dict['/observations/qpos'])
        # 获取当前日期
        today = date.today()
        # 格式化输出
        current_date = today.strftime("%Y-%m-%d")
        dataset_dir = self.package_path + "/collection_data/" + str(current_date)
        # 如果路径不存在，则创建路径
        if not os.path.isdir(dataset_dir):
            os.makedirs(dataset_dir, exist_ok=True)
        # HDF5
        t0 = time.time()
        dataset_path = os.path.join(dataset_dir, f'episode_{self.episode_idx}')
        with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024 ** 2 * 2) as root:
            root.attrs['sim'] = True
            obs = root.create_group('observations')
            image = obs.create_group('images')
            for cam_name in self.camera_names:
                _ = image.create_dataset(cam_name, (max_timesteps, 360, 640, 3), dtype='uint8',
                                         chunks=(1, 360, 640, 3))
            
            # 根据 data_dict 中的数据形状动态创建数据集
            for name, array in self.data_dict.items():
                if isinstance(array, list):
                    array = np.array(array)  # 将列表转换为 NumPy 数组
                
                print(f"Creating dataset for {name}: array shape {array.shape}")
                if name not in root:
                    ColorMsg(msg=str(name),color="red")
                    if len(array.shape) == 2:
                        if array.shape[1] == 14:
                            root.create_dataset(name, (max_timesteps, 14), dtype=array.dtype)
                        elif array.shape[1] == 12:
                            root.create_dataset(name, (max_timesteps, 12), dtype=array.dtype)
                        elif array.shape[1] == 29:
                            if name == "/observations/qpos":
                                root.create_dataset(name, (max_timesteps, 29), dtype=array.dtype)
                            else:
                                root.create_dataset(name, (max_timesteps, 17), dtype=array.dtype)
                        else:
                            raise ValueError(f"Unexpected shape for {name}: {array.shape}")
                    elif len(array.shape) == 3:
                        # For example, if it's an image or other type of data with 3 dimensions
                        root.create_dataset(name, (max_timesteps, array.shape[1], array.shape[2]), dtype=array.dtype)
                    else:
                        raise ValueError(f"Unexpected shape for {name}: {array.shape}")
                
                # 打印数据集形状，确认匹配
                print(f"Saving data to {name}: dataset shape {root[name].shape}, array shape {array.shape}")
                root[name][...] = array

        self.episode_idx = self.episode_idx + 1
        print(f'Saving: {time.time() - t0:.1f} secs\n')
        self.data_dict = {  # 重置数据采集集合
            '/observations/qpos': [],
            '/observations/qvel': [],
            '/observations/images/top': [],
            '/action': [],
        }

    def head_stop_action(self):
        self.head_loop = False
        self.head_start_action(x=0, y=0, loop=False)
        print("停止头部随机运动")

    # 录制
    def start_recording(self, file_name, topic, bag_path=""):
        self.stop_playing()
        if not self.is_recording:
            file = file_name
            self.recorder_process = subprocess.Popen(["rosbag", "record", "-O", file, topic])
            self.is_recording = True
            rospy.loginfo(f"正在以rosbag录制,文件名为{file_name}.bag")

    def stop_recording(self):
        self.stop_playing()
        if self.is_recording:
            self.recorder_process.send_signal(signal.SIGINT)
            self.recorder_process.wait()
            self.is_recording = False
            rospy.loginfo("停止rosbag录制")

    def start_playing(self, bag_file):
        self.stop_playing()
        if not self.is_playing:
            # rospy.loginfo(f"正在播放bag文件: {bag_file}")
            # output = subprocess.check_output(["rosbag", "play", bag_file])  
            # decoded_output = output.decode('utf-8')  
            # success_msg = "Done"
            # index = decoded_output.find(success_msg)
            # if index != -1:
            #     self.stop_playing()
            # rospy.loginfo(f"{bag_file}播放完毕")

            self.player_process = subprocess.Popen(["rosbag", "play", bag_file])
            self.is_playing = True
            rospy.loginfo(f"正在播放bag文件: {bag_file}")

    def stop_playing(self):
        time.sleep(0.1)
        if self.is_playing:
            self.player_process.send_signal(signal.SIGINT)
            self.player_process.wait()
            self.is_playing = False
            rospy.loginfo("停止播放bag文件")

    # 获取所有rosnode名称
    def get_all_nodes(self):
        """使用rosnode list命令获取所有节点的名称"""
        try:
            # 使用subprocess调用rosnode list命令  
            result = subprocess.check_output(["rosnode", "list"], stderr=subprocess.STDOUT)
            # 将输出从字节串解码为字符串  
            nodes = result.decode('utf-8').strip().split('\n')
            return nodes
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Failed to get list of nodes: {e}")
            return []

    # 发布机械臂消息
    def publish_cb_joint_cmd(self, pos=[], group_name="leftarm"):
        if len(pos) < 29:
            ColorMsg(msg="机器人动作序列数据不正确", color="red")
            return
        # 定义关节名称和动作序列
        #joint_names = ["joint11", "joint12", "joint13", "joint14", "joint15", "joint16", "joint21", "joint22","joint23", "joint24", "joint25", "joint26"]
        joint_names = ["joint1", "joint12", "joint13", "joint14", "joint15", "joint16", "joint21", "joint22", "joint23", "joint24", "joint25", "joint26", "joint41", "joint42", "joint43", "joint44", "joint45", "joint46", "joint51", "joint52", "joint53", "joint54", "joint55", "joint56", "joint61", "joint62", "joint63", "joint64", "joint65"]
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
        time.sleep(0.01)


    # 发布机械臂消息
    def publish_cb_joint_cmd_2(self,pos=[],group_name="leftarm"):
        tmp = [0.0,0.0,0.0,0.0,0.0,0.0]
        if len(pos) > 0 and len(pos) < 12:
            if group_name == "leftarm":
                pos = pos+tmp
            else:
                pos = tmp+pos
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
        #time.sleep(0.06)


class RosTxtController():
    def __init__(self) -> None:
        self.positions = []

    # 写入文件
    def positionToFile(self, positions):
        file = str(self.params) + ".txt"
        # 打开文件以写入模式  
        with open(file, 'w', encoding='utf-8') as f:
            # 遍历列表中的每个元素  
            for item in positions:
                # 将元素写入文件，并在每个元素后添加一个换行符  
                f.write("%s\n" % json.dumps(item))
                ColorMsg(msg="--------------成功写入一条信息-----------------", color="yellow")
                print(json.dumps(item))
        ColorMsg(msg="文件写入成功", color="green")


    def readFile(self, file_name):
        data = []
        with open(file_name + ".txt", 'r', encoding='utf-8') as file:
            # 读取文件全部内容  
            # data = file.read()
            for line in file:
                data.append(line)
        return data

    def pubJointMsg(self, positions):
        if self.is_simulation == True:
            # 仿真机械臂
            joint_state = JointState()
            joint_state.header = Header()
            joint_state.header.frame_id = "world"
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = positions["names"]
            joint_state.position = positions["position"]
            self.pub.publish(joint_state)
            time.sleep(0.1)
            ColorMsg(msg="机械臂发布功能已开启，当前为仿真机械臂", color="green")
        else:
            # 真实机械臂
            # 创建轨迹消息
            trajectory_msg = JointTrajectory()
            trajectory_msg.header = Header()
            trajectory_msg.header.frame_id = "world"
            trajectory_msg.header.stamp = rospy.Time.now()
            trajectory_msg.joint_names = positions["names"]
            # 创建轨迹点
            joint_trajectory_point = JointTrajectoryPoint()
            # joint_trajectory_point.velocities = self.v
            # joint_trajectory_point.accelerations = self.v
            joint_trajectory_point.positions = positions["position"]
            joint_trajectory_point.time_from_start = rospy.Duration(2)
            trajectory_msg.points.append(joint_trajectory_point)
            # 发布轨迹消息
            self.pub.publish(trajectory_msg)
            time.sleep(0.1)
            ColorMsg(msg="机械臂发布功能已开启，当前为真实机械臂", color="green")


class ArmController:
    def __init__(self) -> None:
        # 暂时不用
        # self.arm_pub = rospy.Publisher("/cb_joint_cmd", JointState, queue_size=1)
        self.robot = RobotCommander()
        self.move_group = None
        self.current_state = RobotState()
        self.current_state.joint_state.name = None
        self.current_state.joint_state.position = [0, 0, 0, 0, 0, 0]
        try:
            self.joint_goal = self.current_state.joint_state.position[:]
        except:
            ColorMsg(msg="配置错误，不用理会", color="yellow")
        # self.yolo_sub = rospy.Subscriber("/yolov8_distance", String, self.yolo_cb)
        # self.joint_exec(group_name="leftarm")

    def move_to_position(self, x, y, z):
        # 初始化ROS节点
        #rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

        # 初始化MoveIt的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 创建MoveGroupCommander对象
        group_name = "leftarm"  # 根据你的机械臂配置文件中的命名
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # 创建一个目标位置对象
        pose_target = geometry_msgs.msg.Pose()

        # 设置目标位置的x, y, z坐标
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z

        # 这里可以设置目标的方向（四元数表示），例如：
        pose_target.orientation.w = 1.0
        pose_target.orientation.x = 0.0
        pose_target.orientation.y = 0.0
        pose_target.orientation.z = 0.0

        # 设置目标位置
        move_group.set_pose_target(pose_target)

        # 规划运动
        plan = move_group.go(wait=True)

        # 停止运动
        move_group.stop()

        # 清除目标位置
        move_group.clear_pose_targets()

    # 规划轨迹并且执行动作
    def joint_exec(self, group_name="leftarm", x=0.0, y=0.0, z=0.0):
        self.move_group = MoveGroupCommander(group_name)
        self.current_state.joint_state.name = self.robot.get_joint_names(group_name)
        # 0.34,0.11,0.2,0,0,0
        self.joint_goal = [x, y, z, 0, 0, 0]  # 假设这是6个关节的目标值

        # 创建运动规划请求  
        self.move_group.set_joint_value_target(self.joint_goal)
        # 尝试进行运动规划  
        plan = self.move_group.plan()  # 接收规划结果
        print(plan)
        # 检查规划是否成功  
        if plan is not None:
            # 如果规划成功，则执行它（不需要传递plan给go方法）  
            success = self.move_group.go(wait=True)
            if not success:
                rospy.logerr("规划执行失败！")
                return False
            else:
                rospy.loginfo("规划执行成功！")
                return True
        else:
            rospy.logerr("规划失败，没有找到路径！")
            return False

    # 规划轨迹并且执行动作
    def joint_exec_2(self, group_name="leftarm", x=0.0, y=0.0, z=0.0):
        # 初始化MoveGroupInterface  
        move_group = moveit_commander.MoveGroupCommander(group_name)  # 替换为你的move_group名称  

        # 设置目标姿态（这里只是一个示例，你需要根据实际情况设置）  
        target_pose = PoseStamped()
        target_pose.header.frame_id = move_group.get_planning_frame()  # 通常是机器人的base_link或类似的东西  
        target_pose.pose.position.x = x  # 替换为你的x坐标  
        target_pose.pose.position.y = y  # 替换为你的y坐标  
        target_pose.pose.position.z = z  # 替换为你的z坐标  
        target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w = quaternion_from_euler(
            0, 0, 1)  # 设置为某个欧拉角，例如这里为0, 0, 0

        # 发送目标姿态到MoveGroupInterface  
        move_group.set_pose_target(target_pose)
        # 尝试规划路径
        plan = move_group.plan()[0]
        # 检查规划是否成功  
        if plan:
            # 如果规划成功，可以执行该计划（这里只是获取关节角度，所以不执行）  
            move_group.execute(plan)

            # 从计划中获取关节角度  
            joint_trajectory = plan.joint_trajectory
            for point in joint_trajectory.points:
                print("Joint angles at this point:", point.positions)

        else:
            rospy.logerr("Could not compute plan successfully")

    def move_to_pose(self, x, y, z):
        # 定义目标位姿
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = x  # 目标x坐标
        target_pose.position.y = y  # 目标y坐标
        target_pose.position.z = z  # 目标z坐标

        # 假设方向使用四元数表示
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 0.0
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 1.0
        # 初始化MoveIt
        moveit_commander.roscpp_initialize(sys.argv)

        # 创建一个MoveGroupCommander对象
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("leftarm")

        # 设置目标位姿
        group.set_pose_target(target_pose)

        # 规划路径
        plan = group.plan()
        print(plan)
        return
        if not plan.joint_trajectory.points:
            rospy.loginfo("No plan found!")
            return

        # 执行路径
        group.execute(plan, wait=True)

        # 清理
        moveit_commander.roscpp_shutdown()

    # 规划轨迹并且返回轨迹序列
    def joint_plan(self, group_name="leftarm", x=0.0, y=0.0, z=0.0):
        self.move_group = MoveGroupCommander(group_name)
        self.current_state.joint_state.name = self.robot.get_joint_names(group_name)
        self.joint_goal = [x, y, z, 0, 0, 0]  # 假设这是6个关节的目标值
        #self.joint_goal = [0.002953, -0.151555, -0.478596, 1.891225, 1.564646, 0.317977]
        # 创建运动规划请求  
        self.move_group.set_joint_value_target(self.joint_goal)
        # 进行运动规划但不执行  
        plan = self.move_group.plan()
        # 检查规划是否成功  
        if plan[1].joint_trajectory.points:
            print("Path planning succeeded!")
            # 这里您可以进一步处理计划得到的轨迹点，比如保存到文件或进行其他分析  
            # 例如，打印出轨迹点的位置
            tmp_list = list()

            j = plan[1].joint_trajectory.points
            # for point in plan[1].joint_trajectory.points:
            for point in j:
                arm = {}
                arm["positions"] = list(point.positions)
                arm["velocities"] = list(point.velocities)
                arm["accelerations"] = list(point.accelerations)
                arm["effort"] = list(point.effort)
                tmp_list.append(arm)
        else:
            print("Path planning failed!")
        return tmp_list

    def yolo_cb(self, data):
        tmp_j = json.loads(data.data)
        print(tmp_j)

    def run(self, group_name, class_name):
        ColorMsg(msg=f"获取视觉识别，目标{class_name}", color="yellow")
        rs = rospy.wait_for_message("/yolov8_distance", String, timeout=2)
        if len(rs.data) == 0:
            ColorMsg(msg="没有识别到任何物品", color="red")
            return False
        else:
            tmp = 0
            for item in json.loads(rs.data):
                if item["name"] == class_name:
                    ColorMsg(msg=f"识别到目标{class_name}", color="green")
                    print(item)
                    # 坐标转换
                    base_point = self.eye_to_hand(group_name=group_name, x=item["coordinate"]["x"],
                                                  y=item["coordinate"]["y"], z=item["coordinate"]["z"])
                    tmp = 1
                    print(base_point)
                    return base_point
            if tmp == 0:
                ColorMsg(msg=f"未识别到{class_name}的物品", color="yellow")
                return False

    def eye_to_hand(self, group_name, x, y, z):
        listener = tf.TransformListener()
        # 假设我们知道源坐标系和目标坐标系  
        source_frame = "/camera_link"  # 相机坐标系  
        target_frame = "/base_link"  # 机器人基坐标系

        # 等待tf监听器有足够的数据进行转换  
        rospy.sleep(5)

        # 现在我们查询一个点（假设我们知道这个点在相机坐标系下的坐标）  
        camera_point = PointStamped()
        camera_point.header.frame_id = source_frame
        camera_point.header.stamp = rospy.Time.now()  # 或者使用你实际的时间戳  
        camera_point.point.x = x
        camera_point.point.y = y
        camera_point.point.z = z

        try:
            # 使用listener的transformPoint方法进行转换  
            base_point = listener.transformPoint(target_frame, camera_point)
            print("Point in base frame: ", base_point)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("Error transforming point: ", e)
        self.joint_exec(group_name=group_name, x=base_point.point.x, y=base_point.point.y, z=base_point.point.z)
        return base_point

    # 暂时不要用
    # def init_point(self):
    #     p_name = ["joint11","joint12","joint13","joint14","joint15","joint16","joint21","joint22","joint23","joint24","joint25","joint26"]
    #     positions = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    #     joint_state = JointState()
    #     joint_state.header = Header()
    #     joint_state.header.frame_id = "world"
    #     joint_state.header.stamp = rospy.Time.now()
    #     joint_state.name = p_name
    #     joint_state.position = positions
    #     joint_state.velocity = [2.0, 2.0, 2.0, 2.0, 2.0, 7.168999671936035, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
    #     joint_state.effort = []
    #     self.arm_pub.publish(joint_state)
