#! /usr/bin/env python3
import rospy, sys, os, yaml, json,time, glob,threading, subprocess, ast
import random
from trajectory_msgs.msg import *
from std_msgs.msg import Header, String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from robot_internal_srv.srv import internal,internalRequest, internalResponse
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from utils.chassis_controller import ChassisController
from utils.arm_controller import RosBagController, ArmController
from utils.color_msg import ColorMsg
from utils.socket_client import SocketClient


class RobotInternalServer:
    def __init__(self) -> None:
        self.is_chassis = self.ping(host="192.168.111.1",device="底盘") # 判断底盘是否存在
        if self.is_chassis == True:
            '''  没有底盘，需要注销这两行，否则无法运行  '''
            self.chassis_controller = ChassisController()
            p, http_code = self.chassis_controller.get_all_pois()
            self.all_pois = p # 记录所有点位
            self.chassis_controller.set_speed(value="0.5")  # 开机默认设置速度
            ColorMsg(msg="底盘服务已启动", color="green")
        # 机械臂抓取
        self.arm_controller = ArmController()
        self.arm_action = [] # 保存现有的动作bag文件
        
        # 录制 or 播放 机械臂话题
        self.joint_topic = rospy.get_param("~joint_topic", "/joint_states")
        self.ros_bag_controller = RosBagController()
        # 是否为仿真环境
        #self.is_simulation = rospy.get_param("~is_simulation", False)
        self.server = rospy.Service("robot_internal_srv", internal, self.server_cb)
        # 发布机械臂话题
        self.pub = rospy.Publisher(self.joint_topic, JointTrajectory, queue_size=10)

        self.head_hand_pub = rospy.Publisher("/head_hand_cmd", String,queue_size=10)

        self.is_record_loop = True
        self.is_play_loop = True
        self.params = ""
        self.data = {}
        self.data["id"] = 0
        self.data["msg"] = ""
        self.data["code"] = -1
        self.data["params"] = {}
        '''  线程内发布信息参数  '''
        self.thread_method = ""
        self.thread_id = 0
        self.thread_task_id = 0
        self.positions = []
        # 获取当前脚本的目录  
        self.current_dir = os.path.dirname(os.path.abspath(__file__))  
        # 构造上一级目录的路径  
        self.parent_dir = os.path.abspath(os.path.join(self.current_dir, '..'))
        
        self.getAllFile(ext=".bag")
        ColorMsg(msg="机械臂录制/发布服务已启动", color="green")
        
        self.pub = rospy.Publisher("/engine_control_cmd", String, queue_size=1)
        # self.action_factories = self.chassis_controller.get_action_factories()  暂时无用
        self.thread = None  
        #self.data = None  # 初始化数据，这里可以是一个可变对象，比如列表、字典等  
        self.lock = threading.Lock()  # 创建锁来同步访问数据
        self.action_id = 0
        self.pub_id = 0
        self.rs = {}
        # 语音播放
        self.speak_pub = rospy.Publisher('/app_control_cmd', String, queue_size=1)
        self.speak_msg = String()
        #self.robot_ready() # 准备就绪，发布语音提示
        if self.is_chassis == True:
            self.start_thread() # 开启新线程，监听底盘状态


    def ping(self, host="192.168.11.1",device="底盘"):
        try:
            # 使用subprocess.run来执行ping命令，-c 1表示只发送一个包，timeout表示超时时间
            result = subprocess.run(['ping', '-c', '1', host], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
            # 检查返回码，如果为0则表示ping成功
            if result.returncode == 0:
                return True
            else:
                return False
        except subprocess.TimeoutExpired:
            # 如果超时则返回False
            ColorMsg(msg=f"{device}设备通讯失败，设备不存在", color="red")
            return False
        except Exception as e:
            # 处理其他可能的异常
            print(f"An error occurred: {e}")
            ColorMsg(msg=f"{device}设备通讯失败，设备不存在", color="red")
            return False
    # 准备就绪，播放语音提示 
    def robot_ready(self):
        time.sleep(3)
        self.speak_msg.data = '{"method":"start_speak", "id":121212, "params":"我已准备就绪，等待您的指令"}'
        self.speak_pub.publish(self.speak_msg)


    def start_thread(self):
        ColorMsg(msg="底盘监听线程已开启", color="green")
        def thread_function():  
            while True:  
                with self.lock:  # 使用锁来确保安全地访问数据  
                    if self.action_id is not None:  
                        #print(f"线程接收到数据: {self.action_id}")
                        # res为元组类型
                        res = self.chassis_controller.get_current_action_by_id(action_id=self.action_id)

                        if res != None:
                            if res[0]["action_id"] == self.action_id and res[0]["state"]["status"] == 4:
                                if res[0]["state"]["result"] == -1:
                                    if self.tag != "":
                                        ColorMsg(msg=f"action_id:{self.action_id} 移动点位任务失败", color="red")
                                        self.speak_msg.data = '{"method":"start_speak", "id":121212, "params":"'+self.tag+'移动失败"}'
                                        self.speak_pub.publish(self.speak_msg)
                                        self.tag = ""
                                else:
                                    if self.tag != "":
                                        ColorMsg(msg=f"action_id:{self.action_id} 移动点位任务完成", color="green")
                                        self.speak_msg.data = '{"method":"start_speak", "id":121212, "params":"我已到达'+self.tag+'点位"}'
                                        self.speak_pub.publish(self.speak_msg)
                                        self.tag = ""
                                if self.pub_id != self.action_id:
                                    data = String()
                                    # 构造JSON消息
                                    if res[0]["state"]["result"] == 0:
                                        msg = "success"
                                    else:
                                        msg = "error"
                                    message_json = {
                                        "method": "notify_navi_result",
                                        "id": self.thread_id,
                                        "params": {
                                            "task_id": self.action_id,
                                            "code": res[0]["state"]["result"],
                                            "msg": msg
                                        }
                                    }
                                    message_str = json.dumps(message_json)
                                    data.data = message_str
                                    self.pub.publish(data)
                                    self.pub_id = self.action_id
                                else:
                                    print(f"action_id:{self.action_id} 信息已经发布过.....")
                            elif res[0]["action_id"] == self.action_id and res[0]["state"]["status"] == 1:
                                p1,http_code = self.chassis_controller.get_status()
                                p2, http_code = self.chassis_controller.get_current_pose_quality()
                                
                                ColorMsg(msg="action_id:"+str(self.action_id)+" 移动点位任务正在移动中..... 当前电量: "+str(p1["batteryPercentage"])+" 当前定位质量:"+str(p2), color="yellow")

                        # 处理数据...  
                    else:  
                        print("线程已结束请按：Ctrl+C终止程序运行...")
                        break
                time.sleep(1)
        self.thread = threading.Thread(target=thread_function)  
        self.thread.start()

    def update_id(self, new_id):  
        with self.lock:  # 使用锁来确保安全地更新数据  
            self.action_id = new_id
  
    def stop_thread(self):  
        if self.thread:  
            self.update_id(None)  # 发送一个信号让线程退出循环
            self.thread.join() 

    def getAllFile(self, ext):
        txt_file = glob.glob(os.path.join("/home/nx/ROS/robot_internal_srv/action_files", '*.txt'))
        tmp = []
        for f in txt_file:
            file_name, file_ext = os.path.splitext(f)
            name = os.path.basename(file_name)
            tmp.append(name)
        self.arm_action = tmp
        ColorMsg(msg="当前存在的动作有:"+str(self.arm_action), color="yellow")


    def server_cb(self,req):
        tmp_data = {}
        json_key2 = "params"
        #try:
        result = json.loads(req.req)
        # print(result["method"])
        self.data["method"] = result["method"]
        
        try:
            self.data["params"]["task_id"] =result["params"]["task_id"]
            file_name = result["params"]["name"]
        except:
            pass
        self.data["id"] = result["id"]
        self.data["timestamp"] = time.time()
        self.data["ros_time"] = rospy.Time.now().to_sec()
        # 判断参数是否正确
        if result.get("method") is None or result.get("id") is None or result.get("params") is None:
            self.data["msg"] = "缺少必要参数，请查看手册"
            ColorMsg(msg=self.data["msg"], color="red")
            resp = internalResponse(json.dumps(self.data, ensure_ascii=False))
            return resp
        ''' ---------------------------以下为升降杆控制-------------------------------- '''
        if result["method"] == "lifter":
            range = result["params"]["range"]  # 单位mm
            socket_client = SocketClient()
            result = socket_client.send(range=range)
            ColorMsg(msg="升降杆开始移动", color="green")
            resp = internalResponse(result)
            return resp
        ''' ---------------------------以下为机械臂控制-------------------------------- '''
        
        if result["method"] == "arm_init_point": # 将机械臂归位到初始点位
            ColorMsg(msg="暂时不能用", color="yellow")
            self.data["id"] = result["id"]
            self.data["params"] = result["params"]
            self.data["code"] = 0
            self.data["msg"] = "OK"
            resp = internalResponse(json.dumps(self.data, ensure_ascii=False))
            return resp
        if result["method"] == "arm_grab": # 机械臂抓取 TODO::暂时不做了，moveit目前不能使用
            self.arm_controller.move_to_position(x=0.1,y=0.1,z=0.1)
            self.data["id"] = result["id"]
            self.data["params"] = result["params"]
            self.data["code"] = 0
            self.data["msg"] = "OK"
            resp = internalResponse(json.dumps(self.data, ensure_ascii=False))
            return resp
        '''
        if result["method"] == "arm_grab": # 机械臂抓取
            # a_x=0.1 # 左右
            # a_y=0.1 # 上下？？？
            # a_z=0.1 # 远近
            # #self.arm_controller.move_to_pose(a_x,a_y,a_z)
            # self.arm_controller.eye_to_hand(group_name="rightarm",x=a_x,y=a_y,z=a_z)
            # return
            try:
                group_name = result["params"]["group_name"] # 机械臂组名称
                class_name = result["params"]["class_name"] # 要抓取的物品在yolo中的class_name
                rs = self.arm_controller.run(group_name=group_name, class_name=class_name)
                self.data["id"] = result["id"]
                self.data["params"] = result["params"]
                self.data["code"] = 0
                self.data["msg"] = "OK"
            except:
                self.data["id"] = result["id"]
                self.data["params"] = "null"
                self.data["code"] = -1
                self.data["msg"] = "参数错误"
            resp = internalResponse(json.dumps(self.data, ensure_ascii=False))
            if rs == False:
                self.speak_msg.data = '{"method":"start_speak", "id":121212, "params":"未识别到目标'+class_name+'"}'
                self.speak_pub.publish(self.speak_msg)
                
            else:
                #self.arm_controller.joint_exec(group_name=group_name,x=rs.point.x,y=rs.point.y,z=rs.point.z)
                self.arm_controller.joint_exec(group_name=group_name,x=rs.point.z,y=rs.point.x,z=rs.point.y)
            
            return resp
        '''
        # -----------------------robot2 仿真头控制-------------------------
        if result["method"] == "columbus_head": # 头部运动
            '''
            只有一台机器人使用
            rosservice call /robot_internal_srv "req: '{\"method\": \"columbus_head\", \"id\": 121212, \"params\": {\"action\":\"talk|stop|silent\"}}'"  说话|停止|静默
            '''
            ColorMsg(msg="仿真头控制命令columbus_head", color="green")
            s = String()
            try:
                s.data = json.dumps(result["params"])
                ch_pub = rospy.Publisher("/columbus_head/command",String,queue_size=10)
                ch_pub.publish(s)
                self.data["id"] = result["id"]
                self.data["params"] = result["params"]
                self.data["code"] = 0
                self.data["msg"] = "OK"
                resp = internalResponse(json.dumps(self.data, ensure_ascii=False))
                return resp
            except:
                ColorMsg(msg="参数不正确", color="red")
                self.data["id"] = result["id"]
                self.data["params"] = result["params"]
                self.data["code"] = 0
                self.data["msg"] = "error"
                resp = internalResponse(json.dumps(self.data, ensure_ascii=False))
                return resp
        # -----------------------robot2 仿真头控制-------------------------
        '''
        -------------以下为手部&手臂控制----------------
        '''
        if result["method"] == "head_arm_hand_cmd":
            ColorMsg(msg="收到头部控制命令head_motion", color="green")
            s = String()
            s.data = json.dumps(result)
            self.head_hand_pub.publish(s)
            self.data["id"] = result["id"]
            self.data["params"] = result["params"]
            self.data["code"] = 0
            self.data["msg"] = "OK"
            resp = internalResponse(json.dumps(self.data, ensure_ascii=False))
            return resp
        if result["method"] == "head_motion": # 头部运动
            ColorMsg(msg="收到头部控制命令head_motion", color="green")
            s = String()
            s.data = json.dumps(result)
            self.head_hand_pub.publish(s)
            self.data["id"] = result["id"]
            self.data["params"] = result["params"]
            self.data["code"] = 0
            self.data["msg"] = "OK"
            resp = internalResponse(json.dumps(self.data, ensure_ascii=False))
            return resp
        if result["method"] == "start_speak_motion": # 开始说话并执行相应动作
            s = String()
            s.data = json.dumps(result)
            self.head_hand_pub.publish(s)
            self.data["id"] = result["id"]
            self.data["params"] = result["params"]
            self.data["code"] = 0
            self.data["msg"] = "OK"
            resp = internalResponse(json.dumps(self.data, ensure_ascii=False))
            return resp
        if result["method"] == "stop_speak_motion": # 立即停止说话的动作并且头部和手臂回归原点
            s = String()
            s.data = json.dumps(result)
            self.head_hand_pub.publish(s)
            self.data["id"] = result["id"]
            self.data["params"] = result["params"]
            self.data["code"] = 0
            self.data["msg"] = "OK"
            resp = internalResponse(json.dumps(self.data, ensure_ascii=False))
            return resp
        if result["method"] == "arm_to_origin": # 手臂回归原点
            #self.ros_bag_controller.head_hand_start_action()
            s = String()
            s.data = json.dumps(result)
            self.head_hand_pub.publish(s)
            self.data["id"] = result["id"]
            self.data["params"] = result["params"]
            self.data["code"] = 0
            self.data["msg"] = "OK"
            resp = internalResponse(json.dumps(self.data, ensure_ascii=False))
            return resp
        if result["method"] == "start_record_action_to_txt": # 开始录制指定双臂 or 头和腰 or 双手 or 所有关节的动作序列到txt序列文本中

            try:
                joint = ast.literal_eval(repr(result["params"]["joint"]))
                self.data["id"] = result["id"]
                self.data["params"] = result["params"]
                if isinstance(joint, list):  
                    self.data["code"] = 0
                    self.data["msg"] = "OK"
                    self.ros_bag_controller.start_record_action_to_txt(joint=joint)
                else:
                    ColorMsg(msg="joint参数必须为list格式")
                    self.data["code"] = -1
                    self.data["msg"] = "error"
            except:
                ColorMsg(msg="录制指令参数不正确", color="red")
                self.data["id"] = result["id"]
                self.data["params"] = result["params"]
                self.data["code"] = -1
                self.data["msg"] = "error"
            resp = internalResponse(json.dumps(self.data, ensure_ascii=False))
            return resp
        if result["method"] == "stop_record_action_to_txt":  # 停止录制指定双臂 or 头和腰 or 双手 or 所有关节的动作序列到txt序列文本中
            try:
                file_name = result["params"]["file_name"]
            except:
                ColorMsg(msg="参数不正确", color="red")
                self.data["id"] = result["id"]
                self.data["params"] = result["params"]
                self.data["code"] = -1
                self.data["msg"] = "error"
                resp = internalResponse(json.dumps(self.data, ensure_ascii=False))
                return resp
            self.data["id"] = result["id"]
            self.data["params"] = result["params"]
            self.data["code"] = 0
            self.data["msg"] = "OK"
            self.ros_bag_controller.stop_record_action_to_txt(file_name=file_name)
            resp = internalResponse(json.dumps(self.data, ensure_ascii=False))
            return resp
        if result["method"] == "play_record_action_to_txt":  # 停止录制指定双臂 or 头和腰 or 双手 or 所有关节的动作序列到txt序列文本中
            try:
                file_name = result["params"]["file_name"]
            except:
                ColorMsg(msg="参数不正确", color="red")
                self.data["id"] = result["id"]
                self.data["params"] = result["params"]
                self.data["code"] = -1
                self.data["msg"] = "error"
                resp = internalResponse(json.dumps(self.data, ensure_ascii=False))
                return resp
            self.data["id"] = result["id"]
            self.data["params"] = result["params"]
            self.data["code"] = 0
            self.data["msg"] = "OK"
            self.ros_bag_controller.play_record_action_to_txt(file_name=file_name)
            resp = internalResponse(json.dumps(self.data, ensure_ascii=False))
            return resp
        if result["method"] == "start_collection": # 录制机械臂动作到文本
            
            try:
                file_name = result["params"]["file_name"]
                if result["params"]["image"] == "true" or result["params"]["image"] == "True":
                    image = True
                else:
                    image = False
            except:
                ColorMsg(msg="大数据采集参数不正确", color="red")
                self.data["id"] = result["id"]
                self.data["params"] = result["params"]
                self.data["code"] = 0
                self.data["msg"] = "参数不正确，没有指定文件名称"
                resp = internalResponse(json.dumps(self.data, ensure_ascii=False))
                return resp
            print(image)
            ColorMsg(msg="开始大模型数据采集", color="green")
            self.data["id"] = result["id"]
            self.data["params"] = result["params"]
            self.data["code"] = 0
            self.data["msg"] = "OK"
            self.ros_bag_controller.start_collection(image=image)
            resp = internalResponse(json.dumps(self.data, ensure_ascii=False))
            return resp
        if result["method"] == "stop_collection": # 录制机械臂动作到文本
            ColorMsg(msg="停止大模型数据采集", color="yellow")
            self.data["id"] = result["id"]
            self.data["params"] = result["params"]
            save_txt = int(self.data["params"]["save_txt"])
            file_name = result["params"]["file_name"]
            self.data["code"] = 0
            self.data["msg"] = "OK"
            self.ros_bag_controller.stop_collection(save_txt=save_txt,file_name=file_name)
            resp = internalResponse(json.dumps(self.data, ensure_ascii=False))
            return resp
        if result["method"] == "get_hdf5": # 录制机械臂动作到文本
            ColorMsg(msg="停止大模型数据采集", color="yellow")
            self.data["id"] = result["id"]
            self.data["params"] = result["params"]
            self.data["code"] = 0
            self.data["msg"] = "OK"
            self.ros_bag_controller.get_hdf5(action_name="hello_test")
            resp = internalResponse(json.dumps(self.data, ensure_ascii=False))
            return resp
        if result["method"] == "start_txt_joint": # 读取文本机械臂坐标信息发布到机械臂动作
            try:
                file_name = result["params"]["file_name"]
            except:
                file_name="pose_0"
            ColorMsg(msg="开始播放文本机械臂坐标", color="yellow")
            self.data["id"] = result["id"]
            self.data["code"] = 0
            self.data["msg"] = "OK"
            self.ros_bag_controller.hand_start_action(file_name=file_name)
            resp = internalResponse(json.dumps(self.data, ensure_ascii=False))
            return resp
        ''' ---------------------------------以下为底盘------------------------------------------ '''
        if json_key2 in result:
            par = result["params"]
            if result["method"] == "set_map":
                # 设置当前地图持久化，只调用一次即可
                rs, http_code = self.chassis_controller.set_map()
                if http_code == 200:
                    tmp_data["method"] = result["method"]
                    tmp_data["id"] = result["id"]
                    tmp_data["params"] = {
                        "task_id": result["params"]["task_id"],
                        "code": 0,
                        "msg": "success",
                        "data": rs
                    }
                else:
                    tmp_data["method"] = result["method"]
                    tmp_data["id"] = result["id"]
                    tmp_data["params"] = {
                        "task_id": result["params"]["task_id"],
                        "code": -1,
                        "msg": "error",
                        "data":{}
                    }
                resp = internalResponse(self.return_params(data=tmp_data,device_type="chassis"))
                return resp
            if result["method"] == "exit_thread":
                self.stop_thread()
                resp = internalResponse(json.dumps(self.data, ensure_ascii=False))
                return resp
            if result["method"] == "init_point": # 初始化点位到消防栓下
                self.chassis_controller.init_point()
                resp = internalResponse(json.dumps(self.data, ensure_ascii=False))
                return resp
            # 获取底盘当前定位质量,范围:0~100
            if result["method"] == "get_current_pose_quality":
                pose_quality, http_code = self.chassis_controller.get_current_pose_quality()
                if http_code == 200:
                    tmp_data["method"] = result["method"]
                    tmp_data["id"] = result["id"]
                    tmp_data["params"] = {
                        "task_id": result["params"]["task_id"],
                        "code": 0,
                        "msg": "success",
                        "data": pose_quality
                    }
                else:
                    tmp_data["method"] = result["method"]
                    tmp_data["id"] = result["id"]
                    tmp_data["params"] = {
                        "task_id": result["params"]["task_id"],
                        "code": -1,
                        "msg": "error",
                        "data":{}
                    }
                resp = internalResponse(self.return_params(data=tmp_data,device_type="chassis"))
                return resp
            if result["method"] == "get_device_info":
                # 获取地盘信息
                info, http_code = self.chassis_controller.get_device_info()
                if http_code == 200:
                    tmp_data["method"] = result["method"]
                    tmp_data["id"] = result["id"]
                    tmp_data["params"] = {
                        "task_id": result["params"]["task_id"],
                        "code": 0,
                        "msg": "success",
                        "data": info
                    }
                else:
                    tmp_data["method"] = result["method"]
                    tmp_data["id"] = result["id"]
                    tmp_data["params"] = {
                        "task_id": result["params"]["task_id"],
                        "code": -1,
                        "msg": "error",
                        "data":{}
                    }
                resp = internalResponse(self.return_params(data=tmp_data,device_type="chassis"))
                return resp
            if result["method"] == "get_all_poi":
                # 获取所有点位
                pois, http_code = self.chassis_controller.get_all_pois()
                if http_code == 200:
                    tmp_data["method"] = result["method"]
                    tmp_data["id"] = result["id"]
                    tmp_data["params"] = {
                        "task_id": result["params"]["task_id"],
                        "code": 0,
                        "msg": "success",
                        "data": pois
                    }
                else:
                    tmp_data["method"] = result["method"]
                    tmp_data["id"] = result["id"]
                    tmp_data["params"] = {
                        "task_id": result["params"]["task_id"],
                        "code": -1,
                        "msg": "error",
                        "data":{}
                    }
                
                resp = internalResponse(self.return_params(data=tmp_data,device_type="chassis"))
                return resp
            if result["method"] == "get_status":
                # 获取底盘状态
                status, http_code = self.chassis_controller.get_status()
                if http_code == 200:
                    tmp_data["method"] = result["method"]
                    tmp_data["id"] = result["id"]
                    tmp_data["params"] = {
                        "task_id": result["params"]["task_id"],
                        "code": 0,
                        "msg": "success",
                        "data": status
                    }
                else:
                    tmp_data["method"] = result["method"]
                    tmp_data["id"] = result["id"]
                    tmp_data["params"] = {
                        "task_id": result["params"]["task_id"],
                        "code": -1,
                        "msg": "error",
                        "data":{}
                    }
                resp = internalResponse(self.return_params(data=tmp_data,device_type="chassis"))
                return resp
            # 设置底盘移动速度，重启后则失效
            if result["method"] == "set_speed":
                if float(result["params"]["value"]) > 0.1:
                    value = result["params"]["value"]
                else:
                    value = "0.4"
                status, http_code = self.chassis_controller.set_speed(value=value)
                tmp_data["method"] = result["method"]
                tmp_data["id"] = result["id"]
                tmp_data["params"] = {
                    "task_id": result["params"]["task_id"],
                    "code": 0,
                    "msg": "success speed "+str(value),
                    "data":{}
                }
                resp = internalResponse(self.return_params(data=tmp_data,device_type="chassis"))
                return resp
            # 遥控移动，需要定时循环调用
            if result["method"] == "remote_control":
                try:
                    value = int(result["params"]["value"])
                except:
                    tmp_data["method"] = result["method"]
                    tmp_data["id"] = result["id"]
                    tmp_data["params"] = {
                        "task_id": result["params"]["task_id"],
                        "code": -1,
                        "msg": "参数不正确",
                        "data":{}
                    }
                    resp = internalResponse(self.return_params(data=tmp_data,device_type="chassis"))
                    return resp
                if value == 0 or value == 1 or value == 2 or value == 3:
                    status, http_code = self.chassis_controller.remote_control_move(value=value)
                    tmp_data["method"] = result["method"]
                    tmp_data["id"] = result["id"]
                    tmp_data["params"] = {
                        "task_id": result["params"]["task_id"],
                        "code": 0,
                        "msg": "success",
                        "data": status
                    }
                else:
                    tmp_data["method"] = result["method"]
                    tmp_data["id"] = result["id"]
                    tmp_data["params"] = {
                        "task_id": result["params"]["task_id"],
                        "code": -1,
                        "msg": "参数值不在可用范围内",
                        "data":{}
                    }
                resp = internalResponse(self.return_params(data=tmp_data,device_type="chassis"))
                return resp
            if result["method"] == "start_navi":
                #par = result["params"]\
                pose = {}
                self.data["params"] = {}
                point_name_tmp = par.get("target_name", None)
                translator = str.maketrans('','','。')
                point_name = point_name_tmp.translate(translator)
                point_pose = par.get("target_pose", None)
                if point_name != None:
                    # ---------点位名称导航
                    for item in self.all_pois:
                        if point_name == item["metadata"]["display_name"]:
                            pose = item["pose"]
                    
                    if pose.get("x", None) == None:
                        tmp_data["method"] = result["method"]
                        tmp_data["id"] = result["id"]
                        tmp_data["params"] = {
                            "task_id": result["params"]["task_id"],
                            "code": -1,
                            "msg": "Not Found",
                            "data":{}
                        }
                        resp = internalResponse(self.return_params(data=tmp_data,device_type="chassis"))
                        return resp
                    else:
                        self.thread_method = result["method"]
                        self.thread_id = result["id"]
                        self.thread_task_id = result["params"]["task_id"]
                        print("------------点位名称定位----------------")
                        print(point_name)
                        print("------------点位名称定位----------------")
                        absent = 0
                        for item in self.all_pois:
                            if item["metadata"]["display_name"] == point_name:
                                print(item["pose"]["x"])
                                move, http_code = self.chassis_controller.move_point_by_pose(x=int(item["pose"]["x"]),y=int(item["pose"]["y"]), z=0.0, yaw=int(item["pose"]["yaw"]))
                                #move, http_code = self.chassis_controller.move_point_by_name(point_name=point_name,yaw=pose["yaw"])
                            else:
                                absent = absent+1
                        if absent == len(self.all_pois):
                            tmp_data["method"] = result["method"]
                            tmp_data["id"] = result["id"]
                            tmp_data["params"] = {
                                "task_id": result["params"]["task_id"],
                                "code": 0,
                                "msg": "Point does not exist",
                                "data":{
                                    "action_id": move["action_id"]
                                }
                            }
                            resp = internalResponse(self.return_params(data=tmp_data,device_type="chassis"))
                            return resp
                        
                        if http_code == 200:
                            tmp_data["method"] = result["method"]
                            tmp_data["id"] = result["id"]
                            tmp_data["params"] = {
                                "task_id": result["params"]["task_id"],
                                "code": 0,
                                "msg": "success",
                                "data":{
                                    "action_id": move["action_id"]
                                }
                            }
                        else:
                            tmp_data["method"] = result["method"]
                            tmp_data["id"] = result["id"]
                            tmp_data["params"] = {
                                "task_id": result["params"]["task_id"],
                                "code": -1,
                                "msg": "error",
                                "data":{}
                            }
                        self.action_id = move["action_id"]
                        # 发布语音进行导航标签
                        self.tag = point_name
                        self.speak_msg.data = '{"method":"start_speak", "id":121212, "params":"好的,我马上去'+self.tag+'点位"}'
                        self.speak_pub.publish(self.speak_msg)
                elif point_pose != None and point_pose.get("x", None) != None and point_pose.get("y", None) != None:
                    # --------坐标导航
                    move, http_code = self.chassis_controller.move_point_by_pose(x=int(point_pose["x"]),y=int(point_pose["y"]), z=0.0, yaw=int(point_pose["yaw"]))
                    if http_code == 200:
                        tmp_data["method"] = result["method"]
                        tmp_data["id"] = result["id"]
                        tmp_data["params"] = {
                            "task_id": result["params"]["task_id"],
                            "code": 0,
                            "msg": "success",
                            "data":{}
                        }
                    else:
                        tmp_data["method"] = result["method"]
                        tmp_data["id"] = result["id"]
                        tmp_data["params"] = {
                            "task_id": result["params"]["task_id"],
                            "code": -1,
                            "msg": "error",
                            "data":{}
                        }
                    self.action_id = move["action_id"]
                else:
                    self.data["params"]["code"] = -1
                    self.data["params"]["msg"] = "参数错误"
                    # self.data["params"]["data"] = []
                resp = internalResponse(self.return_params(data=tmp_data,device_type="chassis"))
                return resp

            if result["method"] == "stop_navi":
                self.data["params"] = {}
                # 取消导航
                stop_move, http_code = self.chassis_controller.cancel_move()
                if http_code == 200:
                    tmp_data["method"] = result["method"]
                    tmp_data["id"] = result["id"]
                    tmp_data["params"] = {
                        "task_id": result["params"]["task_id"],
                        "code": 0,
                        "msg": "success",
                        "data":{}
                    }
                else:
                    tmp_data["method"] = result["method"]
                    tmp_data["id"] = result["id"]
                    tmp_data["params"] = {
                        "task_id": result["params"]["task_id"],
                        "code": -1,
                        "msg": "error",
                        "data":{}
                    }
                resp = internalResponse(self.return_params(data=tmp_data,device_type="chassis"))
                return resp
            if result["method"] == "navi_status":
                # 查询当前导航状态
                navi_status, http_code = self.chassis_controller.get_move_status(action_id=par["action_id"])
                if isinstance(navi_status, str) == True:
                    # 字符串格式说明没有这个ID任务
                    tmp_data["method"] = result["method"]
                    tmp_data["id"] = result["id"]
                    tmp_data["params"] = {
                        "task_id": result["params"]["task_id"],
                        "code": -1,
                        "msg": "error",
                        "data":{}
                    }

                else:
                    tmp_data["method"] = result["method"]
                    tmp_data["id"] = result["id"]
                    tmp_data["params"] = {
                        "task_id": result["params"]["task_id"],
                        "code": 0,
                        "msg": "success",
                        "data": navi_status
                    }
                resp = internalResponse(self.return_params(data=tmp_data,device_type="chassis"))
                return resp
            else:
                # self.rs["msg"] = "error"
                # ColorMsg(msg="指令错误，请检查手册", color="red")
                # res = json.dumps(self.rs)
                # resp = internalResponse(res)
                # return resp
                pass

        else:
            ColorMsg(msg="没有这个命令，请查看手册", color="red")
        # except:
        #     ColorMsg(msg="命令执行错误，请查看手册", color="red")
    
    # 返回参数
    def return_params(self,data, device_type="chassis"):
        print("************************************************")
        ColorMsg(msg="返回的数据", color="green")
        print(data)
        print("************************************************")
        if device_type == "chassis":
            result = {
                "method": data["method"],
                "id": data["id"],
                "params": {
                    "task_id": data["params"]["task_id"],
                    "code": data["params"]["code"],
                    "msg": data["params"]["msg"],
                    "data": data["params"]["data"]
                }
            }
        elif device_type == "arm":
            result = {
                "id": data["id"],
                "code": data["code"],
                "msg": data["msg"],
                "params": data["params"]
            }
        return json.dumps(result)

if __name__ == "__main__":
    rospy.init_node("robot_internal_server", anonymous=True)
    r = RobotInternalServer()
    rospy.spin()
    
