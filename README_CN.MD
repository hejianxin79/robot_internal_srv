# robot_internal_srv ROS DEMO

robot_internal_srv ROS DEMO，支持 Melodic, 和 Noetic.

## jetson NX配置说明
用户名:nx
密码:12345678
hostname:nx
 - IP
 192.168.11.100 255.255.255.0 192.168.11.2 # 注:默认网关为192.168.11.2
 - ROS MASTER
 NX默认为ROSMASTER
  - WIFI
  NX的WI-FI被禁用
 - VNC
 NX默认为无显示器启动。若要开启HDMI信号输出连接显示器，需要按照以下操作进行
 ```bash
cd /usr/share/X11/xorg.conf.d/
sudo mv xorg.conf xorg.conf.bak   # 启用HDMI输出信号外接显示器，连接显示器后可VNC远程
or
sudo mv xorg.conf.bak xorg.conf   # 采用无显示模式远程连接VNC
sudo reboot
 ```

## 快速使用
 - 机器人打招呼
 ```bash
rosservice call /robot_internal_srv "req: '{\"method\": \"start_txt_joint\", \"id\": 121212, \"params\": {\"file_name\":\"hello_test\"}}'"
 ```

### ROS

- 请直接参考ROS官方[文档](http://wiki.ros.org/ROS/Installation).

### 其他依赖

### ROS主从机设置
主机与从机必须在同一个网络，并且从机必须固定IP地址
主机为机器人身上的jeston NX
- 主机设置
```bash
sudo vim /etc/hosts
你的从机IP 从机hostname
192.168.11.110  ubuntu 
```
- 从机设置
```bash
sudo vim /etc/hosts
你的主机IP 主机hostname
192.168.11.100  nx?? 
```

```bash
sudo vim ~/.bashrc
最下面加入
export ROS_HOSTNAME=从机的hostname
export ROS_MASTER_URI=http://主机IP:11311
export ROS_IP=从机IP 
```
将主机与从机重启
从机终端
```bash
rostopic list 
```
如果看到ROS的话题目录即主从通讯成功
- 其他问题请直接参考ROS官方[文档](http://wiki.ros.org/).



启动

- 硬件设备无误下，服务开启自启动，启动完毕会有语音提示。

远程桌面 

- 默认下NX开启了虚拟显示器，链接显示器无法输出
- 使用显示器输出
```bash
sudo mv /usr/share/X11/xorg.conf.d/xorg.conf /usr/share/X11/xorg.conf.d/xorg.conf.bak   (使用显示器输出)
sudo reboot
```
- 使用虚拟显示器输出
```bash
sudo mv /usr/share/X11/xorg.conf.d/xorg.conf.bak /usr/share/X11/xorg.conf.d/xorg.conf   (不使用显示器输出)
sudo reboot
```
奥比中光深度相机
 - 可查看官网文档 https://developer.orbbec.com.cn/develop_details.html?id=1
 - Install udev rules.
```bash
# 假定你已经source了你的ROS环境，下同
  sudo apt install libgflags-dev  ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev

cd ~/ROS/OrbbecSDK_ROS
catkin_make # 如果编译过 目录下devel和build文件夹则不需要编译，没有的话则需要编译
source ./devel/setup.bash
roscd orbbec_camera
cd scripts
sudo cp 99-obsensor-libusb.rules /etc/udev/rules.d/99-obsensor-libusb.rules 
sudo udevadm control --reload && sudo  udevadm trigger
```
 - 启动gemini2相机
```bash
cd ~/ROS/OrbbecSDK_ROS
catkin_make # 如果编译过 目录下devel和build文件夹则不需要编译，没有的话则需要编译
source ./devel/setup.bash
roslaunch orbbec_camera gemini2.launch 
```

使用
- 手臂与头部同时运动
```bash
cd ~/ROS/robot_internal_srv/
catkin_make # 如果编译过 目录下devel和build文件夹则不需要编译，没有的话则需要编译
source ./devel/setup.bash



rosservice call /robot_internal_srv "req: '{\"method\": \"start_speak_motion\", \"id\": 121212, \"params\": {\"loop\":\"True\",\"group_name\":\"leftarm\"}}'" # 头部与手臂同时运动 

rosservice call /robot_internal_srv "req: '{\"method\": \"stop_speak_motion\", \"id\": 121212, \"params\": {\"loop\":\"True\",\"group_name\":\"leftarm\",\"x\":\"-0.5\",\"y\":\"0.1\" }}'" # 头部与手臂同时停止
```
- 头部转向指定位置
参数:action为优先，如为空则使用xy坐标
```bash
cd ~/ROS/robot_internal_srv/
catkin_make # 如果编译过 目录下devel和build文件夹则不需要编译，没有的话则需要编译
source ./devel/setup.bash
rosservice call /robot_internal_srv "req: '{\"method\": \"head_motion\", \"id\": 121212, \"params\": {\"action\":\"up|down|left|right|loop\",\"x\":\"-0.5\",\"y\":\"0.1\" }}'"  范围：x:-0.5~0.5   y:-0.3~0.1
```
- 带有底盘的机器人
```bash
cd ~/ROS/robot_internal_srv/
catkin_make # 如果编译过 目录下devel和build文件夹则不需要编译，没有的话则需要编译
source ./devel/setup.bash

查询当前导航状态: rosservice call /robot_internal_srv "req: '{\"method\": \"navi_status\", \"id\": 121212, \"params\": {\"task_id\": \"asdfasdf\", \"action_id\": \"4\"}}'"

获取地图所有点位: rosservice call /robot_internal_srv "req: '{\"method\":\"get_all_poi\", \"id\":121212, \"params\":{\"task_id\":\"123123\"}}'"

停止导航:  rosservice call /robot_internal_srv "req: '{\"method\":\"stop_navi\", \"id\":121212, \"params\":{\"task_id\":\"123123\"}}'"

移动导航:  rosservice call /robot_internal_srv "req: '{\"method\": \"start_navi\", \"id\": 121212, \"params\": {\"task_id\": \"asdfasdf\", \"target_name\": \"门口\", \"target_pose\": {\"x\": 0, \"y\": 0, \"z\": 0}}}'"

播放语音（需要机器人路由器连接网络）  rostopic pub /app_control_cmd std_msgs/String "data: '{\"method\":\"start_speak\", \"id\":121212, \"params\":\"您好，很高兴认识您\"}'"

```


选择你想要显示的topic

- 查看 topics / services/ parameters (open a new terminal)

```bash
rostopic list
rosservice list
rosparam list
```


机械臂与头部所有命令示例
```bash
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
```


机器人底盘所有命令示例
```bash
获取地图所有点位: rosservice call /robot_internal_srv "req: '{\"method\":\"get_all_poi\", \"id\":121212, \"params\":{\"task_id\":\"123123\"}}'"

移动导航:  rosservice call /robot_internal_srv "req: '{\"method\": \"start_navi\", \"id\": 121212, \"params\": {\"task_id\": \"asdfasdf\", \"target_name\": \"门口\", \"target_pose\": {\"x\": 0, \"y\": 0, \"z\": 0}}}'"

遥控移动：rosservice call /robot_internal_srv "req: '{\"method\": \"remote_control\", \"id\": 121212, \"params\": {\"value\":\"0|1|2|3\",\"task_id\":121212}}'"  0前进 1后退 2左转 3右转

播放语音  rostopic pub /app_control_cmd std_msgs/String "data: '{\"method\":\"start_speak\", \"id\":121212, \"params\":\"好的,我马上去沙发点位\"}'"


查询当前导航状态: rosservice call /robot_internal_srv "req: '{\"method\": \"navi_status\", \"id\": 121212, \"params\": {\"task_id\": \"asdfasdf\", \"action_id\": \"4\"}}'"

停止导航:  rosservice call /robot_internal_srv "req: '{\"method\":\"stop_navi\", \"id\":121212, \"params\":{\"task_id\":\"123123\"}}'"

停止线程： rosservice call /robot_internal_srv "req: '{\"method\":\"exit_thread\", \"id\":121212, \"params\":{\"task_id\":\"123123\"}}'"

初始化定位到消防栓下： rosservice call /robot_internal_srv "req: '{\"method\":\"init_point\", \"id\":121212, \"params\":{\"task_id\":\"123123\"}}'"

地图持久化： rosservice call /robot_internal_srv "req: '{\"method\":\"set_map\", \"id\":121212, \"params\":{\"task_id\":\"123123\"}}'"


升降机  rosservice call /robot_internal_srv "req: '{\"method\":\"lifter\", \"id\":121212, \"params\":{\"task_id\":\"123123\",\"range\":\"100\"}}'"

```


## 机器人部分topic说明 
# 控制为可pub 状态只能echo
 - /cb_arm_control_cmd  只控制双臂
 - /joint_states 双臂状态
 
 - /cb_body_control_cmd 只控制头和腰部
 - /cb_body_control_state 头部和腰部状态

 - /cb_left_hand_control_cmd 只控制左手
 - /cb_left_hand_state 左手状态

 - /cb_right_hand_control_cmd 只控制右手
 - /cb_right_hand_state 右手状态




## 机械臂状态
6. 机器人提供Topic API定义
6.1 机械臂关节状态
Topic name: /joint_states
Type:sensor_msgs/JointState
数据定义：
header:
  seq: 55694
  stamp:
    secs: 1721301330
    nsecs: 440304525
  frame_id: ''
name: 
    - joint11   右臂第1肩关节
    - joint12   右臂第2肩关节
    - joint13   右臂第3肩关节
    - joint14   右臂第4肘关节
    - joint15   右臂第5肘关节
    - joint16   右臂第6腕关节
    - joint2    头部Z轴关节
    - joint21   左臂第1肩关节
    - joint22   左臂第2肩关节
    - joint23   左臂第3肩关节
    - joint24   左臂第4肘关节
    - joint25   左臂第5肘关节
    - joint26   左臂第6腕关节
    - joint3    头部X轴关节   //暂时未实现
    - joint31   腰部Z轴关节 //暂时未实现
    - joint32   腰部Y轴关节 //暂时未实现
    - joint4    头部Y轴关节 //暂时未实现
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
6.2 手关节状态
Topic name：
左手状态：/cb_left_hand_state
左手状态：/cb_right_hand_state
示例：
header:
  seq: 531943
  stamp:
    secs: 1721789622
    nsecs: 491601135
  frame_id: ''
name: 
- joint41     //右手拇指1关节
- joint42    //右手拇指2关节
- joint43    //右手食指关节
- joint44    //右手中指关节
- joint45    //右手无名指关节
- joint46    //右手小指关节
- joint47    //预留
- joint48
- joint49
- joint50
- joint51
- joint52
- joint53
- joint54
- joint55
- joint56
- joint57
- joint58
- joint59
- joint60
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity: []
effort: []

---
6.3 身体关节状态
Topic name：/cb_body_state
示例：
header:
  seq: 373974
  stamp:
    secs: 1721789702
    nsecs: 343813544
  frame_id: ''
name: 
- joint101
- joint102
- joint103
- joint104
- joint105
position: [0.0, 0.0, 0.0, 0.0, 0.0]
velocity: []
effort: []

---
6.4 机械臂关节控制
Topic name：/cb_arm_control_cmd
Type：sensor_msgs/JointState
数据定义：
header:
  seq: 325
  stamp:
    secs: 1721300121
    nsecs: 447809034
  frame_id: ''
name: 
- joint11    右臂第1肩关节
- joint12    右臂第2肩关节
- joint13    右臂第3肩关节
- joint14    右臂第4肘关节
- joint15    右臂第5肘关节
- joint16    右臂第6腕关节
- joint21    左臂第1肩关节
- joint22    左臂第2肩关节
- joint23    左臂第3肩关节
- joint24    左臂第4肘关节
- joint25    左臂第5肘关节
- joint26    左臂第6腕关节
position: [0.011193275451660156, 0.024183988571166992, 2.0708260536193848, 0.05666148662567139, -1.5571820735931396, -0.052562713623046875, 0.03588080406188965, -0.012127876281738281, -1.2907249927520752, -0.12554669380187988, 1.2810659408569336, 1.2289104461669922]
velocity: []   //置空
effort: []      // 置空
6.5 手关节控制
Topic name：
左手关节：/cb_left_hand_control_cmd    //关节值单位弧度，取值（0-1.57）
右手关节：/cb_right_hand_control_cmd 
示例：
header:
  seq: 184926
  stamp:
    secs: 1721788946
    nsecs: 144241680
  frame_id: ''
name: 
- joint41
- joint42
- joint43
- joint44
- joint45
- joint46
- joint47
- joint48
- joint49
- joint50
- joint51
- joint52
- joint53
- joint54
- joint55
- joint56
- joint57
- joint58
- joint59
- joint60
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity: []
effort: []
6.6 身体关节控制
Topic name：/cb_body_control_cmd 
示例：
header:
  seq: 240344
  stamp:
    secs: 1721789167
    nsecs: 823485344
  frame_id: ''
name: 
- joint101 //头部Z轴关节
- joint102 //头部Y轴关节
- joint103 //头部X轴关节
- joint104 // 腰部Z轴关节 
- joint105 //腰部Y轴关节
position: [0.0, 0.0, 0.0, 0.0, 0.0]
velocity: []
effort: []