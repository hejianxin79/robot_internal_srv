# hand_arm_controller_ros ROS DEMO

hand_arm_controller_ros ROS DEMO，支持 Melodic, 和 Noetic.

## 安装依赖

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


使用
- 手臂与头部同时运动
```bash
cd ~/ROS/robot_internal_srv/
source ./devel/setup.bash
rosservice call /robot_internal_srv "req: '{\"method\": \"start_speak_motion\", \"id\": 121212, \"params\": {\"loop\":\"True\",\"group_name\":\"leftarm\"}}'" # 头部与手臂同时运动 

rosservice call /robot_internal_srv "req: '{\"method\": \"stop_speak_motion\", \"id\": 121212, \"params\": {\"loop\":\"True\",\"group_name\":\"leftarm\",\"x\":\"-0.5\",\"y\":\"0.1\" }}'" # 头部与手臂同时停止
```
- 头部转向指定位置
参数:action为优先，如为空则使用xy坐标
```bash
cd ~/ROS/robot_internal_srv/
source ./devel/setup.bash
rosservice call /robot_internal_srv "req: '{\"method\": \"head_motion\", \"id\": 121212, \"params\": {\"action\":\"up|down|left|right|loop\",\"x\":\"-0.5\",\"y\":\"0.1\" }}'"  范围：x:-0.5~0.5   y:-0.3~0.1
```
- 带有底盘的机器人
```bash
cd ~/ROS/robot_internal_srv/
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
