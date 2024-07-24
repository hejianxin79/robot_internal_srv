ADDRESS='192.144.239.184'           # '42.193.121.181'
PORT=1883                # 2883
CLIENT_ID='0001'
SERIAL_NUMBER="0001"
PWD="VJd1gLGj"
MQTT_SUB_TOPIC='/beaubot/forward/slave/0002' # 接收端                #'/1/0001/forward/post' # 0001为被控制
MQTT_PUB_TOPIC='/beaubot/forward/master/0001' # 发送端                             #"/1/0002/forward/simulate" # 0002 为控制
ROS_PUB_TOPIC="/cb_joint_cmd"
ROS_SUB_TOPIC="/cb_joint_cmd"
# 仿真
#ROS_PUB_TOPIC="/joint_states"
#ROS_SUB_TOPIC="/joint_states"