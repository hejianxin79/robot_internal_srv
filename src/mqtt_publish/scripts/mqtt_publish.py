#! /usr/bin/env python3

import rospy, time, sys, os, json
import paho.mqtt.client as mqtt
from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from config.config import ADDRESS, PORT, CLIENT_ID,SERIAL_NUMBER,PWD,MQTT_SUB_TOPIC,ROS_SUB_TOPIC
from utils.color_msg import ColorMsg
class MqttPublish:  
    def __init__(self):
        self.broker_address = ADDRESS
        self.broker_port = PORT
        self.client_id = CLIENT_ID
        self.mqtt_client = mqtt.Client(client_id=self.client_id)
        self.mqtt_client.username_pw_set(SERIAL_NUMBER, PWD) 
        self.mqtt_client.on_connect = self.on_connect  
        self.mqtt_client.on_message = self.on_message  
        self.pub = None
        ColorMsg(msg="开启MQTT发布，收到机械臂ROS的topic转发给MQTT", color="green")
  
    def on_connect(self, client, userdata, flags, rc):  
        print(f"Connected with result code {str(rc)}")  
        #self.mqtt_client.subscribe(MQTT_SUB_TOPIC)
        sub = rospy.Subscriber(ROS_SUB_TOPIC,JointState,self.toMqtt,queue_size=100)
  
    def on_message(self, client, userdata, msg):  
        rospy.loginfo(f"Received MQTT message on topic '{msg.topic}': {msg.payload.decode()}")  
  
    def connect_mqtt(self):  
        self.mqtt_client.connect(self.broker_address, self.broker_port, 60)  
        self.mqtt_client.loop_start()  
  
    def disconnect_mqtt(self):  
        self.mqtt_client.loop_stop()  
        self.mqtt_client.disconnect()
    # 发布给MQTT服务器
    def toMqtt(self, data):
        print("+++++++++++++++")
        print(data)
        tmp_dict = {
            "header": {
                "seq": data.header.seq,
                "stamp": {
                    "secs": data.header.stamp.secs,
                    "nsecs": data.header.stamp.nsecs
                },
                "frame_id": data.header.frame_id
            },
            "name": data.name,
            "position": data.position,
            "velocity": data.velocity,
            "effort": data.effort
        }
        j_m = json.dumps(tmp_dict)
        self.mqtt_client.publish(MQTT_SUB_TOPIC, j_m, qos=1)
        
    def run(self):  
        rospy.init_node('mqtt_ros_publish', anonymous=True)   
        self.connect_mqtt()
        # 这里获取机械臂遥操的话题然后发布给mqtt
        
        # ROS节点将保持运行，直到被Ctrl+C中断  
        rospy.spin()  
  
        self.disconnect_mqtt()
    
    def joint_msg(self, mqtt_msg=""):
        self.v = [0.0,0.0,0.0,-0.00099, -8.6,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]                                       
        self.joint_names = ["joint2","joint3","joint4","joint11","joint12","joint13","joint14","joint15","joint16","joint21","joint22","joint23","joint24","joint25","joint26","joint31","joint32"]
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.frame_id = "world"
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self.joint_names
        joint_state.position = self.v
        return joint_state

if __name__ == '__main__':
    try:  
        mp = MqttPublish() # /1/0001/forward/post 为被控制话题
        #bridge = MqttRosBridge('42.193.121.181', 2883, '0002', '/1/0002/#')
        mp.run()  
    except rospy.ROSInterruptException:  
        pass