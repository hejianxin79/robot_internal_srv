#! /usr/bin/env python3

import rospy, time, sys, os, json
import paho.mqtt.client as mqtt
from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from config.config import ADDRESS, PORT, CLIENT_ID,SERIAL_NUMBER,PWD,MQTT_SUB_TOPIC,ROS_PUB_TOPIC
from utils.color_msg import ColorMsg


class MqttListen:  
    def __init__(self):
        self.pub_topic = ROS_PUB_TOPIC 
        self.broker_address = ADDRESS  
        self.broker_port = PORT  
        self.mqtt_sub_topic = MQTT_SUB_TOPIC    
        self.mqtt_client = mqtt.Client(client_id=CLIENT_ID)
        self.mqtt_client.username_pw_set(username=SERIAL_NUMBER, password=PWD) 
        self.mqtt_client.on_connect = self.on_connect  
        self.mqtt_client.on_message = self.on_message  
        self.pub = None
        ColorMsg(msg="开启MQTT监听，收到MQTT信息后发布给机械臂", color="green")
  
    def on_connect(self, client, userdata, flags, rc):  
        print(f"Connected with result code {str(rc)}")  
        client.subscribe(self.mqtt_sub_topic)  
  
    def on_message(self, client, userdata, msg):  
        # 这里可以添加将MQTT消息发布到ROS话题的代码  
        if self.pub is not None:
            mqtt_msg = msg.payload.decode()
            # 监听到消息，进行解析，然后发布机械臂动作
            m = self.joint_msg(mqtt_msg=mqtt_msg)
            self.pub.publish(m)  
  
    def connect_mqtt(self):  
        self.mqtt_client.connect(self.broker_address, self.broker_port, 60)  
        self.mqtt_client.loop_start()  
  
    def disconnect_mqtt(self):  
        self.mqtt_client.loop_stop()  
        self.mqtt_client.disconnect()  
  
    def run(self):  
        rospy.init_node('mqtt_ros_listen', anonymous=True)  
        self.pub = rospy.Publisher(self.pub_topic, JointState, queue_size=100)  
        self.connect_mqtt()
        # 只做监听不给MQTT发送消息
        # 这里可以添加代码来定期发布MQTT消息到MQTT broker  
  
        # ROS节点将保持运行，直到被Ctrl+C中断  
        rospy.spin()  
  
        self.disconnect_mqtt()
    
    def joint_msg(self, mqtt_msg=""):
        if mqtt_msg != "":
            msg = json.loads(mqtt_msg)
            print("----------------")
            print(msg)
            self.v = msg["position"]                                       
            self.joint_names = msg["name"]
            joint_state = JointState()
            joint_state.header = Header()
            joint_state.header.frame_id = msg["header"]["frame_id"]
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = self.joint_names
            joint_state.position = self.v
            return joint_state

if __name__ == '__main__':
    try:  
        ml = MqttListen() # /1/0001/forward/post 为被控制话题
        ml.run()  
    except rospy.ROSInterruptException:  
        pass