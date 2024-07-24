#!/usr/bin/env python
# coding=utf-8

# 485通讯 9600波特率

# 眼睛控制指令：
# !normal_start\n   说话时候的眼睛控制；
# !silent_start\n      不说话的时候的眼睛控制
# !stop\n               停止

# 嘴巴控制指令 十六进制指令 
# 01 06 00 2A 10 05 65 C1 闭嘴
# 01 10 00 2B 00 02 04 00 00 02 90 b1 08  嘴巴全开
# 01 10 00 2B 00 02 04 00 00 01 90 B1 f8  嘴巴半开

# rostopic pub /columbus_head/command std_msgs/String "data: '{\"action\": \"talk\"}'"  说话
# rostopic pub /columbus_head/command std_msgs/String "data: '{\"action\": \"stop\"}'"  停止说话
# rostopic pub /columbus_head/command std_msgs/String "data: '{\"action\": \"silent\"}'"  静默状态


import rospy
import serial, time, json
from std_msgs.msg import String

class ColumbusHead:
    def __init__(self):
        self.is_stop = False
        # 使用函数发送指令
        #self.port_name = '/dev/ttyUSB0'  # 替换为你的串行端口名称，例如COM3, /dev/ttyUSB0等
        self.port_name = '/dev/columbus'  # 替换为你的串行端口名称，例如COM3, /dev/ttyUSB0等
        self.baudrate = 9600  # 波特率
        
        self.eye_activity_silent = '!silent_start\n'  # 静默状态动眼指令
        self.eye_activity_normal = '!normal_start\n'   # 说话状态动眼指令
        self.eye_activity_stop = "!stop\n"  # 动眼停止
        self.open_mouth_half = b'\x01\x10\x00\x2B\x00\x02\x04\x00\x00\x01\x90\xB1\xF8' # 嘴巴半张开
        self.open_mouth = b'\x01\x10\x00\x2B\x00\x02\x04\x00\x00\x02\x90\xb1\x08' # 嘴巴张开
        self.close_mouth = b'\x01\x06\x00\x2A\x10\x05\x65\xC1'
        self.ser = serial.Serial(self.port_name, self.baudrate, timeout=1)
        if self.ser.isOpen() :
            print("Serial open success")
            time.sleep(1)
            print("开启静默模式")
            self.silent()
        else:
            print("Error, can't open serial")

    def send_command_to_rs485(self,port_name, baudrate, command, command_type="Hex"):
        #print("Send comd to head" + command)
        # 创建一个串行端口对象
        #ser = serial.Serial(port_name, baudrate, timeout=1)

        # 检查串行端口是否打开
        if not self.ser.isOpen():
            print("无法打开串行端口, reopen")
            self.ser = serial.Serial(port_name, baudrate, timeout=1)
            if not self.ser.isOpen() :
                print("Error, can't open serial")
                return
        # 发送指令
        if command_type != "Hex":
            # 如果是字符串命令，需要进行编码
            self.ser.write(command.encode())
        else:
            self.ser.write(command)
        #print(f"已发送指令: {command}")
        # 可以选择等待一段时间以接收响应，这里我们只是简单等待然后关闭端口
        time.sleep(0.3)

    def doMsg(self, data):
        print("--------"+data.data+"---------")
        j = json.loads(data.data)
        if j["action"] == "stop":
            # 关闭嘴巴
            self.send_command_to_rs485(self.port_name, self.baudrate, self.close_mouth)
            time.sleep(0.3)
            # 禁止循环动嘴
            self.is_stop = True
        if j["action"] == "talk":
            # 关闭禁止循环动嘴
            self.is_stop = False
            # 开启说话动眼
            self.send_command_to_rs485(self.port_name, self.baudrate,self.eye_activity_normal, command_type="str")
            # 开始循环动嘴
            count = 0
            while True:
                count += 1
                if (count > 5):
                    break
                if self.is_stop == False:
                    self.send_command_to_rs485(self.port_name, self.baudrate, self.open_mouth)
                    self.send_command_to_rs485(self.port_name, self.baudrate, self.close_mouth)
                else:
                    self.send_command_to_rs485(self.port_name, self.baudrate, self.close_mouth)
                    break
            time.sleep(1)
            self.send_command_to_rs485(self.port_name, self.baudrate, self.eye_activity_stop, command_type="str")
        if j["action"] == "silent":
            print("Run silent action")
            self.silent()
            

    def silent(self):
        self.is_stop = True
        time.sleep(1)
        self.send_command_to_rs485(self.port_name, self.baudrate, self.eye_activity_stop, command_type="str")
        time.sleep(1)
        self.send_command_to_rs485(self.port_name, self.baudrate,self.eye_activity_silent, command_type="str")


    def main(self):
        sub = rospy.Subscriber("/columbus_head/command", String, self.doMsg, queue_size=10)
        rospy.spin()
        # 关闭串行端口
        self.ser.close()
        print("串行端口已关闭")

if __name__=="__main__":
    rospy.init_node("columbus_head_node")
    b = ColumbusHead()
    b.main()
