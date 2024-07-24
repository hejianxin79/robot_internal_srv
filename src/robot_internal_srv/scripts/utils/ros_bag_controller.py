#! /usr/bin/env python3
import rospy
import subprocess  
import signal

class RosbagController:
    def __init__(self):  
        self.is_recording = False  
        self.recorder_process = None  
        self.is_playing = False  
        self.player_process = None

  
    def start_recording(self, params, topic,bag_path):
        self.stop_playing() 
        if not self.is_recording:
            file = bag_path+"/"+params
            self.recorder_process = subprocess.Popen(["rosbag", "record", "-O",file, topic])  
            self.is_recording = True  
            rospy.loginfo(f"正在以rosbag录制,文件名为{params}.bag")  
  
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