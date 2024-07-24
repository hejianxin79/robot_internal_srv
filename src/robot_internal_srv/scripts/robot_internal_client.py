#! /usr/bin/env python3

'''
机械臂归位初始点为 rosservice call /robot_internal_srv "req: '{\"method\": \"arm_init_point\", \"id\": 121212, \"params\": {\"name\": \"hello\",\"task_id\": \"asdfasdf\", \"action_id\": \"4\"}}'"

{'method': 'get_all_poi', 'id': 112233, 'params': {'task_id': '66666', 'code': 0, 'msg': 'success', 'data': [{'id': '53b36930-4896-47ab-96cd-1e6fd4ef5f16', 'metadata': {'display_name': '消防栓下'}, 'pose': {'x': 7.71999979019165, 'y': 5.840000152587891, 'yaw': 0.0}}, {'id': '84b5f1af-d485-43d9-8635-a2e86e72bd5f', 'metadata': {'display_name': '奇点云门口'}, 'pose': {'x': -2.799999952316284, 'y': 7.78000020980835, 'yaw': 0.0}}, {'id': '87e3e5cd-0b21-40f1-a65d-3f5f9bed32a5', 'metadata': {'display_name': '大门'}, 'pose': {'x': 7.059999942779541, 'y': 3.079999923706055, 'yaw': 0.9000000953674316}}, {'id': '8da852bd-7a2d-4c0c-ad28-4840aa5d0c78', 'metadata': {'display_name': '门口'}, 'pose': {'x': -11.69999980926514, 'y': 9.880000114440918, 'yaw': 0.0}}, {'id': 'aee7a098-6b42-4b91-aff8-046cd8871cca', 'metadata': {'display_name': 'A2'}, 'pose': {'x': 5.639999866485596, 'y': 5.079999923706055, 'yaw': 3.140000104904175}}, {'id': 'b64f34c7-a02b-4bd5-a5f4-767ef564c6aa', 'metadata': {'display_name': '楼梯下'}, 'pose': {'x': -14.82999992370605, 'y': 8.729999542236328, 'yaw': -0.2099999934434891}}, {'id': 'bb961382-8e0b-4251-9b88-d96a9b5d69dd', 'metadata': {'display_name': '上古资本门口'}, 'pose': {'x': -12.36999988555908, 'y': 9.3100004196167, 'yaw': 1.360000014305115}}, {'id': 'cbfe0adb-6aee-4eb4-8890-fc603060c549', 'metadata': {'display_name': 'A3'}, 'pose': {'x': -12.15999984741211, 'y': 5.090000152587891, 'yaw': 0.0}}, {'id': 'd7697dda-4ce4-49dc-95fb-a94d9f2e877e', 'metadata': {'display_name': '闸机2'}, 'pose': {'x': 5.5, 'y': -0.3600000143051147, 'yaw': 0.0}}, {'id': 'f5094da3-8306-4743-8ced-1cd7078d8346', 'metadata': {'display_name': 'A1'}, 'pose': {'x': -12.88000011444092, 'y': 8.319999694824219, 'yaw': 0.0}}]}}

'{"method": "run", "id": 112233, "params": {"task_id": "66666"}}'
'{"method": "get_all_poi", "id": 112233, "params": {"task_id": "66666"}}'   获取所有点位
'{"method": "start_navi", "id": 121212, "params": {"task_id": "asdfasdf", "target_name": "门口", "target_pose": {"x": 0, "y": 0, "z": 0}}}' 移动命令,返回action_id
'{"method": "navi_status", "id": 121212, "params": {"task_id": "asdfasdf", "action_id": "4"}}' #通过action_id查询移动状态命令
'{"method": "start_play_motion", "id": 121212, "params": {"name": "hello","task_id": "asdfasdf"}}'  # 手臂rosbag play动作
'''


import rospy, sys, os, json, time
from std_msgs.msg import String
from robot_internal_srv.srv import internal, internalRequest,internalRequest
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from utils.color_msg import ColorMsg

ALL =  [
    {'id': '84b5f1af-d485-43d9-8635-a2e86e72bd5f', 'metadata': {'display_name': '奇点云门口'}, 'pose': {'x': -2.799999952316284, 'y': 7.78000020980835, 'yaw': 0.0}}, 
    {'id': '8da852bd-7a2d-4c0c-ad28-4840aa5d0c78', 'metadata': {'display_name': '门口'}, 'pose': {'x': -11.69999980926514, 'y': 9.880000114440918, 'yaw': 0.0}},
    {'id': 'b64f34c7-a02b-4bd5-a5f4-767ef564c6aa', 'metadata': {'display_name': '楼梯下'}, 'pose': {'x': -14.82999992370605, 'y': 8.729999542236328, 'yaw': -0.2099999934434891}},
    {'id': 'bb961382-8e0b-4251-9b88-d96a9b5d69dd', 'metadata': {'display_name': '上古资本门口'}, 'pose': {'x': -12.36999988555908, 'y': 9.3100004196167, 'yaw': 1.360000014305115}}, 
    {'id': 'cbfe0adb-6aee-4eb4-8890-fc603060c549', 'metadata': {'display_name': 'A3'}, 'pose': {'x': -12.15999984741211, 'y': 5.090000152587891, 'yaw': 0.0}}, 
    {'id': 'f5094da3-8306-4743-8ced-1cd7078d8346', 'metadata': {'display_name': 'A1'}, 'pose': {'x': -12.88000011444092, 'y': 8.319999694824219, 'yaw': 0.0}}
    ]

class RobotInternalClient:
    def __init__(self) -> None:
        self.client = rospy.ServiceProxy("robot_internal_srv", internal)
        self.client.wait_for_service()
        self.req = internalRequest()
        self.speech_pub = rospy.Publisher('/app_control_cmd', String, queue_size=1)
        self.speech_msg = String()
        # 所有点位
        self.all_poi = None
        self.exit_loop = False
    # 发送命令
    def send(self, cmd):
        self.req.req = cmd
        resp = self.client.call(self.req.req)
        result = json.loads(resp.resp)
        print("***********************************")
        ColorMsg(msg="ROS Server返回的数据")
        print(result)
        print("***********************************")
        return result
    def run(self):
        # 获取所有点位
        self.all_poi = self.send(cmd = '{"method": "get_all_poi", "id": 112233, "params": {"task_id": "66666"}}')
        # 循环发送点位
        if self.all_poi != None:
            #for poi in self.all_poi["params"]["data"]:
            for poi in ALL:
                poi_name = poi["metadata"]["display_name"]
                # {'method': 'start_navi', 'id': 121212, 'params': {'task_id': 'asdfasdf', 'code': 0, 'msg': 'success', 'data': {'action_id': 42}}}
                action = self.send(cmd='{"method": "start_navi", "id": 121212, "params": {"task_id": "asdfasdf", "target_name": "'+poi_name+'", "target_pose": {"x": 0, "y": 0, "z": 0}}}')
                ColorMsg(msg=f"开始移动到{poi_name}", color="yellow")
                # 播放语音，移动到某一点位
                self.speech_msg.data = '{"method":"start_speak", "id":121212, "params":"我将要移动到'+str(poi_name)+'点位"}'
                self.speech_pub.publish(self.speech_msg)
                # 无线循环任务状态，不在移动则退出循环
                while True:
                    action_id = action["params"]["data"]["action_id"]
                    # {'method': 'navi_status', 'id': 121212, 'params': {'task_id': 'asdfasdf', 'code': 0, 'msg': 'success', 'data': {'action_id': 45, 'action_name': 'slamtec.agent.actions.MultiFloorMoveAction', 'stage': 'GOING_TO_TARGET', 'state': {'reason': '', 'result': 0, 'status': 1}}}}
                    status = self.send(cmd = '{"method": "navi_status", "id": 121212, "params": {"task_id": "asdfasdf", "action_id": "'+str(action_id)+'"}}')
                    ColorMsg(msg=f"查询{poi_name}移动状态", color="yellow")
                    if status["params"]["data"]["state"]["status"] != 1 and status["params"]["data"]["state"]["status"] != 4:
                        ColorMsg(msg=f"{poi_name}移动失败，请检查底盘急停按钮", color="red")
                        #self.exit_loop = True
                        time.sleep(2)
                        break
                    elif status["params"]["data"]["state"]["status"] == 4:
                        #移动成功
                        ColorMsg(msg=f"{poi_name}移动成功", color="green")
                        # 播放语音
                        # '{"method":"start_speak", "id":121212, "params":"你好"}'
                        self.speech_msg.data = '{"method":"start_speak", "id":121212, "params":"成功到达指定点位'+poi_name+'"}'
                        self.speech_pub.publish(self.speech_msg)
                        ColorMsg(msg=f"发布语音播放话题", color="green")
                        # 播放手臂
                        arm = self.send(cmd='{"method": "start_play_motion", "id": 121212, "params": {"name": "hello","task_id": "asdfasdf"}}')
                        ColorMsg(msg=f"开始播放hello手臂动作", color="green")
                        # 睡眠30秒后退出无限循环继续下一个点位
                        time.sleep(30)
                        arm_init = self.send(cmd = '{"method": "arm_init_point", "id": 121212, "params": {"name":"111","task_id": "asdfasdf", "action_id": "4"}}')
                        time.sleep(10)
                        break
                    else:
                        ColorMsg(msg="正在移动中....", color="green")
                        time.sleep(2)
                        pass

                
                if self.exit_loop == True:
                    break


if __name__ == "__main__":
    rospy.init_node("robot_internal_client", anonymous=True)
    rc = RobotInternalClient()
    rc.run()
    