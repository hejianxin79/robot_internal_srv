#! /usr/bin/env python3

import rospy, sys, os, requests
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from http_request import HTTPRequest

'''
获取地图所有点位: rosservice call /robot_internal_srv "req: '{\"method\":\"get_all_poi\", \"id\":121212, \"params\":{\"task_id\":\"123123\"}}'"

移动导航:  rosservice call /robot_internal_srv "req: '{\"method\": \"start_navi\", \"id\": 121212, \"params\": {\"task_id\": \"asdfasdf\", \"target_name\": \"门口\", \"target_pose\": {\"x\": 0, \"y\": 0, \"z\": 0}}}'"

遥控移动：rosservice call /robot_internal_srv "req: '{\"method\": \"remote_control\", \"id\": 121212, \"params\": {\"value\":\"0|1|2|3\",\"task_id\":121212}}'"  0前进 1后退 2左转 3右转

播放语音  rostopic pub /app_control_cmd std_msgs/String "data: '{\"method\":\"start_speak\", \"id\":121212, \"params\":\"好的,我马上去沙发点位\"}'"


查询当前导航状态: rosservice call /robot_internal_srv "req: '{\"method\": \"navi_status\", \"id\": 121212, \"params\": {\"task_id\": \"asdfasdf\", \"action_id\": \"4\"}}'"

停止导航:  rosservice call /robot_internal_srv "req: '{\"method\":\"stop_navi\", \"id\":121212, \"params\":{\"task_id\":\"123123\"}}'"

停止线程： rosservice call /robot_internal_srv "req: '{\"method\":\"exit_thread\", \"id\":121212, \"params\":{\"task_id\":\"123123\"}}'"

初始化定位到消防栓下： rosservice call /robot_internal_srv "req: '{\"method\":\"init_point\", \"id\":121212, \"params\":{\"task_id\":\"123123\"}}'"

地图持久化： rosservice call /robot_internal_srv "req: '{\"method\":\"set_map\", \"id\":121212, \"params\":{\"task_id\":\"123123\"}}'"

机械臂归位到零点 暂时不要用 rosservice call /robot_internal_srv "req: '{\"method\": \"arm_init_point\", \"id\": 121212, \"params\": {\"name\": \"hello\",\"task_id\": \"asdfasdf\", \"action_id\": \"4\"}}'"

头部运动  rosservice call /robot_internal_srv "req: '{\"method\": \"start_speak_motion\", \"id\": 121212, \"params\": {\"x\":\"-0.5\",\"y\":\"0.1\" }}'"  x:-0.5~0.5   y:-0.3~0.1
rostopic pub /cb_body_control_cmd std_msgs/Float32MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0.0,-1.0,-1.0]" --rate 10

机械臂抓取  rosservice call /robot_internal_srv "req: '{\"method\":\"arm_grab\", \"id\":121212, \"params\":{\"task_id\":\"123123\",\"group_name\":\"leftarm\",\"class_name\":\"cup\"}}'"

相机坐标转换 rosrun tf static_transform_publisher 0.5 0 0.3 0 0 0 1 base_link camera_link 100

升降机  rosservice call /robot_internal_srv "req: '{\"method\":\"lifter\", \"id\":121212, \"params\":{\"task_id\":\"123123\",\"range\":\"100\"}}'"

'''

class ChassisController:
    def __init__(self) -> None:
        self.http = HTTPRequest()


    def init_point(self):
        # 消防栓下
        # json_dic = {
        #     "x": 7.71,
        #     "y": 5.84,
        #     "z": 0,
        #     "yaw": 1.04,
        #     "picth": 0,
        #     "roll": 0
        # }
        # 奇点云门口
        json_dic = {
            "x": -2.7999999,
            "y": 7.78000020,
            "z": 0,
            "yaw": 0,
            "picth": 0,
            "roll": 0
        }

        return self.http.put(endpoint="/api/core/slam/v1/localization/pose", data=json_dic)

    def get_action_factories(self):
        '''
        获取底盘当前支持的所有action模式
        '''
        return self.http.get(endpoint="/api/core/motion/v1/action-factories")
    # 查询当前任务的当前状态
    def get_current_action(self):
        return self.http.get(endpoint="/api/core/motion/v1/actions/:current")
    # 根据action_id查询指定任务状态
    def get_current_action_by_id(self, action_id):
        if action_id == 0:
            return None
        else:
            return self.http.get(endpoint="/api/core/motion/v1/actions/"+str(action_id), print_out=False)
    # 设置地图持久化，建图后使用一次即可
    def set_map(self):
        return self.http.post(endpoint="/api/multi-floor/map/v1/stcm/:save")
        
    # 获取所有当前地图上的星标点位
    def get_all_pois(self):
        return self.http.get(endpoint="/api/core/artifact/v1/pois")
    # 获取电池电量
    def get_status(self):
        return self.http.get(endpoint="/api/core/system/v1/power/status")
    # 取消导航移动
    def cancel_move(self):
        return self.http.delete(endpoint="/api/core/motion/v1/actions/:current")
    # 获取指定action_id的移动状态
    def get_move_status(self, action_id):
        return self.http.get(endpoint="/api/core/motion/v1/actions/"+str(action_id))

    # 获取底盘设备信息
    def get_device_info(self):
        device_info = {}
        # /api/core/system/v1/capabilities 机器人底盘能力
        device_info["capabilities"], state_code = self.http.get(endpoint="/api/core/system/v1/capabilities", print_out=False)
        # /api/core/system/v1/power/status 电源状态
        device_info["power_status"], state_code = self.http.get(endpoint="/api/core/system/v1/power/status", print_out=False)
        # /api/core/system/v1/robot/info 设备信息
        device_info["info"], state_code = self.http.get(endpoint="/api/core/system/v1/robot/info", print_out=False)
        # /api/core/system/v1/robot/health 健康状态
        device_info["current_pose_health"], state_code = self.http.get(endpoint="/api/core/system/v1/robot/health", print_out=False)
        # /api/core/system/v1/parameter?param=base.max_moving_speed base.max_moving_speed - 最大线速度 base.max_angular_speed - 最大角速度 docking.docked_register_strategy - 充电桩注册策略，always 每次回桩都注册，when_not_exists 桩不存在时注册
        device_info["max_moving_speed"], state_code = self.http.get(endpoint="/api/core/system/v1/parameter?param=base.max_moving_speed", print_out=False)
        return device_info, state_code

        
    # 获取机器人当前位姿
    def get_current_pose(self):
        return self.http.get(endpoint="/api/core/slam/v1/localization/pose")
    
    # 获取当前定位质量
    def get_current_pose_quality(self):
        return self.http.get(endpoint="/api/core/slam/v1/localization/quality")
    
    # 获取充电桩位置
    def get_home_pose(self):
        return self.http.get(endpoint="/api/core/slam/v1/homepose")

    # 设置速度
    def set_speed(self, value="0.4"):
        json_dict = {
            "param": "base.max_moving_speed",
            "value": value
        }
        return self.http.post(endpoint="/api/core/system/v1/parameter", data=json_dict)
    
    # 遥控移动
    def remote_control_move(self, value=0):
        json_dict = {
            "action_name": "slamtec.agent.actions.MoveByAction",
            "options": {
                "direction": value
            }
        }
        return self.http.post(endpoint="/api/core/motion/v1/actions", data=json_dict)
    # 底盘移动到指定位置，point_name如果为None则按照point_pose坐标移动
    def move_point_by_name(self, point_name, yaw=0.0):
        json_dict = {  
            "action_name": "slamtec.agent.actions.MultiFloorMoveAction",  
            "options": {  
                "target": {  
                    "poi_name":point_name
                },  
                "move_options": {  
                    "mode": 0,  
                    "flags": [
                        "precise",
                        "with_yaw"
                    ],  
                    "yaw": yaw,  
                    "acceptable_precision": 0.1,  
                    "fail_retry_count": 3,  
                    "speed_ratio": 0.5
                }  
            }  
        }
        return self.http.post(endpoint="/api/core/motion/v1/actions", data=json_dict)        

    def move_point_by_pose(self,x=0.0,y=0.0,z=0.0,yaw=0.0):
        json_dict = {
            "action_name": "slamtec.agent.actions.MoveToAction",
            "options": {
                "target": {
                    "x": x,
                    "y": y,
                    "z": z
                },
                "move_options": {
                "mode": 0,
                "flags": [
                    "precise",
                    "with_yaw"
                ],
                "yaw": yaw,
                "acceptable_precision": 0.1,
                "fail_retry_count": 3,
                "speed_ratio": 0.5
                }
            }
        }
        return self.http.post(endpoint="/api/core/motion/v1/actions", data=json_dict)
