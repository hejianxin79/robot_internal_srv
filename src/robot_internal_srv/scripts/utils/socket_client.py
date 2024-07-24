import socket, json
from color_msg import ColorMsg


class SocketClient:
    def __init__(self):
        self.host = "192.168.11.38"
        self.port = 20008
        # 创建一个socket对象  
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        

    
    def send(self, range=0):  
        try:
            # 连接到服务器  
            self.client.connect((self.host, self.port))  
            message = json.dumps(self.send_message(range=range))

            # 发送一些数据  
            send_msg = message.encode('utf-8')  # 需要是bytes类型  
            self.client.sendall(send_msg)  

            # 接收来自服务器的数据  
            data = self.client.recv(1024)  # 最多接收1024字节的数据  
        finally:  
            # 关闭连接  
            self.client.close()
        return data.decode("utf-8")

    def send_message(self,range=0.00):
        '''
        {
            "TCP_Send": "MotorDrive_Send.json",
            "TCP_Send_Motor": {
                "Motor_pos_ref" : 0,        "//": "-11.36~318.64,单位mm",
                "Calibration"	: 0,  "//":	"0:无动作。1：自动找原点。其它值：将当前位置设置为所发送数值"
            }
        }
        '''
        msg =  {
            "TCP_Send": "MotorDrive_Send.json",
            "TCP_Send_Motor": {
                "Motor_pos_ref" : int(range),        # "-11.36~318.64,单位mm",
                "Calibration"	: 0,  #	"0:无动作。1：自动找原点。其它值：将当前位置设置为所发送数值"
            }
        }
        print("--------------------------")
        print(msg)
        return msg
  
