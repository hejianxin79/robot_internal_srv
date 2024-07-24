#! /usr/bin/env python3
import sys,json,time,os,requests
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from color_msg import ColorMsg
class HTTPRequest:  
    def __init__(self):  
        self.base_url = "http://192.168.11.1:1448"
        self.headers = {  
            'Content-Type': 'application/json', 
        }  
  
    def _make_request(self, method, endpoint, print_out=True, **kwargs):  
        """  
        辅助方法，用于发送 HTTP 请求  
        :param method: 请求方法（如 'get', 'post', 'put', 'delete'）  
        :param endpoint: 请求的端点  
        :param kwargs: 其他请求参数  
        :return: 响应内容  
        """  
        url = self.base_url + endpoint  
        response = requests.request(method, url, headers=self.headers, **kwargs)
        if print_out == True:
            msg = f"请求类型:{method}  请求地址:{url}"
            ColorMsg(msg=msg, color="yellow")
        if response.status_code == 200:
            if response.text != "":
                return response.json(), response.status_code
            else:
                return "OK",response.status_code
        else:
            return response.text,response.status_code
  
    def get(self, endpoint, params=None, print_out=True):  
        """  
        发送 GET 请求  
        :param endpoint: 请求的端点  
        :param params: 请求参数，以字典形式传递  
        :return: 响应内容  
        """  
        return self._make_request('get', endpoint, params=params, print_out=print_out)  
  
    def post(self, endpoint, data=None, print_out=True):  
        """  
        发送 POST 请求  
        :param endpoint: 请求的端点  
        :param data: 请求体数据，以字典形式传递  
        :return: 响应内容  
        """  
        return self._make_request('post', endpoint, json=data, print_out=print_out)  
  
    def put(self, endpoint, data=None, print_out=True):  
        """  
        发送 PUT 请求  
        :param endpoint: 请求的端点  
        :param data: 请求体数据，以字典形式传递  
        :return: 响应内容  
        """  
        return self._make_request('put', endpoint, json=data, print_out=print_out)  
  
    def delete(self, endpoint, print_out=True):  
        """  
        发送 DELETE 请求  
        :param endpoint: 请求的端点  
        :return: 响应内容  
        """  
        return self._make_request('delete', endpoint, print_out)
