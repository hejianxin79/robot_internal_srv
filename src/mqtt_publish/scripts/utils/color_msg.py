#! /usr/bin/env python3

import time

class ColorMsg():
    def __init__(self,msg: str, color: str = '', timestamp: bool = True):
        self.msg = msg
        self.color = color
        self.timestamp = timestamp
        self.colorMsg()
    def colorMsg(self):
        str = ""
        if self.timestamp:
            str += time.strftime('%Y-%m-%d %H:%M:%S',
                                time.localtime(time.time())) + "  "
        if self.color == "red":
            str += "\033[1;31;40m"
        elif self.color == "green":
            str += "\033[1;32;40m"
        elif self.color == "yellow":
            str += "\033[1;33;40m"
        else:
            print(str + self.msg)
            return
        str += self.msg + "\033[0m"
        print(str)