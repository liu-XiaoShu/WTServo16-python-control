#!/usr/bin/env python3 
# -*- coding:utf-8 -*-
#---------------- 伺服电机/舵机 控制 --------------------------------
# 功能:    通过串口及控制板子，指定的协议和控制板交互，从而控制舵机
# 时间:    2020-04-03
# 作者:    liushk
# 备注说明:该通讯协议为控制　WTServo16　16 路舵机控制板
#----------------------------------------------------------------------
import time
import re
import sys
import serial
import binascii

class ServoMotorControl:
    def __init__(self, serial_port="/dev/ttyUSB0"):
        """
            功能:   参数初始化,通讯协议初始化
            参数:   serial_port 串口号
            返回值: 无
        """
        #串口协议参数
        self.FRAME_HEADER = "FF"           #帧头
        self.CMD_SERVO_SPEED = "01"        #设置舵机速度
        self.CMD_SERVO_MOVE = "02"         #设置移动到的位置
        self.CMD_START_ACTION_GROUP = "09" #启动动作组
        self.CMD_STOP_RECOVERY ="0B"       #急停/恢复
        self.STATE_DETECTION ="00120000"   #握手指令/控制板状态检测指令
        self.STATE_ANSWER ="F0120000"      #应答指令

        #该舵机控制板最大控制16个舵机
        self.max_servo_num = 16

        #串口
        try:
            self.uart = serial.Serial(serial_port, 9600, timeout=2)
        except:
            print("\033[0;36;31m[舵机串口]: %s设备不存在!!\033[0m"%serial_port)
            sys.exit()

    def serial_send(self, send_cmd):
        """
            功能:   串口发送指令函数
            参数:   send_cmd 需要发送的指令
            返回值: bool: True/False
        """
        try:
            #print(str(send_cmd))
            self.uart.write(bytes.fromhex(send_cmd))  #16进制发送
            #print("\033[0;36;32m[舵机串口]: 指令发送成功!!\033[0m")
            return True
        except:
            print("\033[0;36;31m[舵机串口]: 指令发送失败!!\033[0m")
            return False

    def serial_receive(self, timeOut, InforToBeTested=0):
        """
           功能:   串口接收数据
           参数:   无
           返回值: 读取信息列表/False
        """
        count = 0
        serial_content_list = []
        try:
            while count < timeOut:
                serial_content = self.uart.readline().hex() #16进制读取数据
                if serial_content:
                    #print(serial_content)
                    serial_content_list.append(serial_content)
                    if InforToBeTested and re.findall(InforToBeTested, serial_content, re.I):
                        return ["InforToBeTested_FIND"]
                time.sleep(0.01)
                count += 1
            return serial_content_list
        except:
            print("\033[0;36;31m[舵机串口]: 串口信息接收失败!!\033[0m")
            return False


    def int_to_hexStr(self, int_data):
        """
            功能:   10进制转换为16进制字符
                    串,并把小于2位补0
            参数:   10进制数值
            返回值: 16进制字符串
        """
        hex_data = str(hex(int_data))
        if len(hex_data) < 4:
            hex_data = "0x0" +str(hex_data).split("0x")[-1]
        hex_data = hex_data.split("0x")[-1]
        return hex_data

    def str_to_hex(self, str_value):
        """
            功能:   把输入的八位2进制字
                    符串，转换为16进制字符串
            参数:   八位2进制字符串
            返回值: 16进制字符串
        """
        #字符串转10进制
        dec = int(str_value, 2)
        hex_data = None
        #10进制转16进制
        hex_data = hex(dec)
        if len(hex_data) < 4:
            hex_data =  "0x0" + hex_data.split("0x")[-1]
        hex_data = "0x" + hex_data.split("0x")[-1].upper()
        hex_data = hex_data.split("0x")[-1]
        return hex_data


    def get_low_high_byte(self, decimal_value, low_or_high):
        """
            功能:   获取输入参数的低八位,变换成
                    16进制字符串输出
            参数:   十进制的数值
            返回值: 低八位,十六进制字符串
        """
        low_byte_str = None
        high_byte_str = None
        ##转换为二进制
        binary = bin(decimal_value)
        binary = binary.split("0b")[-1]
        for i in range((16-len(binary))):
            binary = "0" + binary
        ##获取高8位
        if low_or_high == "high":
            high_byte_str = binary[0:8]
            return self.str_to_hex(high_byte_str)
        ##获取低8位
        else:
            low_byte_str = binary[8:16]
            return self.str_to_hex(low_byte_str)


    def ServoControlPanelStatus(self):
        """
            功能:   控制板状态检测:
            参数:   无
            返回值: bool: True/False 
                    OnLine/OffLine 在线/离线 
        """
        result = []
        PanelStatus_Cmd = self.FRAME_HEADER + self.STATE_DETECTION
        OnLine_Cmd = self.FRAME_HEADER + self.STATE_ANSWER
        self.serial_send(PanelStatus_Cmd)
        result = self.serial_receive(3, OnLine_Cmd)
        if result and str(result[0]) == "InforToBeTested_FIND":
            print("\033[0;36;32m[串口舵机]: 设备在线\033[0m")
            return True
        else:
            print("\033[0;36;31m[串口舵机]: 设备离线\033[0m")
            return False


    def MoveServo(self, servoID, move_angle, move_speed):
        """
            功能:   单个舵机转动
        　  参数:   servoID:    舵机编号 
                    move_angle: 转动角度
                                角度 0°~180°
                    move_speed: 转动速度 
                                单位 (9°/s),取值为 1~20
            返回值: 执行结果

            1 舵机的位置,单位 us (0.09°) 取值为 
              500~2500,舵机的控制脉宽
              是 500us~2500us,对应的角度 0°~180°
            2 转动速度单位 (9°/s),取值为 1~20
              对应: 9 ~180°
        """
        success_flag = 0
        print("\033[0;36;32m[串口舵机]: 舵机ID: %s 转动角度: %s 转速: %s\033[0m"%(servoID, move_angle, move_speed))
        servoID = int(servoID)
        move_angle = int(move_angle)
        move_speed = int(move_speed)
        if servoID < 0 or servoID > (self.max_servo_num - 1):
            print("\033[0;36;31m[串口舵机]: %d位舵机控制板,舵机ID应在:0~%d\033[0m"%(self.max_servo_num, (self.max_servo_num-1)))
            return False
        elif move_angle < 0 or move_angle > 180:
            print("\033[0;36;31m[串口舵机]: 舵机转角应在:0°~180°\033[0m")
            return False
        elif move_speed < 9 or  move_speed > 180:
            print("\033[0;36;31m[串口舵机]: 舵机转动速度应在:9°/s~180°/s\033[0m")
            return False

        #转动角度计算 (500us~2500us,对应的角度 0°~180, 单位 us (0.09°))

        #0度有些不准，微调
        if move_angle < 7:
            move_angle = 7
        move_angle = int(move_angle/0.09) + 500 #us

        #通讯指令拼装
        moveAngle_Cmd = ""        
        moveSeed_Cmd = ""

        #设置速度指令:
        moveSeed_Cmd = moveSeed_Cmd + str(self.FRAME_HEADER)    #填充帧头
        moveSeed_Cmd = moveSeed_Cmd + str(self.CMD_SERVO_SPEED) #填充速度控制指令
        moveSeed_Cmd = moveSeed_Cmd + str(self.int_to_hexStr(servoID)) #需要设置速度舵机id
        moveSeed_Cmd = moveSeed_Cmd + str(self.get_low_high_byte(int(move_speed/9), 'low')) #取速度低八位
        moveSeed_Cmd = moveSeed_Cmd + str(self.get_low_high_byte(int(move_speed/9), 'high'))#取速度高八位数

        #设置转动角度命令
        moveAngle_Cmd = moveAngle_Cmd + str(self.FRAME_HEADER)    #填充帧头
        moveAngle_Cmd = moveAngle_Cmd + str(self.CMD_SERVO_MOVE)    #填充帧头
        moveAngle_Cmd = moveAngle_Cmd + str(self.int_to_hexStr(servoID)) #需要转动舵机id
        moveAngle_Cmd = moveAngle_Cmd + str(self.get_low_high_byte(move_angle, 'low')) #取速度低八位
        moveAngle_Cmd = moveAngle_Cmd + str(self.get_low_high_byte(move_angle, 'high')) #取速度高八位

        #发送速度设置命令
        if self.serial_send(moveSeed_Cmd):
            success_flag += 1
        #发送移动命令
        if self.serial_send(moveAngle_Cmd):
            success_flag += 1
        if success_flag == 2:
            return True
        else:
            return False

    def EmergencyStopServo(self):
        """
            功能:   紧急停止舵机,主要是停止舵机动作组
                    单个舵机不能控制
            参数:   无
            返回值: 无
        """
        E_StopServo_Cmd = ""
        E_StopServo_Cmd = E_StopServo_Cmd + str(self.FRAME_HEADER) #填充帧头
        E_StopServo_Cmd = E_StopServo_Cmd + str(self.CMD_STOP_RECOVERY) #填充急停/恢复指令
        E_StopServo_Cmd = E_StopServo_Cmd + "000100" #填充急停指令
        self.serial_send(E_StopServo_Cmd)


    def RecoveryServoAction(self):
        """
            功能:   恢复舵机动作组, 单
                    个舵机不能控制
            参数:   无
            返回值: 无
        """
        Recovery_Servo_Cmd = ""
        Recovery_Servo_Cmd = Recovery_Servo_Cmd + str(self.FRAME_HEADER) #填充帧头
        Recovery_Servo_Cmd = Recovery_Servo_Cmd + str(self.CMD_STOP_RECOVERY) #填充急停/恢复指令
        Recovery_Servo_Cmd = Recovery_Servo_Cmd + "000100" #填充急停指令
        self.serial_send(Recovery_Servo_Cmd)


    def strHex_to_int(self, strHex_value_list):
        """
            功能:   把输入的16进制字符串列表，组成
                    16位2进制,转换为10进制
            参数:   16进制字符串
            返回值: 10进制
        """
        binary_list = []
        binary_data = None
        dec_data = 0
        for strHex_value in strHex_value_list:
            binary = bin(int(strHex_value, 16))
            binary = binary.split("0b")[-1]
            for i in range(8-len(binary)):
                binary = "0" + binary 
            binary_list.append(binary)
        binary_data = "".join(binary_list)
        dec_data = int(binary_data, 2)
        print(dec_data)
        return dec_data



if __name__=="__main__":
    sm = ServoMotorControl()
    if len(sys.argv) < 3:
        print("python3 new_servo_motor_control.py 舵机id 转动角度, 转动速度")
        sys.exit()
    else:
        servoID = sys.argv[1]
        move_angle = sys.argv[2]
        move_speed = sys.argv[3]
        sm.MoveServo(servoID, move_angle, move_speed)


