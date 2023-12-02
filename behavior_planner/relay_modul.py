import serial
import time

"""
@autor keisuke yoshida
------------------------------------------------
リレー接続用プログラム

送信 : True(リレー接続) or False(緊急停止)

[関数]
send_relay_command  : リレーの状態送信用(serial通信)
"""

class Main_Circuit_Control:
        def __init__(self):
            self.ser = serial.Serial('/dev/ttyUSB-arduino-power', 115200)
            time.sleep(2) # Waiting time for port connection
 
        def send_relay_command(self,mode):
            if mode :
                send_data = '1'
                print("回路接続")
            else :
                send_data = '0'
                print("回路遮断")  
            self.ser.write(send_data.encode())
            self.ser.flush() 
            response = self.ser.readline().strip().decode('utf-8')  
            print(response)

# debag
# msc = Main_Circuit_Control()
# mode = True
# msc.send_relay_command(mode)