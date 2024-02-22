from threading import Thread
import multiprocessing
import serial
from CalcLidarData import CalcLidarData
import time

class lidarStream:
    """ lidar object that read and update data for angle and distance"""
    def __init__(self,port_serial='/dev/ttyS0',baudrate=230400, bytesize=8 , stopbits=1,angle_min = 0, angle_max = 360,total_points = 100):
        self.ser = serial.Serial(port= port_serial,
                    baudrate=baudrate,
                    timeout=5.0,
                    bytesize=bytesize,
                    parity='N',
                    stopbits=stopbits)
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.total_points = total_points
        self.stopped = False
        self.result_lidar = list()
        self.time = 0
        self.ser.close()

    def start(self):
        self.stopped = False
        Thread(target= self.update , args =()).start()
        return self

    def update(self):

        self.ser.open()
        start_time = time.time()
        # self.ser.f+self.ser.read()self.ser.read()
        i=0
        tmpString = ""
        angle_old = 0
        self.result_lidar = list(range(self.total_points))
        Index_list = 0
        while True:
            loopFlag = True
            flag2c = False

            if self.stopped:
                break 
            
            while loopFlag:
                b = self.ser.read()
                tmpInt = int.from_bytes(b, 'big')

                if tmpInt == 0x54:
                    tmpString += b.hex() + " "
                    flag2c = True
                    continue

                elif tmpInt == 0x2c and flag2c:
                    tmpString += b.hex()

                    if not len(tmpString[0:-5].replace(' ', '')) == 90:
                        tmpString = ""
                        loopFlag = False
                        flag2c = False
                        continue

                    lidarData = CalcLidarData(tmpString[0:-5])
                    # print(f"lidar information: Distance{lidarData.Distance_i} angle : {lidarData.Angle_i} \n")
                    # print(f"lidar information: Sum Distance{sum(lidarData.Distance_i)} len_distance {len(lidarData.Distance_i)} \n")
                    # print(f"lidar information: start {lidarData.Angle_i[0]} end {lidarData.Angle_i[11]} len_distance {len(lidarData.Distance_i)} \n")
                    # add point lidar
                    
                    
                    for angle,distance in zip(lidarData.Angle_i,lidarData.Distance_i):
                        if  angle > self.angle_min and angle< self.angle_max:
                            if  angle > angle_old:
                                self.result_lidar[Index_list] = distance
                                Index_list+=1
                                angle_old = angle
                            else:

                                Index_list = 0
                                self.result_lidar[Index_list] = distance
                                Index_list+=1
                                angle_old = angle

                    tmpString = ""
                    loopFlag = False


                else:
                    tmpString += b.hex() + " "

                flag2c = False
            i+=1
            if i%80 ==0:
                end_time = time.time()
                self.time = end_time - start_time
                start_time = time.time()
                print(f"thread lidar {self.time:.2f} seconds")  
                self.ser.close()
                self.stop()


    def read(self):
        # print("Bytes waiting in inut buffer",self.ser.in_waiting)
        return self.result_lidar[:self.total_points]
    
    def stop(self):
        self.stopped = True