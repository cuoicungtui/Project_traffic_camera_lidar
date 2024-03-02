import serial
from CalcLidarData import CalcLidarData
import time
import json
import numpy as np


ser = serial.Serial(port='/dev/ttyS0',
                    baudrate=230400,
                    timeout=5.0,
                    bytesize=8,
                    parity='N',
                    stopbits=1)

tmpString = ""

i = 0
angle_old = 0
list_lidar_point = np.zeros(1000)
angle_min = 260.0
angle_max = 280.0

data = {
    "angle_min" : angle_min,
    "angle_max" : angle_max,
    "Len_points": 0,
    "Sum_distance": 0,
    "points" :list_lidar_point.tolist()
    }


index_lidar = 0
start_time = time.time()
while True:
# while time.time() - start_time < 1:
    loopFlag = True
    flag2c = False

    while loopFlag:
      
        b = ser.read()
        tmpInt = int.from_bytes(b, 'big')
        # print("read")

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
            print(f"lidar information: Distance{lidarData.Distance_i} angle : {lidarData.Angle_i} \n")
            print(f"lidar information: Sum Distance{sum(lidarData.Distance_i)} len_distance {len(lidarData.Distance_i)} \n")
            # print(f"lidar information: start {lidarData.Angle_i[0]} end {lidarData.Angle_i[11]} len_distance {len(lidarData.Distance_i)} \n")
            # add point lidar
            
            for angle,distance in zip(lidarData.Angle_i,lidarData.Distance_i):
                if angle > angle_min and angle < angle_max:
                    if  angle > angle_old :
                        list_lidar_point[index_lidar] = distance
                        angle_old = angle
                        index_lidar+=1
                    else:
                        print(f"Length lidar distance : {len(list_lidar_point)} Sum distance {sum(list_lidar_point)} \n")
                        print(list_lidar_point[:index_lidar],'\n')

                        data['Len_points'] = max(len(list_lidar_point),data['Len_points'])
                        data['Sum_distance'] = max(sum(list_lidar_point),data['Sum_distance'])
                        if data['Sum_distance'] == sum(list_lidar_point):
                            
                        # list_lidar_point.clear()
                            data['points'] = list_lidar_point[:index_lidar].tolist()
                        index_lidar = 0
                        list_lidar_point[index_lidar] = distance
                        angle_old = angle
                # if angle > angle_old:
                #     angle_old = angle
                # if angle < angle_old:
                #     print("end angle ",angle_old)
                #     end_time = time.time()
                #     elapsed_time = end_time - start_time
                #     print(f"thread lidar {elapsed_time:.2f} seconds")     
                #     start_time = time.time()   
                #     angle_old = angle
                #     print("start angle ",angle)
                #     print("i = ",i)

            tmpString = ""
            loopFlag = False

        else:
            tmpString += b.hex() + " "
        flag2c = False
       

    i += 1
    if i == 50000:
        # end_time = time.time()
        # elapsed_time = end_time - start_time
        # print(f"thread led {elapsed_time:.2f} seconds")
        break
# print("i = ",i)
file_path = "lidar_infor.json"
with open(file_path,'w') as json_file:
    json.dump(data,json_file,indent=2)

    # print(f"Total Distance: {total_distance} units")  # Output total distance