import serial
import time
from CalcLidarData import CalcLidarData

ser = serial.Serial(port='/dev/ttyS0',
                    baudrate=230400,
                    timeout=5.0,
                    bytesize=8,
                    parity='N',
                    stopbits=1)

tmpString = ""
points_in_one_revolution = 0
start_time = time.time()
revolutions = 0
revolution_started = False

while True:
    loopFlag = True
    flag2c = False

    while loopFlag:
        b = ser.read()
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

            # Increment the points counter for each point in the lidar data
            points_in_one_revolution += len(lidarData.Angle_i)

            tmpString = ""
            loopFlag = False

        else:
            tmpString += b.hex() + " "

        flag2c = False

    # Check if one revolution is complete (360 degrees)
    if 'lidarData' in locals():
        if lidarData.Angle_i[-1] < lidarData.Angle_i[0]:
            if revolution_started:
                revolutions += 1
                print(f"Revolution {revolutions}: Points in one revolution: {points_in_one_revolution}")

                # Reset points counter for the next revolution
                points_in_one_revolution = 0
            revolution_started = True
        else:
            revolution_started = False

        elapsed_time = time.time() - start_time

        # Check if 1 second has passed
        if elapsed_time >= 1.0:
            print(f"Processing frequency: {revolutions} revolutions per second")

            # Reset counters and timer
            revolutions = 0
            start_time = time.time()