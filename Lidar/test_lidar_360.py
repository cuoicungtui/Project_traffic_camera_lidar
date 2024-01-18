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
total_distance = 0
start_time = time.time()
revolutions = 0

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

            # Summing up the distances
            total_distance += sum(lidarData.Distance_i)

            tmpString = ""
            loopFlag = False

        else:
            tmpString += b.hex() + " "

        flag2c = False

    if 'lidarData' in locals():
        # Check if one revolution is complete (360 degrees)
        if sum(lidarData.Angle_i) >= 360.0:
            revolutions += 1
            print(f"Revolution {revolutions}: Total Distance: {total_distance} units")

            # Reset total distance for the next revolution
            total_distance = 0

    elapsed_time = time.time() - start_time

    # Check if 1 second has passed
    if elapsed_time >= 1.0:
        print(f"Processing frequency: {revolutions} revolutions per second")

        # Reset counters and timer
        revolutions = 0
        start_time = time.time()