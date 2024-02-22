import cv2
import numpy as np
import time
from threading import Thread
import multiprocessing
import queue
# Import packages
import os
import argparse
import sys
import importlib.util
from ssdDetect import polygon_calculate
import threading
import json

from video_stream import VideoStream

# Import lib Lidar
import serial
from CalcLidarData import CalcLidarData
from lidar_stream import lidarStream

import RPi.GPIO as GPIO
import time

OUTPUT_LEDS = [0,0,0]  

global result_queue_cam
CHECK_CAM = False

# Create a tracker based on tracker name
trackerTypes = ['BOOSTING', 'MIL', 'KCF','TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']
def createTrackerByName(trackerType):

    if trackerType == trackerTypes[0]:
        tracker = cv2.TrackerBoosting_create()
    elif trackerType == trackerTypes[1]:
        tracker = cv2.TrackerMIL_create()
    elif trackerType == trackerTypes[2]:
        tracker = cv2.TrackerKCF_create()
    elif trackerType == trackerTypes[3]:
        tracker = cv2.TrackerTLD_create()
    elif trackerType == trackerTypes[4]:
        tracker = cv2.TrackerMedianFlow_create()
    elif trackerType == trackerTypes[5]:
        tracker = cv2.TrackerGOTURN_create()
    elif trackerType == trackerTypes[6]:
        tracker = cv2.TrackerMOSSE_create()
    elif trackerType == trackerTypes[7]:
        tracker = cv2.TrackerCSRT_create()
    else:
        tracker = None
        print('Incorrect tracker name')
        print('Available trackers are:')
        for t in trackerTypes:
            print(t)
    return tracker


def detect_camera(videostream,imW,imH,camera_thread_event):
    global result_queue_cam
    global CHECK_CAM
    # ... (your existing code for camera detection)
    # Assuming PointsInfor is the result from camera detection
    MODEL_NAME = './All_Model_detect/Sample_TFLite_model'
    GRAPH_NAME = "detect.tflite"
    LABELMAP_NAME = "labelmap.txt"
    min_conf_threshold = float(0.5)
    JSON_PATH = 'polygon.json'

    pkg = importlib.util.find_spec('tflite_runtime')
    if pkg:
        from tflite_runtime.interpreter import Interpreter
    else:
        from tensorflow.lite.python.interpreter import Interpreter

    #Get path to current working directory
    CWD_PATH = os.getcwd()
    # path json polygon
    JSON_PATH = os.path.join(CWD_PATH,JSON_PATH)
    print("JSON path : ",JSON_PATH)

    PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,GRAPH_NAME)
    PATH_TO_LABELS = os.path.join(CWD_PATH,MODEL_NAME,LABELMAP_NAME)

    # Load the label map
    with open(PATH_TO_LABELS, 'r') as f:
        labels = [line.strip() for line in f.readlines()]

    if labels[0] == '???':
        del(labels[0])
    
    # print(labels,'\n')
    interpreter = Interpreter(model_path=PATH_TO_CKPT)

    interpreter.allocate_tensors()

    # Get model details
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    height = input_details[0]['shape'][1]
    width = input_details[0]['shape'][2]

    floating_model = (input_details[0]['dtype'] == np.float32)

    input_mean = 127.5
    input_std = 127.5
    limit_area = 10000

    # Check output layer name to determine if this model was created with TF2 or TF1,
    # because outputs are ordered differently for TF2 and TF1 models
    outname = output_details[0]['name']

    if ('StatefulPartitionedCall' in outname): # This is a TF2 x
        boxes_idx, classes_idx, scores_idx = 1, 3, 0
    else: # This is a TF1 model
        boxes_idx, classes_idx, scores_idx = 0, 1, 2
    
    # Get Polygon_calculate
    polygon_cal = polygon_calculate(JSON_PATH,imW,imH)
    # detect frame return boxes
    def detect_ssd(frame):
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (width, height))
        input_data = np.expand_dims(frame_resized, axis=0)
        # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
        if floating_model:
            input_data = (np.float32(input_data) - input_mean) / input_std
        # Perform the actual detection by running the model with the image as input
        interpreter.set_tensor(input_details[0]['index'],input_data)
        interpreter.invoke()
        # Retrieve detection results
        boxes = interpreter.get_tensor(output_details[boxes_idx]['index'])[0] # Bounding box coordinates of detected objects
        classes = interpreter.get_tensor(output_details[classes_idx]['index'])[0] # Class index of detected objects
        scores = interpreter.get_tensor(output_details[scores_idx]['index'])[0] # Confidence of detected objects
        #print("shape Detect : ",boxes.shape,classes.shape,scores.shape,"\n")
        # print("class : ",classes,"\n")
        boxes_new = []
        classes_new = []
        scores_new = []
        centroid_new = []

        class_checks = [0,1,2,3,5,6,10,15,16,17,18,19,20,21,22]
        #class_checks = [0,3,4]
        for i in range(len(scores)):
            if classes[i] in class_checks:
                if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0)):
                    
                    # scale boxes - values (0,1) to size width height
                    ymin = int(max(1,(boxes[i][0] * imH)))
                    xmin = int(max(1,(boxes[i][1] * imW)))
                    ymax = int(min(imH,(boxes[i][2] * imH)))
                    xmax = int(min(imW,(boxes[i][3] * imW)))
                    
                    # if(polygon_cal.area_box((xmin,ymin,xmax,ymax),limit_area)):
                    centroid_new.append([int((xmin+xmax)//2),int((ymin+ymax)//2)])
                    boxes_new.append((xmin,ymin,xmax,ymax))
                    classes_new.append(classes[i])
                    scores_new.append(scores[i])
        # print("SHAPE CALASS: ",classes_new,"|||", scores_new)
        return boxes_new,classes_new,scores_new,centroid_new

    boxes, classes,scores ,centroids_old = [],[],[],[]

    trackerType = trackerTypes[4]  
    # Create MultiTracker object
    multiTracker = cv2.MultiTracker_create()
    count = 0
    num_frame_to_detect = 14
 
    while(True):
        # start_time = time.time()

        # Acquire frame and resize to expected shape [1xHxWx3]
        frame = videostream.read()

        frame_old, frame = polygon_cal.cut_frame_polygon(frame)

        # get updated location of objects in subsequent frames
        success, boxes_update = multiTracker.update(frame)

        if count == num_frame_to_detect:
            controids = polygon_cal.centroid(boxes_update)
            PointsInfor = polygon_cal.check_result(controids,centroids_old)
            # print(f"Information point:{PointsInfor}  \n")
            result_queue_cam = PointsInfor
            CHECK_CAM = True
            count = 0
            camera_thread_event.set()

        if count == 0:
            boxes, classes,scores,centroids_old = detect_ssd(frame)
            multiTracker = cv2.MultiTracker_create()
            # Initialize MultiTracker
            for bbox in boxes:
                box_track = (bbox[0],bbox[1],bbox[2]-bbox[0],bbox[3]-bbox[1])
                multiTracker.add(createTrackerByName(trackerType), frame, box_track)
    
            # if len(scores)==0:
            #     count=-1
        count+=1
        # end_time = time.time()
        # elapsed_time = end_time - start_time
        # print(f"thread camera {elapsed_time:.2f} seconds. ")
        # print(count)
        # frame = polygon_cal.draw_polygon(frame)
        # frame = cv2.circle( frame, (polygon_cal.points['right_check'][0], polygon_cal.points['right_check'][1]), 5, (0,255,255), -1)
        # frame = cv2.circle( frame, (polygon_cal.points['left_check'][0], polygon_cal.points['left_check'][1]), 5, (255,0,255), -1)
        # # frame = cv2.resize(frame, (1080, 720))
        cv2.imshow('Object detector 1', frame)


def COntrol_leds(num_led_right=19, num_led_left=6, num_led_warning=13):
    global OUTPUT_LEDS

    GPIO.setmode(GPIO.BCM)
    # GPIO.setwarnings(False)
    # # num_led_right: ID of GPIO pin (Ex: num_led_right = 17 => GPIO17)
    # # num_led_left: ID of GPIO pin (Ex: num_led_left = 27 => GPIO27)
    # # num_led_warning: ID of GPIO pin (Ex: num_led_warning = 22 => GPIO22)
    # # led_1: integer signal of LED 1 from pip num_led_right 
    # # led_2: integer signal of LED 2 from pip num_led_left 
    # # les_3: integer signal of LED 3 from pip num_led_warning 
    GPIO.cleanup()
    GPIO.setup(num_led_right,GPIO.OUT)
    GPIO.setup(num_led_left,GPIO.OUT)
    GPIO.setup(num_led_warning,GPIO.OUT)
    # # print("Run")

    GPIO.output(num_led_right,GPIO.LOW)
    GPIO.output(num_led_left,GPIO.HIGH)
    GPIO.output(num_led_warning,GPIO.LOW)


    while True:
        # GPIO.output(num_led_right,GPIO.HIGH)
        # GPIO.output(num_led_left,GPIO.LOW)
        # GPIO.output(num_led_warning,GPIO.HIGH)
        # OUTPUT_LEDS = [1,1,1]
        # print(OUTPUT_LEDS)
        time.sleep(0.25) 
        start_time = time.time()
        if sum(OUTPUT_LEDS) > 0:
            if OUTPUT_LEDS[0] > 0:
                GPIO.output(num_led_right,GPIO.HIGH) #GPIO.LOW
            else: 
                GPIO.output(num_led_right,GPIO.LOW)
            if OUTPUT_LEDS[1] > 0:
                GPIO.output(num_led_left,GPIO.LOW)
            else: 
                GPIO.output(num_led_left,GPIO.HIGH)
            if OUTPUT_LEDS[2] > 0:
                GPIO.output(num_led_warning,GPIO.HIGH)
            else: 
                GPIO.output(num_led_warning,GPIO.LOW)
            if max(OUTPUT_LEDS) > 1:
                time.sleep(0.15)
                if OUTPUT_LEDS[0] > 1:
                    GPIO.output(num_led_right,GPIO.LOW)
                if OUTPUT_LEDS[1] > 1:
                    GPIO.output(num_led_left,GPIO.HIGH)
                if OUTPUT_LEDS[2] > 1:
                    GPIO.output(num_led_warning,GPIO.LOW)
        else:
            GPIO.output(num_led_right,GPIO.LOW)
            GPIO.output(num_led_left,GPIO.HIGH)
            GPIO.output(num_led_warning,GPIO.LOW)
        end_time = time.time()
        # elapsed_time = end_time - start_time
        # print(f"thread led {elapsed_time:.2f} seconds")
    

 # [RIGHT,LEFT,Warning]  on:1 off:0: blick : 2

def check_array(old_arr,new_arr,threshold_lidar):
    dem = 0
    for i in range(min(len(old_arr),len(new_arr))):

        if abs(old_arr[i]-new_arr[i]) >  5:
            dem+=1
            if dem >3: return True
        else:
            dem=0

    return False
    

def main_process():
    global OUTPUT_LEDS

    INDEX_CHECK = 0
    NUM_Check_Lidar = 5

    NUM_CHECK_WARNING = 5
    INDEX_WARNING= 0

    OFF_THRESHOLD = 1
    ON_THRESHOLD = 3
    # LIDAR_THRESHOLD = 02.5
    # WARNING_THRESHOLD = 0.6

    CHECK_FRAME_LIDAR = np.zeros(NUM_Check_Lidar)
    CHECK_FRAME_LEFT = np.zeros(NUM_Check_Lidar)
    CHECK_FRAME_RIGHT = np.zeros(NUM_Check_Lidar)


    CHECK_FRAME_FORBIDDEN_LEFT = np.zeros(NUM_CHECK_WARNING)
    CHECK_FRAME_FORBIDDEN_RIGHT = np.zeros(NUM_CHECK_WARNING)
    CHECK_FRAME_FREEZE = np.zeros(NUM_CHECK_WARNING)

    global CHECK_CAM
    global result_queue_cam

    # Connect Cam
    VIDEO_PATH = 'rtsp://admin2:Atlab123@@192.168.1.64:554/Streaming/Channels/101'
    # VIDEO_PATH = '/home/pi/Projects/Roadtrafficvideo2.mp4'

    imW,imH = 1280,720
    videostream = VideoStream(resolution=(imW,imH),framerate=25,STREAM_URL= VIDEO_PATH).start()
    #videostream = cv2.VideoCapture(VIDEO_PATH)
    time.sleep(1)


    Lidar_path_json = 'lidar_infor.json'
    with open(Lidar_path_json,'r') as json_file:
        Lidar_infor = json.load(json_file)

    angle_min = Lidar_infor['angle_min']
    angle_max = Lidar_infor['angle_max']
    total_points = Lidar_infor['Len_points']
    Lidar_ponts = Lidar_infor['points']


    camera_thread_event = threading.Event()
    # Create and start the camera detection thread
    camera_thread = Thread(target=detect_camera, args=(videostream,imW,imH,camera_thread_event))


    # Create and start the lidar detection thread
    lidarstream = lidarStream(port_serial='/dev/ttyS0',baudrate=230400,
                                 bytesize=8 , stopbits=1,
                                 angle_min = angle_min, 
                                 angle_max = angle_max,
                                 total_points = total_points)


    control_leds_thread = Thread(target=COntrol_leds)
    control_leds_thread.start()
    camera_thread.start()

    All_result = {}
    All_result['Lidar_infor'] = False
    
    index_count = 0
    start_time = time.time()
    while True:
        
        if CHECK_CAM:
            
            # read out camera and lidar => create result ALL
            camera_result = result_queue_cam
            All_result = {
                'Left':camera_result['Left'],
                'Right':camera_result['Right'],
                'Forbidden_right' : camera_result['Forbidden_right'],
                'Forbidden_left'  : camera_result['Forbidden_left'],
                'freeze' : camera_result['freeze']
            }

            if not camera_result['Left'] or not camera_result['Right']:
                lidar_self = lidarstream.start()
                while not lidar_self.stopped:
                    time.sleep(0.01)
                lidar_result = lidar_self.read()
                    
                if check_array(Lidar_ponts,lidar_result,3):
                    All_result['Lidar_infor'] = True
                else:
                    All_result['Lidar_infor'] = False
            else:
                All_result['Lidar_infor'] = True
      
            print(All_result)
            # Quantity statistics lidar infor and camera infor
            if All_result['Lidar_infor']:
                CHECK_FRAME_LIDAR[INDEX_CHECK] = 1 
            else:
                CHECK_FRAME_LIDAR[INDEX_CHECK] = 0
            if All_result['Left']:
                CHECK_FRAME_LEFT[INDEX_CHECK] = 1
            else:
                CHECK_FRAME_LEFT[INDEX_CHECK] = 0
            if All_result['Right']:
                CHECK_FRAME_RIGHT[INDEX_CHECK] = 1
            else:
                CHECK_FRAME_RIGHT[INDEX_CHECK] = 0  
            if INDEX_CHECK >= NUM_Check_Lidar -1:
                INDEX_CHECK = 0
            else:
                INDEX_CHECK+=1

            if All_result['Forbidden_left']:
                CHECK_FRAME_FORBIDDEN_LEFT[INDEX_WARNING] = 1
            else:
                CHECK_FRAME_FORBIDDEN_LEFT[INDEX_WARNING] = 0
            if All_result['Forbidden_right']:
                CHECK_FRAME_FORBIDDEN_RIGHT[INDEX_WARNING] = 1
            else:
                CHECK_FRAME_FORBIDDEN_RIGHT[INDEX_WARNING] = 0

            if All_result['freeze']:
                CHECK_FRAME_FREEZE[INDEX_WARNING] = 1
            else:
                CHECK_FRAME_FREEZE[INDEX_WARNING] = 0

            INDEX_WARNING+=1
            if INDEX_WARNING >= NUM_CHECK_WARNING :
                INDEX_WARNING = 0
                

            # print("All resuklt : ",All_result')
            #  export OUT_LED
            TEMP_LED = [0,0,0]

            if sum(CHECK_FRAME_FORBIDDEN_RIGHT)>= ON_THRESHOLD :
                TEMP_LED[0] = 2
            else:
              if sum(CHECK_FRAME_RIGHT)>= ON_THRESHOLD :
                TEMP_LED[0] = 1
            if sum(CHECK_FRAME_RIGHT)  <= OFF_THRESHOLD :
                TEMP_LED[0] = 0 

            if sum(CHECK_FRAME_FORBIDDEN_LEFT) >= ON_THRESHOLD  :
                TEMP_LED[1] = 2
            else:
                if sum(CHECK_FRAME_LEFT)  >= ON_THRESHOLD  :
                    TEMP_LED[1] = 1
            if sum(CHECK_FRAME_LEFT) <= OFF_THRESHOLD :
                TEMP_LED[1] = 0 


            if sum(CHECK_FRAME_FREEZE)  >= ON_THRESHOLD :
                TEMP_LED[2] = 2
            else:             
                if  sum(CHECK_FRAME_LIDAR) >= ON_THRESHOLD and OUTPUT_LEDS[0] ==0 and OUTPUT_LEDS[1] == 0:
                    TEMP_LED[2] = 1
                else:
                    if  sum(CHECK_FRAME_LIDAR) <= OFF_THRESHOLD:
                        TEMP_LED[2] = 0
                
            OUTPUT_LEDS = TEMP_LED
            print(f"Infor : {TEMP_LED} {index_count}" ) 

            index_count+=1
            camera_thread_event.clear()
            CHECK_CAM = False
            end_time = time.time()
            elapsed_time = end_time - start_time
            print(f"thread main {elapsed_time:.2f} seconds \n")     
            start_time = time.time()   

        time.sleep(0.05)


    # Clean up
    videostream.stop()
    lidarstream.stop()

if __name__ == "__main__":
    # Run the main process
    main_process()
