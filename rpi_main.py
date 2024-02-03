######## Webcam Object Detection Using Tensorflow-trained Classifier #########
#

# This code is based off the TensorFlow Lite image classification example at:
# https://github.com/tensorflow/tensorflow/blob/master/tensorflow/lite/examples/python/label_image.py
#

# Import packages
import os
import argparse
import cv2
import numpy as np
import sys
import pdb
import time
import pathlib
from threading import Thread
import multiprocessing as mp
import importlib.util
import datetime
from datetime import date
from datetime import datetime
import time
import io
import pickle
import pyrebase 

###### GPS #########
import pynmea2
import serial
ser = serial.Serial('/dev/serial0', 9600, timeout=1.0)
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))

########### GPIOs and Display ###########
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
from rpi_lcd import LCD
lcd = LCD()
disp = ""

######### rotary encoder setup #########
sw = 11
dt = 13
clk = 15
GPIO.setup(dt, GPIO.IN)
GPIO.setup(sw, GPIO.IN)
GPIO.setup(clk, GPIO.IN)
I_State = GPIO.input(clk)

sw_state = True #switch output is 0 when pressed 

bus_ID = "Test"

###### firebase config #############
# Enter the required details here
fb_gps_config = {
  "apiKey": "",
  "authDomain": "",
  "databaseURL": "",
  "projectId": "",
  "storageBucket": "",
  "messagingSenderId": "",
  "appId": "6",
  "measurementId": ""
}

fb_gps =  pyrebase.initialize_app(fb_gps_config)
db_gps = fb_gps.database()
storage= fb_gps.storage()

####################################################


# Load previous bus route for convenience
with open('previous_route.pickle','rb') as f:
    counter = pickle.load(f)
#some common bus routes in Mauritius
bus_routes = ("15 Flacq", "15 Rose-Hill", "16 Flacq", "16 Rose-Hill","194 Port-Louis", "194 Rose-Hill")
bus_route = bus_routes[counter]

latitude = 0.0
longitude =0.0
speed = 0.0
bus_speed = 0.0

#widths in cm
car_width = 173 #https://mechanicbase.com/cars/average-car-width/
bus_width = 255 #https://www.dimensions.com/collection/buses
truck_width = 200 #https://www.dimensions.com/collection/pickup-trucks
person_width = 41 #https://criticalbody.com/average-shoulder-width/
rider_width = 41 # same as human shoulder width 
motor_width = 75 #https://powersportsguide.com/average-motorcycle-dimensions/#:~:text=mudguards%20and%20taillights.-,How%20Wide%20is%20the%20Average%20Motorcycle%3F,typically%2030%2D35%20inches%20wide.

k_pixel = 1.135*1050

pix_width_lane1 = 425
pix_width_lane2 = 854

#max_stop_distance = 7400 # in cm
#mid_stop_distance = 5000



#max_stop_distance = ((v * 1000 /3600)^2 / (2*a) )+ reaction_time*v
def stop_distance(bus_speed):
    brake_deceleration = 7 # m/(s^2)
    reaction_time = 2 # in seconds
    
    min_stop_distance = ((bus_speed**2 / ( 2*brake_deceleration) )+ reaction_time*bus_speed)
    
    return min_stop_distance
    

dangerous_state = False
slow_state = False
slow_timing = 0
normal_state = True
not_normal_timing = 0

#Function to receive GPS cooridnates from the GPS sensor
def gps_receive():

    while 1:
        try:
            line = sio.readline()
            
            msg = pynmea2.parse(line)
            #print(repr(msg))
            if msg.sentence_type == 'GGA':
                latitude.value = msg.latitude
                longitude.value =  msg.longitude
                #print("Latitude= ", latitude)
                gps_data = {"lat": msg.latitude, "lon":msg.longitude}
                db_gps.child("GPS").child(bus_route).update(gps_data)
                #print("Longitude= ", longitude)
            elif msg.sentence_type == 'VTG':
                
                speed.value= float(msg.spd_over_grnd_kmph or 0.0)  #none type to 0
                
                #print("Speed over ground km/h= ",bus_speed)
            
        except serial.SerialException as e:
            #print('Device error: {}'.format(e))
            latitude.value = 0
            longitude.value = 0
            speed.value= 0.0
            break
        except pynmea2.ParseError as e:
            #print('Parse error: {}'.format(e))
            latitude.value = 0
            longitude.value = 0
            speed.value= 0.0
            continue

gps_process = mp.Process(target=gps_receive) 

# Define VideoStream class to handle streaming of video from webcam in separate processing thread
# Source - Adrian Rosebrock, PyImageSearch: https://www.pyimagesearch.com/2015/12/28/increasing-raspberry-pi-fps-with-python-and-opencv/

class VideoStream:
    """Camera object that controls video streaming from the Picamera or usb camera"""
    def __init__(self,resolution=(640,480),framerate=30): #initial 640 480
        # Initialize the PiCamera and the camera image stream
        self.stream = cv2.VideoCapture(0)
        ret = self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        ret = self.stream.set(3,resolution[0])
        ret = self.stream.set(4,resolution[1])
            
        # Read first frame from the stream
        (self.grabbed, self.frame) = self.stream.read()

    # Variable to control when the camera is stopped
        self.stopped = False

    def start(self):
    # Start the thread that reads frames from the video stream
        Thread(target=self.update,args=()).start()
        return self

    def update(self):
        # Keep looping indefinitely until the thread is stopped
        while True:
            # If the camera is stopped, stop the thread
            if self.stopped:
                # Close camera resources
                self.stream.release()
                return

            # Otherwise, grab the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
    # Return the most recent frame
        return self.frame

    def stop(self):
    # Indicate that the camera and thread should be stopped
        self.stopped = True

if __name__ == "__main__":
    
    #Code to allow driver to select bus route
    
    uploadtime= time.time()
    
    lcd.text("Select bus route ",1)
    lcd.text("route",2)
    
    while sw_state == 1:
        sw_state =  GPIO.input(sw)
        if I_State != GPIO.input(clk):

                if I_State != GPIO.input(dt):
                    counter += 1
                    if counter >= len(bus_routes): counter = 0
                    print("Count is =" , counter)
                    
                    lcd.text( ("Route =  " ),1)
                    lcd.text((bus_routes[counter] ),2)
                    
                    sw_state =  GPIO.input(sw)
                    time.sleep(0.3)
                elif I_State == GPIO.input(dt):
                    counter -= 1
                    if counter < 0: counter = len(bus_routes) -1
                    print ("Route is =", bus_routes[counter])

                    lcd.text( ("Route =  " ),1)
                    lcd.text((bus_routes[counter] ),2)
                    sw_state =  GPIO.input(sw)
                    time.sleep(0.3)
        I_state = GPIO.input(clk)
        


    print("Selected route: ", counter)
    lcd.text("Selected route: ",1)
    bus_route = bus_routes[counter]
    lcd.text( bus_route ,2)
    
    # save current bus route for next usage
    with open('previous_route.pickle','wb') as f:
        pickle.dump(counter,f)
        
    
    time.sleep(2)
    
    #initializing shared memory spaces between parent and child processes
    longitude = mp.Value('d',1 )
    latitude = mp.Value('d',1)
    speed = mp.Value('d',1)
    gps_process.start()
    distance = None
    #define parser inputs
    parser = argparse.ArgumentParser()
    parser.add_argument('--modeldir', help='Folder the .tflite file is located in',
                        default='TFLite_model_bbd')
    parser.add_argument('--graph', help='Name of the .tflite file, if different than detect.tflite',
                        default='detect.tflite')
    parser.add_argument('--labels', help='Name of the labelmap file, if different than labelmap.txt',
                        default='labelmap.txt')
    parser.add_argument('--threshold', help='Minimum confidence threshold for displaying detected objects',
                        default=0.5)
    parser.add_argument('--resolution', help='Desired webcam resolution in WxH. If the webcam does not support the resolution entered, errors may occur.',
                        default='1280x720')

    parser.add_argument('--output_path', help="Where to save processed imges from pi.",
                        default='saved_images')

    args = parser.parse_args()

    MODEL_NAME = args.modeldir
    GRAPH_NAME = args.graph
    LABELMAP_NAME = args.labels

    #extracting minimum confidence threshold 
    min_conf_threshold = float(args.threshold)

    #extracting resolution from parser
    resW, resH = args.resolution.split('x')
    imW, imH = int(resW), int(resH)


    # Import TensorFlow libraries
    pkg = importlib.util.find_spec('tensorflow')
    if pkg is None:
        from tflite_runtime.interpreter import Interpreter
        

    # Get path to current working directory
    CWD_PATH = os.getcwd()

    # Path to .tflite file, which contains the model that is used for object detection
    PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,GRAPH_NAME) #CWD/TFLite_model_bbd/detect.tflite

    # Path to label map file
    PATH_TO_LABELS = os.path.join(CWD_PATH,MODEL_NAME,LABELMAP_NAME) #CWD/TFLite_model_bbd/labelmap.txt

    # Load the label map
    with open(PATH_TO_LABELS, 'r') as f:
        labels = [line.strip() for line in f.readlines()]
    print(labels)

    # https://www.tensorflow.org/lite/models/object_detection/overview


    interpreter = Interpreter(model_path=PATH_TO_CKPT)

    interpreter.allocate_tensors()

    # Get model details
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    
    height = input_details[0]['shape'][1]   #height required for model
    width = input_details[0]['shape'][2]    #width required for model
    print("Height = ",height,"  Width = ",width)

    floating_model = (input_details[0]['dtype'] == np.float32)

    input_mean = 127.5
    input_std = 127.5

    #make output directory
    #os.makedirs(args.output_path, exist_ok=True)

    f = [] #to calculate average fps later on

    try:
        print("Progam started ")

    
        #timestamp an output directory for each capture
        outdir = pathlib.Path(args.output_path) / time.strftime('%Y-%m-%d_%H-%M-%S-%Z')
        outdir.mkdir(parents=True)
        
        time.sleep(.1)

        #Initialize frame rate calculation
        frame_rate_calc = 1
        freq = cv2.getTickFrequency()

        # Initialize video stream
        videostream = VideoStream(resolution=(imW,imH),framerate=30).start()
        time.sleep(1)

        #for frame1 in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):
        while True:

            # Start timer (for calculating frame rate)
            t1 = cv2.getTickCount()

            # Grab frame from video stream
            frame1 = videostream.read()

            # Acquire frame and resize to expected shape [1xHxWx3]
            frame = frame1.copy()
            #model takes RGB not BGR, hence conversion is required
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            #resizing to model's height and width
            frame_resized = cv2.resize(frame_rgb, (width, height))
            input_data = np.expand_dims(frame_resized, axis=0)

            # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
            if floating_model:
                input_data = (np.float32(input_data) - input_mean) / input_std

            # Perform the actual detection by running the model with the image as input
            interpreter.set_tensor(input_details[0]['index'],input_data)
            interpreter.invoke()

            # Retrieve detection results
            boxes = interpreter.get_tensor(output_details[0]['index'])[0] # Bounding box coordinates of detected objects
            classes = interpreter.get_tensor(output_details[1]['index'])[0] # Class index of detected objects
            scores = interpreter.get_tensor(output_details[2]['index'])[0] # Confidence of detected objects
            #num = interpreter.get_tensor(output_details[3]['index'])[0]  # Total number of detected objects (inaccurate and not needed)
            
            frame = cv2.line(frame, (pix_width_lane1,0), (pix_width_lane1,720), (123,123,123), 1)
            frame = cv2.line(frame, (pix_width_lane2,0), (pix_width_lane2,720), (123,123,123), 1) 
            # Loop over all detections and draw detection box if confidence is above minimum threshold
            
            object_in_roi = False
            for i in range(len(scores)):
                if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0)):

                    # Get bounding box coordinates and draw box
                    # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
                    ymin = int(max(1,(boxes[i][0] * imH)))
                    xmin = int(max(1,(boxes[i][1] * imW)))
                    ymax = int(min(imH,(boxes[i][2] * imH)))
                    xmax = int(min(imW,(boxes[i][3] * imW)))
                    
                    cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)
                    
                    # Draw label
                    object_name = labels[int(classes[i])] # Look up object name from "labels" array using class index
                    label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
                    labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
                    label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                    cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
                    cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text
                   
                    # red dot in the centre of the object. 
                    cx = int((xmin+xmax)/2) #because pixels are int values
                    cy = int((ymin+ymax)/2) 
                    cv2.circle(frame, (cx ,cy ), radius=5, color=(0, 0, 255), thickness=-1) #draw dot
                    
                    #print("Frame size width= ",frame.shape[1])
                    #print("Frame size height= ",frame.shape[0])
                    
                    #check if red dot is in region of interest
                    if cx > pix_width_lane1 and cx < pix_width_lane2:
                        
                        object_in_roi = True

                        if object_name == "car":
                            pix_width =  (xmax - xmin)
                            distance = k_pixel * car_width / pix_width
                            print("Car detected with a distance (cm) of:", distance)
                            
                        elif object_name == "person":
                            pix_width =  (xmax - xmin)
                            distance = k_pixel * person_width / pix_width
                            print(object_name,"detected with a distance (cm) of:", distance)
                            
                        elif object_name == "bus":
                            pix_width =  (xmax - xmin)
                            distance = k_pixel *bus_width / pix_width
                            print(object_name," detected with a distance (cm) of:", distance)
            
                        elif object_name == "truck":
                            pix_width =  (xmax - xmin)
                            distance = k_pixel * truck_width / pix_width
                            print(object_name," detected with a distance (cm) of:", distance)
                            
                        elif object_name == "rider":
                            pix_width =  (xmax - xmin)
                            distance = k_pixel * rider_width / pix_width
                            print(object_name," detected with a distance (cm) of:", distance)
        
                        elif object_name == "motor":
                            pix_width =  (xmax - xmin)
                            distance = k_pixel * motor_width / pix_width
                            print(object_name," detected with a distance (cm) of:", distance)
                        
                    
            if object_in_roi == False: distance = None
            

            if speed.value == None: bus_speed = 0.0
            else: bus_speed =float(speed.value)
            
            # Draw framerate in corner of frame
            cv2.putText(frame,'FPS: {0:.2f}'.format(frame_rate_calc),(30,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)
            cv2.putText(frame,('Speed: '+str(bus_speed)),(300,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)
            if distance != None:
                cv2.putText(frame,('Distance: {0:.2f}'.format(distance) ),(600,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)
            else:
                cv2.putText(frame,'Distance = None ' ,(600,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)
          


            # All the results have been drawn on the frame, so it's time to display it.
            
            
            # Calculate framerate
            t2 = cv2.getTickCount()
            time1 = (t2-t1)/freq
            frame_rate_calc= 1/time1
            f.append(frame_rate_calc) #sorting fps to calculate average later on

            cv2.imshow('ssd', frame) #show feed

            #path = '/home/pi/tflite1/webcam/' + str(datetime.datetime.now()) + ".jpg"
            
            #dangerous_path = str(outdir) + "/Dangerous/" +  + str(datetime.datetime.now()) + ".jpg"

           # slow_path = str(outdir) + "/Slow/" +  + str(datetime.datetime.now()) + ".jpg"

            #path = str(outdir) + '/'  + str(datetime.datetime.now()) + ".jpg"
            #status = cv2.imwrite(path, frame)
            
            min_stop_distance = stop_distance(bus_speed) 
            max_stop_distance = min_stop_distance * 2
            #Slow, fast and normal decision 
            
            if bus_speed >= 75:
                dangerous_state = True
                normal_state = True
                slow_state = False
                #saving pictures in dangerous folder
                #status = cv2.imwrite(dangerous_path, frame)
           
           # not considering too low speed
            elif (bus_speed <= 15):
                normal_state = True
                dangerous_state =False
                slow_state=False
            
            #if any vehicle is detected, not sure if driving purposely slow   
            elif (distance ==  None):
                if (bus_speed < 55 and bus_speed > 15 and normal_state == True):
                    slow_state = False
                    normal_state = False
                    dangerous_state = False
                    not_normal_timing = time.time()
                    #start saving picture
              #      status = cv2.imwrite(slow_path,frame)

                #start saving pictures
               # status = cv2.imwrite(slow_path,frame)
            else:
                
                if (distance < min_stop_distance):
                    slow_state = False
                    normal_state = True
                    dangerous_state = True
                
                #detects vehicle too far but unsure if purposely driving slow
                if (distance > max_stop_distance and normal_state== True): 
                     slow_state = False
                     normal_state = False
                     dangerous_state = False


                if  (distance < (1.5*min_stop_distance) and normal_state == False): #in case of overtaking 
                    slow_state = True
                    normal_state = False
                    dangerous_state = False

                #if vehile is between 1.5*min_stop_distance and max_stop_distance, tha program assume the preceeding vehicle is iniitally far before slows down
                
                elif (distance > min_stop_distance and distance < max_stop_distance ): # in case algorithm fails to detect 
                    slow_driving = False
                    normal_state = True
                    dangerous_state = False
                
   
            
            path = str(outdir) + '/'  + str(datetime.now()) + ".jpg"
            status = cv2.imwrite(path, frame)       
            
            
            if slow_state == True and  time.time() - uploadtime > 2:
                print("slow driving detected")
                storage.child(bus_ID).child("Slow_Driving").child(str(date.today())).child(str(datetime.now().time())).put(path)
                uploadtime = time.time()
            elif dangerous_state == True and  time.time()- uploadtime  > 2:
                print("dangerous driving detected")
                storage.child(bus_ID).child("Dangerous_Driving").child(str(date.today())).child(str(datetime.now().time())).put(path)
                uploadtime = time.time()
                
            elif normal_state == False and time.time()- uploadtime  > 2:
                print("Slow driving detected")
                storage.child(bus_ID).child("Slow_Driving").child(str(date.today())).child(str(datetime.now().time())).put(path)   
                uploadtime = time.time()

            # Press 'q' to quit
            if cv2.waitKey(1) == ord('q'):
                print(f"Saved images to: {outdir}")
                #GPIO.output(4, False)
                #led_on = False
                # Clean up
                cv2.destroyAllWindows()
                videostream.stop()
                #print(str(sum(f)/len(f)))
                lcd.clear()
                GPIO.cleanup()
                gps_process.terminate()
                gps_data = {"lat": 0.0, "lon": 0.0}
                db_gps.child("GPS").child(bus_route).update(gps_data)
                break
                    
    finally:
        lcd.clear()
        GPIO.cleanup()
            
        gps_process.terminate()
        gps_data = {"lat": 0.0, "lon": 0.0}
        db_gps.child("GPS").child(bus_route).update(gps_data) 

        # Clean up
        cv2.destroyAllWindows()
        videostream.stop()
        print("Avergae FPS =  ",str(sum(f)/len(f)))
        print("Program ended")
