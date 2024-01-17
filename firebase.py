######firebase#############
import pyrebase 


# Enter your Firebase config and API
fb_gps_config = {
  "apiKey": "",                 
  "authDomain": "",             
  "databaseURL": "",
  "projectId": "",
  "storageBucket": "",
  "messagingSenderId": "",
  "appId": "",
  "measurementId": ""
}

fb_gps =  pyrebase.initialize_app(fb_gps_config)
db_gps = fb_gps.database()

#data = {"lat":0.0, "lon":0.0}
#db_gps.child("GPS").child("15 Rose-Hill").set(data) create database
#db_gps.child("GPS").child("15 Rose-Hill").update(data) update database
#db_gps.child("GPS").child("15 Rose-Hill").remove() del database
##db_gps.child("GPS").child("15 Rose-Hill").get()  read data

#######################gps################
import io
import pynmea2
import serial


ser = serial.Serial('/dev/serial0', 9600, timeout=1.0)
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
########################################################

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD) #always check bcm vs board
import time
import pickle

#configuring lcd display
from rpi_lcd import LCD
#from signal import signal, SIGTERM, SIGHUP,pause

lcd = LCD()
disp = ""

#### Rorary encoder
sw = 11
dt = 13
clk = 15

counter = 0
bus_routes = ("15 Flacq", "15 Rose-Hill", "16 Flacq", "16 Rose-Hill","194 Port-Louis", "194 Rose-Hill")

# To prevent driver from researching the whole thing
with open('previous_route.pickle','rb') as f:
    counter = pickle.load(f)
    

GPIO.setup(dt, GPIO.IN)
GPIO.setup(sw, GPIO.IN)
GPIO.setup(clk, GPIO.IN)

I_State = GPIO.input(clk)

sw_state = True #switch output is 0 when pressed 


if __name__ == '__main__':

    #sw_state = mp.Value('b',1 ) #here it's .Value

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
    
    with open('previous_route.pickle','wb') as f:
        pickle.dump(counter,f)
        
    
    time.sleep(2)
    lcd.clear()
    GPIO.cleanup()
    
    while 1:
        try:
            line = sio.readline()
            msg = pynmea2.parse(line)
            #print(repr(msg))
            if msg.sentence_type == 'GGA':
                
                
                print("Latitude= ",msg.latitude)
                print("Longitude= ",msg.longitude)
                gps_data = {"lat": msg.latitude, "lon":msg.longitude}
                db_gps.child("GPS").child(bus_route).update(gps_data)               

            elif msg.sentence_type == 'VTG':
                print("Speed over ground km/h= ",msg.spd_over_grnd_kmph) 
        except serial.SerialException as e:
            print('Device error: {}'.format(e))
            break
        except pynmea2.ParseError as e:
            print('Parse error: {}'.format(e))
            continue
            
 
