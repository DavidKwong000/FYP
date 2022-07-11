import os
import cv2
import base64
import numpy as np
from time import time
import time as stime
from flask import Flask
from datetime import datetime

import PCF8591 as ADC
import RPi.GPIO as GPIO

from flask_socketio import SocketIO
from threading import Lock

from time import sleep

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)
thread = None
thread_lock = Lock()
msgid = 0
buzzer_fire = False
buzzer_dir = False

#defind each pin of the sensors
makerobo_TRIG = 11
makerobo_ECHO = 12
makerobo_TouchPin = 13
makerobo_DO = 29
makerobo_BuzzerPin = 15

makerobo_motorLR = 16
makerobo_motorLR2 = 18
makerobo_motorS = 22

makerobo_DO_L = 31

#fire detecter detect time
timer = 2

#defind the status of touch pin
touch = 0

temperature=0

#return state
result = {"danger_zone" : False, "Fire" : False, "Temperature" : 0.0, "Motor" : False, "Rain": False}

s_data = None

def makerobo_setupL():
	GPIO.setup(makerobo_DO_L, GPIO.IN)

def makerobo_setuptemperature():
    global p
    GPIO.setup(makerobo_motorLR,GPIO.OUT)
    GPIO.setup(makerobo_motorLR2,GPIO.OUT)
    GPIO.setup(makerobo_motorS,GPIO.OUT)
    GPIO.output(makerobo_motorLR,GPIO.LOW)
    GPIO.output(makerobo_motorLR2,GPIO.LOW)
    p=GPIO.PWM(makerobo_motorS,1000)


def makerobo_setuph():
	global makerobo_ds18b20

	for i in os.listdir('/sys/bus/w1/devices'):
		if i != 'w1_bus_master1':
			makerobo_ds18b20 = i

def makerobo_setupultrasonic():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.setup(makerobo_TRIG, GPIO.OUT)
    GPIO.setup(makerobo_ECHO, GPIO.IN)

def makerobo_setupBuzzer():
    GPIO.setup(makerobo_BuzzerPin, GPIO.OUT)
    GPIO.output(makerobo_BuzzerPin, GPIO.HIGH)

def makerobo_setupTouch():
    GPIO.setup(makerobo_TouchPin, GPIO.IN, pull_up_down = GPIO.PUD_UP)

def makerobo_setup():
	ADC.setup(0x48)
	GPIO.setup(makerobo_DO, GPIO.IN)
	GPIO.setwarnings(False)

def makerobo_isAdult(x):
    global touch
    if x != touch:
        if x == 1:
            return True
        if x == 0:
            return False
        touch = x

def makerobo_buzzer_on():
    GPIO.output(makerobo_BuzzerPin, GPIO.LOW)

def makerobo_buzzer_off():
    GPIO.output(makerobo_BuzzerPin, GPIO.HIGH)

def ur_disMeasure():
    GPIO.output(makerobo_TRIG, 0)
    stime.sleep(0.000002)

    GPIO.output(makerobo_TRIG, 1)
    stime.sleep(0.00001)
    GPIO.output(makerobo_TRIG, 0)

    while GPIO.input(makerobo_ECHO) == 0:
        us_a = 0
    us_time1 = stime.time()

    while GPIO.input(makerobo_ECHO) == 1:
        us_a = 1
    us_time2 = stime.time()
    us_during = us_time2 - us_time1

    return us_during * 340 / 2 * 100

def makerobo_Print(x):
    global msgid
    global time_start
    global buzzer_fire
    global buzzer_dir
    global init_time
    if x != 0:
        result["Fire"] = False
        buzzer_fire = False
        init_time = time()
        if buzzer_fire == True or buzzer_dir == True:
            makerobo_buzzer_on()
        else:
            makerobo_buzzer_off()

    if init_time + float(s_data['Fire_number']) <= time():
        result["Fire"] = True
        msgid += 1
        socketio.emit('message', {"id": msgid,"title": "Fire", "message": "On Fire!",
                    "messageTime": str(datetime.now())})
        buzzer_fire = True
        if buzzer_fire == True or buzzer_dir == True:
            makerobo_buzzer_on()
        else:
            makerobo_buzzer_off()

def makerobo_loop_Arthur():

    #fire detecter detect time
    while True:
        makerobo_tmp = GPIO.input(makerobo_DO)
        makerobo_Print(makerobo_tmp)
        break

def video_loop():

    cap = cv2.VideoCapture(0)

    width = 1280
    height = 960

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    area = width * height
    ret, frame = cap.read()
    avg = cv2.blur(frame, (4, 4))
    avg_float = np.float32(avg)
    init_time = time()
    data = {}

    while(True):
        ret, frame = cap.read()

        if ret == False:
            break

        blur = cv2.blur(frame, (4, 4))

        diff = cv2.absdiff(avg, blur)

        gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)

        ret, thresh = cv2.threshold(gray, 25, 255, cv2.THRESH_BINARY)

        kernel = np.ones((5, 5), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

        cnts, cntImg = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in cnts:
            if cv2.contourArea(c) < 2500:
                continue
            (x, y, w, h) = cv2.boundingRect(c)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            init_time = time()

        cv2.drawContours(frame, cnts, -1, (0, 255, 255), 2)
        result['warning'] = False
        if init_time + 10 <= time():
            #warning message
            print("warning")
            result['warning'] = True
        cv2.imwrite('frame.jpg', frame)
        with open('frame.jpg', "rb") as f:
            im_bytes = f.read()
        im_b64 = base64.b64encode(im_bytes).decode("utf8")
        result['image'] = im_b64
        break

    cap.release()
    cv2.destroyAllWindows()

def makerobo_loop_David():
    while True:
        global msgid
        global buzzer_fire
        global buzzer_dir
        us_dis = ur_disMeasure()
        print(us_dis)
        if us_dis <= 30:
            if makerobo_isAdult(GPIO.input(makerobo_TouchPin)) != True:
                print("on")
                result["danger_zone"] = True
                buzzer_dir = True
                msgid += 1
                socketio.emit('message', {"id": msgid,"title": "danger_zone", "message": "Your baby in the danger area!",
                                "messageTime": str(datetime.now())})
                if buzzer_fire == True or buzzer_dir == True:
                    makerobo_buzzer_on()
                else:
                    makerobo_buzzer_off()
                print(buzzer_fire)
                print(buzzer_dir)
                break
            else:
                print('t')
                buzzer_dir = False
                if buzzer_fire == True or buzzer_dir == True:
                    makerobo_buzzer_on()
                else:
                    makerobo_buzzer_off()

        else:
            result["danger_zone"] = False
            buzzer_dir = False
            print(buzzer_fire)
            print(buzzer_dir)
            if buzzer_fire == True or buzzer_dir == True:
                makerobo_buzzer_on()
            else:
                makerobo_buzzer_off()
            break

def makerobo_read():
	makerobo_location = '/sys/bus/w1/devices/' + makerobo_ds18b20 + '/w1_slave'
	makerobo_tfile = open(makerobo_location)
	makerobo_text = makerobo_tfile.read()
	makerobo_tfile.close()
	secondline = makerobo_text.split("\n")[1]
	temperaturedata = secondline.split(" ")[9]
	temperature = float(temperaturedata[2:])
	temperature = temperature / 1000
	return temperature

def makerobo_loop_Hayden():
    global msgid
    while True:
        if makerobo_read() != None:
            result["Temperature"] = makerobo_read()
            if makerobo_read() < int(s_data['Motor_number']):
                sleep(0)
                GPIO.output(makerobo_motorLR, GPIO.HIGH)
                GPIO.output(makerobo_motorLR2, GPIO.LOW)
                p.ChangeDutyCycle(0)
                result["Motor"] = False
                print ("Current temperature : %0.3f C" % makerobo_read())
                break
            else:
                GPIO.output(makerobo_motorLR, GPIO.HIGH)
                GPIO.output(makerobo_motorLR2, GPIO.LOW)
                p.start(100)
                result["Motor"] = True
                print("Too Hot")
                msgid += 1
                socketio.emit('message', {"id": msgid,"title": "Motor", "message": "Too Hot!",
                    "messageTime": str(datetime.now())})
                break



def makerobo_Print_water(x):
	global msgid
	detect_counter = 0
	water_time_counter = 0
	if x == 1:          # No water
		result["Rain"] = False
		print ('   *  Not detect water!   *')
	else:          # Have water
		result["Rain"] = True
		msgid += 1
		socketio.emit('message', {"id": msgid,"title": "Rain", "message": "Have Rain!",
                    "messageTime": str(datetime.now())})
		print ('   *   Detect water!!   *')
		water_time_counter += 1
	if water_time_counter == 10:
		detect_counter += 1
	if detect_counter == 2:
		print ('   *  Need to change it   *')
		detect_counter = 0
		water_time_counter = 0

def makerobo_loop_Leo_rain():
	makerobo_status = 1      # sensor state
	while True:
		print (ADC.read(0))
		result["Rain"] = False
		makerobo_tmp = GPIO.input(makerobo_DO_L)
		if makerobo_tmp != makerobo_status:
			makerobo_Print_water(makerobo_tmp)
			makerobo_status = makerobo_tmp		
		stime.sleep(0.2)                    # delate for 200ms
		break
		

def destroy():
    GPIO.output(makerobo_BuzzerPin, GPIO.HIGH)
    GPIO.cleanup()

def sensor_state():
    while True:

        socketio.sleep(1)
        video_loop()
        if s_data is not None:
            if s_data['Fire'] is True:
                if s_data['Fire_number'] is 0:
                    print("You must type the Fire number!")
                else:
                    makerobo_loop_Arthur()
            if s_data['Distance'] is True:
                makerobo_loop_David()
            if s_data['Motor'] is True:
                if s_data['Motor_number'] is 0:
                    print("You must type the Temperature number!")
                else:
                    makerobo_loop_Hayden()
            if s_data['Rain'] is True:
                makerobo_loop_Leo_rain()
        socketio.emit('update', result)

@socketio.on('connect')
def connect():
    print("connect...")
    global thread
    with thread_lock:
        if thread is None:
            thread = socketio.start_background_task(target = sensor_state)

@socketio.on('disconnect')
def disconnect():
    print("disconnect...")

@socketio.on('state')
def state(data):
    global s_data
    s_data = data
    print(data)

if __name__ == "__main__":
    makerobo_setupultrasonic()
    makerobo_setupBuzzer()
    makerobo_setupTouch()
    time_start = 0
    makerobo_setup()
    makerobo_setuptemperature()
    makerobo_setuph()
    makerobo_setupL()
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)