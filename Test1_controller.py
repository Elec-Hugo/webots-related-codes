
"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Camera
from vehicle import Driver

import math
import socket
from _thread import *
import threading
import pickle

TCP_IP = '127.0.0.1'
TCP_PORT = 12345
BUFFER_SIZE = 1024
TIME_STEP=50
print_lock = threading.Lock()

# misc variables
Speed = 0.0
Angle = 0.0
manualSteering = 0
steeringAngle = 0.0

# positive: turn right, negative: turn left
def setSteeringAngle(driver,wheel_angle) :

    # limit the difference with previous steering_angle
    if wheel_angle - steeringAngle > 0.1:
        wheel_angle = steeringAngle + 0.1;
    if wheel_angle - steeringAngle < -0.1:
        wheel_angle = steeringAngle - 0.1;
    steering_angle = wheel_angle;
    # limit range of the steering angle
    if wheel_angle > 0.5:
        wheel_angle = 0.5;
    elif wheel_angle < -0.5:
        wheel_angle = -0.5;
    driver.setSteeringAngle(wheel_angle);

def changeManualSteerAngle(driver,inc):
    #driver.setSteeringAngle(0.2)
    new_manual_steering = manualSteering + inc;
    if new_manual_steering <= 25.0 and new_manual_steering >= -25.0:
        manual_steering = new_manual_steering;
        setSteeringAngle(driver,manual_steering * 0.02);

    #if manual_steering == 0:
        #print("going straight\n");
    #else:
        #if steeringAngle < 0 : leftORright = "left" 
        #else: leftORright = "right"
        #print("turning ", steeringAngle," rad ", leftORright);

# thread function
def threaded(driver,s,c,camera1,camera2,camera3):

    camera_width=camera1.getWidth()
    camera_height=camera1.getHeight()
    BUFFER_SIZE = camera_height*camera_width*4*2
    print('Buffer size:', camera_width, '*', camera_height,'*4*2')
    
    while True:

        # data received from client
        try:
            data = c.recv(1)
        except Exception as err:   
            break
            
        if not data:
            print('Bye')

            # lock released on exit
            #print_lock.release()
            break

        print('New command')
        cmd = str(data.decode('ascii'))
        if cmd == 'D':
            data = ''
            image1 = camera1.getImageArray();
            image2 = camera2.getImageArray();
            image3 = camera3.getImageArray();
            for x in range(0,camera_width):
                for y in range(0,camera_height):
                    if x == 0 and y == 0:
                        minRGB1 = str(min(image1[0][0][0],image1[0][0][1],image1[0][0][2])).zfill(3)
                    else :
                        minRGB1= minRGB1+ ','+ str(min(image1[x][y][0],image1[x][y][1],image1[x][y][2])).zfill(3)  
                             
            for x in range(0,camera_width):
                for y in range(0,camera_height):
                    if x == 0 and y == 0:
                        minRGB2 = str(min(image2[0][0][0],image2[0][0][1],image2[0][0][2])).zfill(3)
                    else :
                        minRGB2= minRGB2 + ','+ str(min(image2[x][y][0],image2[x][y][1],image2[x][y][2])).zfill(3) 
            
            for x in range(0,camera_width):
                for y in range(0,camera_height):
                    if x == 0 and y == 0:
                        minRGB3 = str(min(image3[0][0][0],image3[0][0][1],image3[0][0][2])).zfill(3)
                    else :
                        minRGB3= minRGB3 + ','+ str(min(image3[x][y][0],image3[x][y][1],image3[x][y][2])).zfill(3) 

            minRGB1 = minRGB1 + ','
            minRGB2 = minRGB2 + ','
            minRGB3 = minRGB3 + ','            
            print("Image Left was Sent")
            print("Image Right was Sent")
            # send back reversed string to client
            try:
                c.send(minRGB1.encode('ascii'))
                c.send(minRGB2.encode('ascii'))
                c.send(minRGB3.encode('ascii'))
            except Exeption as err:
                print('Bye') 
                break
        elif cmd == 'V':
            try:
                speedInt = c.recv(4)
                speedFlt = c.recv(3)
                speedInt = int(speedInt.decode('ascii'))
                speedFlt = int(speedFlt.decode('ascii'))
                Speed = speedInt + speedFlt / 1000.0       
                print("Speed: ",Speed )   
                driver.setCruisingSpeed(Speed)
            
            except Exeption as err:
                print('Bye') 
                break
        elif cmd == 'A':
            try:
                angleInt = c.recv(4)
                angleFlt = c.recv(3)
                angleInt = int(angleInt.decode('ascii'))
                angleFlt = int(angleFlt.decode('ascii'))    
                Angle = angleInt + angleFlt / 1000.0     
                print("Angle: ",Angle )
                changeManualSteerAngle(driver,Angle)                           
            except Exeption as err:
                print('Bye') 
                break
        else:
            print('Bye') 
            # lock released on exit
            #print_lock.release()
            break
    # connection closed   
    c.close()
    s.close()
    start = 1  

def Main():
   
    # create the car instance.
    driver = Driver()
    camera1 = driver.getCamera("camera1");
    camera2 = driver.getCamera("camera2");
    camera1.enable(TIME_STEP);
    camera2.enable(TIME_STEP);
    
    camera3 = driver.getCamera("camera3");
    camera3.enable(TIME_STEP);
    
    # You should insert a getDevice-like function in order to get the
    # instance of a device of the robot. Something like:
    #  motor = robot.getMotor('motorname')
    #  ds = robot.getDistanceSensor('dsname')
    #  ds.enable(timestep)

        
    
    # lock acquired by client
    #print_lock.acquire()
    
    start = 0
    # a forever loop until client wants to exit
    while driver.step() != -1:
              
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.bind((TCP_IP, TCP_PORT))
            print("socket binded to port", TCP_PORT)
            start = 1
            # put the socket into listening mode
            s.listen(1)
            print("socket is listening")
            
            # establish connection with client
            c, addr = s.accept()
            # data received from client
            data = c.recv(1)   
            print('Connected to :', addr[0], ':', addr[1])
            
            if str(data.decode('ascii')) == 'S':
                
                print("Start new thread")   
                # Start a new thread and return its identifier
                start_new_thread(threaded, (driver,s ,c,camera1,camera2,camera3,))
        except Exception as err:
            0

        pass
    s.close()
    
    
# Enter here exit cleanup code.
if __name__ == '__main__':
    Main()

