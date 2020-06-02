
from controller import Robot
from controller import Camera
from vehicle import Driver

import math
import socket
from _thread import *
import threading
import pickle

from controller import Supervisor
import math

class BotController():

    def __init__(self,driver):
        print("New Bot class")
        self.driver = driver
        
        self.TCP_IP = '127.0.0.1'
        self.TCP_PORT = 12345
        self.BUFFER_SIZE = 1024
        
        #self.print_lock = threading.Lock()
        
        # misc variables
        self.Speed = 0.0
        self.Angle = 0.0
        self.manualSteering = 0
        self.steeringAngle = 0.0
        
        # Sensore
        self.camera1 = None
        self.camera2 = None
        self.camera3 = None        
        
        self.TIME_STEP = None
        
        self.manual_steering = 0
        self.new_manual_steering = 0
        
        print("Start Bot controller")
        
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        while(1):
            try :
                self.s.bind((self.TCP_IP, self.TCP_PORT))
                break
            except:
                pass
               
        print("socket binded to port", self.TCP_PORT)
        self.start = 1
        # put the socket into listening mode
        self.s.listen(1)
        print("socket is listening")
        
        # establish connection with client
        self.c, self.addr = self.s.accept()
        print('Connected to :', self.addr[0], ':', self.addr[1])
        # data received from client
        while(1):
            data = self.c.recv(1)                           
            if str(data.decode('ascii')) == 'S':                
                print("Start new thread")   
                # Start a new thread and return its identifier                
                start_new_thread(self.threaded, ("Start",))
                break
                
    # Initalize the motors.
    def setup(self):
        self.TIME_STEP = int(self.driver.getBasicTimeStep())
        self.camera1 = self.driver.getCamera("camera1");
        self.camera2 = self.driver.getCamera("camera2");            
        self.camera1.enable(self.TIME_STEP);
        self.camera2.enable(self.TIME_STEP);            
        self.camera3 = self.driver.getCamera("camera3");
        self.camera3.enable(self.TIME_STEP);
        
        self.camera_width = self.camera1.getWidth()
        self.camera_height = self.camera1.getHeight()
        self.BUFFER_SIZE = self.camera_height*self.camera_width*4*2
        print('Buffer size:', self.camera_width, '*',self.camera_height,'*4*2')

    # More driving logic here
    
    # positive: turn right, negative: turn left
    def setSteeringAngle(self,wheel_angle) :
    
        # limit the difference with previous steering_angle
        if wheel_angle - self.steeringAngle > 0.1:
            wheel_angle = self.steeringAngle + 0.15;
        if wheel_angle - self.steeringAngle < -0.1:
            wheel_angle = self.steeringAngle - 0.15;
        self.steering_angle = wheel_angle;
        # limit range of the steering angle
        if wheel_angle > 0.5:
            wheel_angle = 0.5;
        elif wheel_angle < -0.5:
            wheel_angle = -0.5;
        self.driver.setSteeringAngle(wheel_angle);
    
    def changeManualSteerAngle(self,inc):
        #driver.setSteeringAngle(0.2)
        self.new_manual_steering = self.manualSteering + inc;
        if self.new_manual_steering <= 25.0 and self.new_manual_steering >= -25.0:
            self.manualSteering = self.new_manual_steering;
            self.driver.setSteeringAngle(self.manualSteering * 0.02);
    

    # thread function
    def threaded(self,s):
        print("Start threaded.")
        self.setup()
        while self.driver.step() != -1:
    
            # data received from client
            try:
                data = self.c.recv(1)
            except Exception as err:   
                break
                
            if not data:
                print('Bye:1')
    
                # lock released on exit
                #print_lock.release()
                break
    
            print('New command')
            cmd = str(data.decode('ascii'))
            if cmd == 'D':
                data = ''
                image1 = self.camera1.getImageArray();
                image2 = self.camera2.getImageArray();
                image3 = self.camera3.getImageArray();
                for x in range(0,self.camera_width):
                    for y in range(0,self.camera_height):
                        if x == 0 and y == 0:
                            minRGB1 = str(min(image1[0][0][0],image1[0][0][1],image1[0][0][2])).zfill(3)
                        else :
                            minRGB1= minRGB1+ ','+ str(min(image1[x][y][0],image1[x][y][1],image1[x][y][2])).zfill(3)  
                                 
                for x in range(0,self.camera_width):
                    for y in range(0,self.camera_height):
                        if x == 0 and y == 0:
                            minRGB2 = str(min(image2[0][0][0],image2[0][0][1],image2[0][0][2])).zfill(3)
                        else :
                            minRGB2= minRGB2 + ','+ str(min(image2[x][y][0],image2[x][y][1],image2[x][y][2])).zfill(3) 
                
                for x in range(0,self.camera_width):
                    for y in range(0,self.camera_height):
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
                    self.c.send(minRGB1.encode('ascii'))
                    self.c.send(minRGB2.encode('ascii'))
                    self.c.send(minRGB3.encode('ascii'))
                except:
                    print('Bye:2') 
                    break
            elif cmd == 'V':
                try:
                    speedInt = self.c.recv(4)
                    speedFlt = self.c.recv(3)                 
                    speedInt = int(speedInt.decode('ascii'))
                    speedFlt = int(speedFlt.decode('ascii'))
                    Speed = speedInt + speedFlt / 1000.0       
                    self.driver.setCruisingSpeed(Speed)
                    print("Speed: ",Speed )   
                                    
                except:
                    print('Bye:3') 
                    #break
            elif cmd == 'A':
                try:
                    angleInt = self.c.recv(4)
                    angleFlt = self.c.recv(3)
                    angleInt = int(angleInt.decode('ascii'))
                    angleFlt = int(angleFlt.decode('ascii'))    
                    Angle = angleInt + angleFlt / 1000.0     
                    print("Angle: ",Angle )
                    self.changeManualSteerAngle(Angle)                           
                except:
                    print('Bye:4') 
                    #break
            elif cmd == 'T': # Step
                try:
                    finished = 1
                    if(self.driver.step(self.timestep) != -1):                
                        self.c.send(finished.encode('ascii'))
                    else:
                        finished = 0
                        self.c.send(finished.encode('ascii'))    
                except:
                    print('Bye:5') 
                    break
            elif cmd == 'E': # Setup
                self.setup()
            elif cmd == 'R':
                self.driver.simulationReset()  
            
            else:
                print('Bye:6') 
                # lock released on exit
                #print_lock.release()
                break
        # connection closed   
        self.c.close()
        self.s.close()
        self.start = 1  
    
def Main():
    driver = Driver()   
    #supervisor = Supervisor()
    
    TIME_STEP = int(driver.getBasicTimeStep())
    
    start = 0
    # a forever loop until client wants to exit
    bot = BotController(driver)
    while driver.step() != -1:                      
        pass
        
    
    
# Enter here exit cleanup code.
if __name__ == '__main__':
    Main()

