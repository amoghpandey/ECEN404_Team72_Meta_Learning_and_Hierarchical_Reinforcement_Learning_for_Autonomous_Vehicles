import csv
import cv2
import math
import os
import shlex
import subprocess
import tempfile
import pidcontroller
import mpu
import numpy as np
import time
import pandas as pd

from subprocess import PIPE, Popen
from threading  import Thread
import sys
import numpy as np
import re
from queue import Queue, Empty, LifoQueue

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.Piloting import moveBy, PCMD
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, AttitudeChanged, PositionChanged, AltitudeChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.enums.camera import streaming_mode

#with help from Vishnu and Gargi
# NOTE: Line numbers of this example are referenced in the user guide.
# Don't forget to update the user guide after every modification of this example.

class StreamingExample:

    def __init__(self):
        # Create the olympe.Drone object from its IP address
        self.drone = olympe.Drone(
            "10.202.0.1",
            loglevel=3,
        ) 
        
        
        self.q1 = LifoQueue()
        self.agent_pos = [0,0]
        # Run the command
        ON_POSIX = 'posix' in sys.builtin_module_names
        command1 = "parrot-gz topic -e /gazebo/default/pose/info | grep  -A 5 'name: \"anafi4k1\"'"
        self.p = Popen(command1, stdout=PIPE, bufsize=1, close_fds=ON_POSIX, shell=True)
        # Create a thread which dies with main program
        self.t1 = Thread(target = self.process_output1, args=(self.p.stdout, self.q1))
        self.t1.daemon = True
        self.t1.start()

    def start(self):
        # Connect the the drone
        self.drone.connection()
       

    def stop(self):
        # Properly stop the video stream and disconnect
        
        self.drone.disconnection()
        

   
    
    def set_control_magnitude(self, yaw_correction, pitch_correction, roll_correction, throttle_correction):
        yaw_mag = round(yaw_correction)
        pitch_mag = round(pitch_correction)
        roll_mag = round(roll_correction)
        throttle_mag = round(throttle_correction)

        if yaw_mag > 100:
            yaw_mag = 100
        elif yaw_mag < -100:
            yaw_mag = -100
        
        if pitch_mag > 100:
            pitch_mag = 100
        elif pitch_mag < -100:
            pitch_mag = -100

        if roll_mag > 100:
            roll_mag = 100
        elif roll_mag < -100:
            roll_mag = -100

        if throttle_mag > 100:
            throttle_mag = 100
        elif throttle_mag < -100:
            throttle_mag = -100
        
        #self.drone.start_piloting()
        self.drone(PCMD(1, roll_mag, pitch_mag, yaw_mag, throttle_mag, timestampAndSeqNum=0, _timeout=10))
        #self.drone.stop_piloting()
    # Process the output from the file
    def process_output1(self,out, queue):
        for line1 in iter(out.readline, b''):
            line1 = str(line1)
            if "x" in line1:
                 number = re.findall(r"[-+]?\d*\.\d+|\d+", line1)[0]
                 self.agent_pos[0] = float(number)
            if "y" in line1:
                 number = re.findall(r"[-+]?\d*\.\d+|\d+", line1)[0]
                 self.agent_pos[1] = float(number)
            '''if "z" in line:
                 number = re.findall(r"[-+]?\d*\.\d+|\d+", line)[0]
                 self.agent_pos[2] = float(number)'''
            queue.put(line1)
        out.close()
    def pos_feedback(self):
        for i in range(1):
            try:
                line1 = self.q1.get()
            except Empty:
                # Clear out the queue
                self.q1.queue.clear()
                

    def positive_control(self,x,y):
        print('-----------------------------------hi_4------------------------------------------')
        pid_pitch = pidcontroller.PID(50, 0.0, 0) #2.8, 1.8 level 2
        pid_roll = pidcontroller.PID(50, 0.0, 0)  #5.0, 2.0 level 2
        


        #target_yaw = 0  # 0 is north 
        target_x = x
        target_y = y
        

        
        start_time_pure= time.gmtime()
        start_time = start_time_pure.tm_sec
        '''heading = ["X-coord", "Y-coord", "Z-coord", "x", "y", "z", "count", "position"]
        with    open('XYZ_data.csv', 'w', newline='') as csvFile:
                writer = csv.writer(csvFile)
                writer.writerow(heading)
                csvFile.close()

        heading = ["pitch", "roll", "yaw", "throttle"]
        with    open('Control_data.csv', 'w', newline='') as csvFile:
                writer = csv.writer(csvFile)
                writer.writerow(heading)
                csvFile.close()

        heading = ["pitch", "roll", "yaw", "throttle"]
        with    open('Control_raw_data.csv', 'w', newline='') as csvFile:
                writer = csv.writer(csvFile)
                writer.writerow(heading)
                csvFile.close()

        heading = ["error_x_forward", "error_y_sideward", "error_z_vertical", "error_yaw"]
        with    open('Error_data.csv', 'w', newline='') as csvFile:
                writer = csv.writer(csvFile)
                writer.writerow(heading)
                csvFile.close()
        '''

        while (True):
      
            #print('--------time----------',time.time(),'------------time--------------')
            self.pos_feedback()
  
            current_x = self.agent_pos[0]
            current_y = self.agent_pos[1]
            print('================================',current_x,'=======================================')
            print('================================',current_y,'=======================================')
            #current_altitude = -Y + bottom_to_checker_origin - bottom_to_drone_camera

            error_x = target_x - current_x
            error_y = target_y - current_y
            print('================================',error_x,'=======================================')
            print('================================',error_y,'=======================================')
            pitch_correction = pid_pitch.Update(error_x)
            roll_correction = -pid_roll.Update(error_y)
            print('================================',pitch_correction,'=======================================')
            print('================================',roll_correction,'=======================================')

            if abs(error_x) < 0.05 and abs(error_y) < 0.05:                  
                #self.drone(Landing()) 
                #self.drone.disconnection()
                self.drone(PCMD(1, 0, 0, 0, 0, timestampAndSeqNum=0, _timeout=10))
                break
            
            else:    
                self.set_control_magnitude(0, int(round(pitch_correction)), int(round(roll_correction)), 0)
                time.sleep(1)

                print('Current x: %f' % current_x)
                print('Current y: %f' % current_y)
                print('Error_x %f' % error_x)
                print('Error_y %f' % error_y)
                print('Setting the pitch command to %f' % pitch_correction)
                print('Setting the roll command to %f' % roll_correction)
                




            



    def fly(self):
        # Takeoff, fly, land, ...
        print("Takeoff if necessary...")
        self.drone(
            FlyingStateChanged(state="hovering", _policy="check")
            | FlyingStateChanged(state="flying", _policy="check")
            | (
                GPSFixStateChanged(fixed=1, _timeout=10, _policy="check_wait")
                >> (
                    TakeOff(_no_expect=True)
                    & FlyingStateChanged(
                        state="hovering", _timeout=10, _policy="check_wait")
                )
            )
        ).wait()
                 
        df = pd.read_csv('route.csv') 
        #self.drone(moveBy(4, 2, 0, 0)>> FlyingStateChanged(state="hovering", _timeout=5)).wait()

        while (True):
           
            for i in range(len(df)):
                print('-----------------------------------',(df.X[i]/4)-10.125,(df.Y[i]/4)-10.125,'------------------------------------------')
                self.positive_control((df.X[i]/4)-10.125,(df.Y[i]/4)-10.125)
      
                self.drone(PCMD(1, 0, 0, 0, 0, timestampAndSeqNum=0, _timeout=10))
            #self.positive_control(-7.2,-7.2)
            #self.drone(PCMD(1, 0, 0, 0, 0, timestampAndSeqNum=0, _timeout=10))
            
            self.drone(Landing()) 
            self.drone.disconnection()


    

if __name__ == "__main__":
    streaming_example = StreamingExample()
    
    streaming_example.start()
    
    streaming_example.fly()
