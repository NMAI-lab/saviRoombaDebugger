#!/usr/bin/env python

# Demonstration program of the savi_ros_bdi system

# Created on Wed Feb 19 16:17:21 2020
# @author: Patrick Gavigan

class VirtualBot:
    def __init__(self):
        self.line = "c"
        self.position = (1,1)
        self.qr = "post1"
        self.direction = (1,1)
        self.battery = 95
        self.docked = False
        
    def act(self, actionParameter):
        print("Update internal state based on the action here")
        print("Action parameter was: " + actionParameter)
        
        # If the bot is doing drive(left) or drive(right), adjust the line accordingly
        
        # If the bot does turn(left) or turn(right), adjust the direction accordingly
        
        # If the bot does drive(forward) or drive(stop), adjust the position 
        # occordingly and updat the QR codes
        
        # If the action is dock or undock, figure out what to do about it
        
        
        # Perhaps later on (not initially) add some randomness to the line 
        # sensor so that sometimes the bot is slightly off the line
        
        # Have the battery reduce in charge?
        
 
    # To do: Figure out where to call this from
    # Charge and discharge the battery
    def updateBattery(self):
        if self.docked and (self.battery < 100):
            self.battery = self.battery + 1
        elif (not self.docked) and (self.battery > 0):
            self.battery = self.battery - 1
       
            
        
    # Sense the line state data
    def perceiveLine(self):
        return self.line
    
    # Sense the qr state data
    def perceiveQR(self):
        return self.qr

    # Sense the battery state data
    def perceiveBattery(self):
        # Have the battery reduce charge?
        return self.battery