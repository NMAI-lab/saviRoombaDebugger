#!/usr/bin/env python


import rospy
import re
import random
from VirtualMap import VirtualMap

class VirtualBot:
    def __init__(self):
        self.lineValues = ["ll","l","c","r"]    # Treat this as a circular list
        self.lineIndex = self.lineValues.index("c")  # Index of the circular list
        #self.position = 1
        #self.postPointIndex = 0
        #self.postPoints = ["post1", "post2", "post3", "post4", "post5"]
        #self.movementCount = 0
        #self.directionOK = True # Update this later to be based on geometry
        self.battery = 95
        self.docked = False
        self.map = VirtualMap()
        
    def act(self, action):
        rospy.loginfo("Action parameter was: " + action)
        
        # Handle the docking station cases
        if action == "station(dock)":
            self.dock()
        elif action == "station(undock)":
            self.undock()
        
        # Extract the action parameter between the brackets
        parameter = re.search('\((.*)\)', action).group(1)
    
        # Deal with drive action
        if re.search("drive", action):
            self.drive(parameter)
    
        # Deal with turn action
        elif re.search("turn", action):
            self.turn(parameter)
    
        # Deal with invalid action
        else:
            rospy.loginfo("Invalid action ignored")
        
        # Update battery
        self.updateBattery()
        
        # Check for map error
        if (not self.map.checkOnPath()):
            rospy.loginfo("!!!! MAP ERROR, THE ROBOT LEFT THE MAP!!!!!")


    # Implementation of the drive action
    def drive(self, parameter):
        if self.docked == False:
            rospy.loginfo("I need to drive: " + str(parameter))
            if parameter == "forward":
                self.map.move()
                rospy.loginfo("Map location: " + str(self.map.position))
            else:
                self.updateLine(parameter)
        else: 
            rospy.loginfo("I can't drive, I'm docked!")
        
    # Implementation of the turn action
    # Similar to drive action but continues until the line sensor detects "c"
    def turn(self, parameter):
        if self.docked == False:
            rospy.loginfo("I need to turn: " + str(parameter))
            if parameter == 'left':
                self.map.turn(-1)
            else:
                self.map.turn(1)
            
            # If the bot does turn(left) or turn(right), adjust the direction accordingly
            self.lineIndex = self.lineValues.index("c")

        else:
            rospy.loginfo("I can't turn, I'm docked!")
        
    def dock(self):
        # Need to add a check that the position is at the docking station
        
        self.docked = True
        rospy.loginfo("Robot is docked")
        
    def undock(self):
        self.docked = False
        
        # Need to do an update of the position and the line here
        
        rospy.loginfo("Robot is undocked")
        
        
    # This is a hack! Need to update to reflect geometry of the map
    # Basically, this implements the drive(forward) action
    #def updatePosition(self):
    #    if self.perceiveLine() == "c":
    #        if self.movementCount > 4:
    #            self.movementCount = 0
    #            self.postPointIndex += 1
    #        else: 
    #            self.movementCount += 1
    #    else:
    #        self.lineIndex = self.lineValues.index("ll")                    

        
    # Update the line sensor data
    # Perhaps later on (not initially) add some randomness to the line 
    # sensor so that sometimes the bot is slightly off the line
    def updateLine(self, direction):
        if direction == "left":
            self.lineIndex += 1
        elif direction == "right":
            self.lineIndex -= 1
        elif direction == "sprial":
            self.lineIndex = random.randint(0,len(self.lineValues)-1)
            
        if self.lineIndex >= len(self.lineValues):
            self.lineIndex = 0
        elif self.lineIndex < 0:
            self.lineIndex = len(self.lineValues) - 1
 

    # Charge and discharge the battery
    def updateBattery(self):
        if self.docked and (self.battery < 100):
            self.battery = self.battery + 1
        elif (not self.docked) and (self.battery > 0):
            self.battery = self.battery - 1
       

    # Sense the line state data
    def perceiveLine(self):
        return self.lineValues[self.lineIndex]
    
    # Sense the qr state data
    def perceiveQR(self):
        return self.map.getPostPoint()
        #if self.movementCount == 0:
        #    return self.postPoints[self.postPointIndex]
        #else:
        #    return -1

    # Sense the battery state data
    def perceiveBattery(self):
        # Have the battery reduce charge?
        return self.battery
    
def test():
    bot = VirtualBot()
    print(bot.perceiveBattery())
    
    bot.act(str("drive(forward)"))
    
if __name__ == '__main__':
    test()