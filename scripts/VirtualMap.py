#!/usr/bin/env python

import json
import math
import copy

class VirtualMap:
    def __init__(self):
        # Load node graph
        f = open('nodeGraph.json')
        self.nodeGraph = json.load(f)
        
        # Load node locations
        f = open('nodeLocations.json')
        self.nodeLocations = json.load(f)
        
        # Load node names
        f = open('nodeNames.json')
        self.nodeNames = json.load(f)
        
        # Set starting position
        self.initialPosition = copy.deepcopy(self.nodeLocations[self.nodeNames[0]])
        self.position = self.initialPosition
        
        # Possible directions for the robot to face assuming 90 degree turns
        # (to keep it simple for now)
        self.directionOptions = [[0,-1],[-1,0],[0,1],[1,0]]
        
        # Index of the initial direction that the robot is facing
        self.initialDirectionIndex = 0
        self.directionIndex = self.initialDirectionIndex

        # Double check that the robot is on the path
        onPath = self.checkOnPath()
        if (onPath == False):
            print("Something went wrong with setting up the robot")
        
    # Return the direction that the robot is facing
    def getDirection(self):
        return self.directionOptions[self.directionIndex]
        
    
    # direction can either be a positive number or a negative number. The
    # magnitude of that number does not matter. Direction = 0 results in no 
    # change to the direction
    # Positive number indicatates a turn to the right, negative number is
    # a turn to the left
    def turn(self, direction):
        # Bullet proofing to make sure that the direction that the robot is
        # to turn makes sense
        if direction > 0:
            direction = 1
        elif direction < 0:
            direction = -1
        else: 
            direction = 0
            
        self.directionIndex += direction
        
        # Ensure that the circular buffer is maintained
        if self.directionIndex >= len(self.directionOptions):
            self.directionIndex = 0
        elif self.directionIndex < 0:
            self.directionIndex = len(self.directionOptions) - 1

    # Move the robot once in the direction being faced        
    def move(self):
        for i in range(len(self.position)):
            self.position[i] += self.getDirection()[i]
        
        
    # A way for the robot to look for the path by moving around in circles 
    # until it finds a spot that is on the path
    def getToPath(self):
        self.position = self.initialPosition
        self.directionIndex = self.initialDirectionIndex
        # movementsToTake = 1
        # movementsTaken = 0
        # numTurnsTaken = 0
        
        # while (not self.checkOnPath()):
        #     if movementsTaken <= movementsToTake:
        #         self.move()
        #         movementsTaken += 1
        #     else:
        #         self.turn(1)
        #         self.move()
        #         movementsTaken = 1
        #         movementsToTake += 1
        #         numTurnsTaken += 1
        #         if numTurnsTaken >= 2:
        #             numTurnsTaken = 0
        #             movementsToTake += 1
        # print(self.position)
        
        
    def getPostPoint(self):
        # Check if any of the locations have the same location as the robot
        for location in self.nodeLocations:
            if self.nodeLocations[location] == self.position:
                return location
                    
        # If no location code was found
        return -1
        
        
    # Check if the robot is on the path (is it between two connected lcations
    # on the map?)
    def checkOnPath(self):
        pathGood = False
        
        # Deal with the special case where the robot is at a post point
        # If you are then you are on the path
        if (self.getPostPoint() != -1):
            pathGood = True
        
        # Check if we are between two connected points
        else:
            for postA in self.nodeNames:
                a = self.nodeLocations[postA]
                for item in self.nodeGraph[postA]:
                    postB = item[0]
                    b = self.nodeLocations[postB]
                    if self.isBetween(a, b):
                        pathGood = True
        
        if pathGood == True:
            self.lastGoodPosition = self.position
            self.lastGoodDirectionIndex = self.directionIndex
        
        # If we are not between two connected points, and we are not at a post
        # point, we are off the path
        return pathGood
    
    # Check if th erobot is between a and b
    def isBetween(self, a,b):
        c = self.position
        return self.dist(a,c) + self.dist(b,c) == self.dist(a,b)
    
    def dist(self, a,b):
        ax = a[0]
        bx = b[0]
        ay = a[1]
        by = b[1]
        
        A = bx - ax
        B = by - ay
        
        C2 = (A * A) + (B * B)
        C = math.sqrt(C2)
        return C
        
    
# Test routine
def test():
    myMap = VirtualMap()
    
    print("Check that we are on the path (Expect True): " + str(myMap.checkOnPath()))
    
    myMap.turn(1)
    myMap.move()
    
    print("Check that we are on the path (Expect False): " + str(myMap.checkOnPath()))
    print("Check that we are on the path (Expect -1): " + str(myMap.getPostPoint()))
    
    myMap.getToPath()
    print(myMap.getDirection())
    myMap.move()
    print(str(myMap.position))
    
    


if __name__ == '__main__':
    test()