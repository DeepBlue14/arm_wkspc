#! /usr/bin/env python

#####
##
##
##
#####

from enum import IntEnum

class Button(IntEnum):
    LEFT   = 1
    MIDDLE = 2
    RIGHT  = 3

class Direction(IntEnum):
    NORTH = 1
    SOUTH = 2
    EAST  = 3
    WEST  = 4

class Cursor():
    def __init__(self):
        self.historyX = []
        self.historyY = []
        self.historyVelX = []
        self.historyVelY = []
        self.prevX = 0
        self.prevY = 0
        self.currX = 0
        self.currY = 0
        self.futrX = 0
        self.futrY = 0
        self.prevVelX = 0
        self.prevVelY = 0
    
    
    def updateHistory(self):
        if len(self.historyX) > 1000:
            del self.historyX[0]
            del self.historyY[0]
            del self.historyVelX[0]
            del self.historyVelY[0]
        
        self.prevVelX = abs(self.currX - self.prevX)
        self.prevVelY = abs(self.currY - self.prevY)
        
        self.prevX = self.currX
        self.prevY = self.currY
        self.historyX.append(self.prevX)
        self.historyY.append(self.prevY)
        self.historyVelX.append(self.prevVelX)
        self.historyVelY.append(self.prevVelY)



