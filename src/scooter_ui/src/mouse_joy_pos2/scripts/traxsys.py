#! /usr/bin/env python

#####
##
##
##
#####

import rospy
from geometry_msgs.msg import Twist

import pygame

from Mouse import Button, Cursor
from GameGui import GameGui
from Callibration import Callibration


class Traxsys():
    def __init__(self):
        self._myCursor = Cursor()
        self._myGui = GameGui()
        self._isRunning = 1
        
        self._sensitivity = 1.0
        self._isAutEvent = -1        # Was the 
        self._useMouseLock = True    # Lock mouse in GUI window
        self._useUnilateralMove = False
        
        self._pub = rospy.Publisher('chatter', Twist, queue_size=1)
        rospy.init_node('talker', anonymous=True)
        
        
    def run(self):
        while self._isRunning:
            event = pygame.event.poll()
            key = pygame.key.get_pressed()
            
            self._isReallyLeft = False
            self._isReallyRight = False
            self._isReallyUp = False
            self._isReallyDown = False
            
            if key[pygame.K_ESCAPE]:
                print "Bye!"
                exit()
            elif key[pygame.K_SPACE] and self._useMouseLock:     # Unbind mouse from UI window
                self._useMouseLock = False
            elif key[pygame.K_SPACE] and not self._useMouseLock: # Rebind mouse to UI window
                self._useMouseLock = True
            
            if event.type == pygame.QUIT:
                self._isRunning = 0
            elif event.type == pygame.MOUSEMOTION:
                self._myCursor.updateHistory()
                self._myCursor.currX, self._myCursor.currY = event.pos
                
                self._myCursor.futrX = self._myCursor.currX
                self._myCursor.futrY = self._myCursor.currY
                
                if self._myCursor.currX < self._myGui.windowOffset:
                    self._myCursor.futrX = self._myGui.windowOffset
                    if self._useMouseLock:
                        self._isReallyLeft = True
                    self._isAutEvent = 1
                elif self._myCursor.currX > self._myGui.windowX-self._myGui.windowOffset:
                    self._myCursor.futrX = self._myGui.windowX-self._myGui.windowOffset
                    if self._useMouseLock:
                        self._isReallyRight = True
                    self._isAutEvent = 1
                
                if self._myCursor.currY < self._myGui.windowOffset:
                    self._myCursor.futrY = self._myGui.windowOffset
                    if self._useMouseLock:
                        self._isReallyUp = True
                    self._isAutEvent = 1
                elif self._myCursor.currY > self._myGui.windowY-self._myGui.windowOffset:
                    self._myCursor.futrY = self._myGui.windowY-self._myGui.windowOffset
                    if self._useMouseLock:
                        self._isReallyDown = True
                    self._isAutEvent = 1
                
                
                if self._myCursor.currX > self._myCursor.prevX and self._isAutEvent != 0: # or self._myCursor.prevX <= self._myGui.windowOffset
                    print "=>"
                elif self._myCursor.currX < self._myCursor.prevX and self._isAutEvent != 0:
                    print "<="
                
                if self._myCursor.currY < self._myCursor.prevY and self._isAutEvent != 0:
                    print "UP"
                elif self._myCursor.currY > self._myCursor.prevY and self._isAutEvent != 0:
                    print "DOWN"
                
                self.msg = Twist()
                self.msg.linear.x = 0
                self.msg.linear.y = 0
                self._pub.publish(self.msg)
                
                if self._isReallyLeft or self._isReallyRight or self._isReallyUp or self._isReallyDown:
                    pygame.mouse.set_pos((self._myCursor.futrX, self._myCursor.futrY))
            
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == Button.LEFT:
                print "You pressed the left mouse button at (%d, %d)" % event.pos
                #print "Starting callibration..."
            elif event.type == pygame.MOUSEBUTTONUP and event.button == Button.LEFT:
                print "You released the left mouse button at (%d, %d)" % event.pos
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == Button.MIDDLE:
                print "You pressed the middle mouse button at (%d, %d)" % event.pos
            elif event.type == pygame.MOUSEBUTTONUP and event.button == Button.MIDDLE:
                print "You released the middle mouse button at (%d, %d)" % event.pos
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == Button.RIGHT:
                print "You pressed the right mouse button at (%d, %d)" % event.pos
            elif event.type == pygame.MOUSEBUTTONUP and event.button == Button.RIGHT:
                print "You released the right mouse button at (%d, %d)" % event.pos
            
            if self._isAutEvent > -1:
                self._isAutEvent -= 1
            
            self._myGui.update(self._myCursor.currX, self._myCursor.currY)




##### main() #####

if __name__ == "__main__":
    myDevice = Traxsys()
    myDevice.run()
