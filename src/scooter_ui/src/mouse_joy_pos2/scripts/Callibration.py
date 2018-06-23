#! /usr/bin/env python

#####
##
##
##
#####

import time
import sys, os
from PyQt4 import QtGui
from PyQt4 import QtCore
from threading import Lock
import rospy
from std_msgs.msg import String
#from vacuum_experiment_msgs.msg import Telemetry, PhoneReply

#from cfgloader import Settings
import pyqtgraph as pg
import numpy as np


class Callibration():
    def __init__(self):
        self._calLeft  = []
        self._calRight = []
        self._calUp    = []
        self._calDown  = []
    
    
    def cal_left_update(self, myTwist):
        self._calLeft.append(myTwist)
        
    
    def cal_right_update(self, myTwist):
        self._calRight.append(myTwist)
        
        
    def cal_up_update(self, myTwist):
        self._calUp.append(myTwist)
        
    
    def cal_down_update(self, myTwist):
        self._calDown.append(myTwist)
    
    
    """
    " Finds the median number of misfires for a given direction.
    "
    """
    def run_callibration(self, myList, myAns):
        meanLst = [sum(myList[x:x+100])/100 for x in range(0, len(myList), 100)]
        return np.median(np.asarray(meanLst))
    

