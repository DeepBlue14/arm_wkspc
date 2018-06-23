#!/usr/bin/python

#Synchronize a couple of image streams and use them for stereo calibration

import rospy
import cv2
import numpy as np
import sys

import message_filters
from sensor_msgs.msg import Image, CameraInfo

import pickle
import time

from cv_bridge import CvBridge, CvBridgeError

class StereoRegistrator():
    def __init__(self):
        self.imagePoints1 = []
        self.imagePoints2 = []
        #Parameters of the checkerboard, these are corners, not checkers spaces
        boardW = 9
        boardH = 7
        boardN = boardW * boardH
        self.boardSize = (boardW, boardH)
        boardPhysSize = 0.0395 #Physical distance between points, in meters (or whatever units you want the output in)

        self.objp = []
        for ii in range(boardN):
            self.objp.append((boardPhysSize * (ii/boardW), boardPhysSize * (ii%boardW), 0.0))
        self.objp = np.asarray(self.objp)
        self.objectPoints = []
        
        #Number of boards to use for calibration
        self.sampleCount = 200
        
        #CVBridge to use for image conversion
        self.imageBridge = CvBridge()
        
        #Windows for debugging
        cv2.namedWindow("Image 1", 1)
        cv2.namedWindow("Image 2", 1)
        
    def capturePoints(self, img1, img2):
        
        #Convert color ROS image to grayscale OpenCV images
        img1 = self.imageBridge.imgmsg_to_cv2(img1, "bgr8")
        img2 = self.imageBridge.imgmsg_to_cv2(img2, "bgr8")
        img1gray = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        img2gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

        #Find chessboards
        ret1, corners1 = cv2.findChessboardCorners(img1gray, self.boardSize, None, (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FILTER_QUADS))
        ret2, corners2 = cv2.findChessboardCorners(img2gray, self.boardSize, None, (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FILTER_QUADS))
        
        #Subcorner and draw the chessboard points, args 3 and 4 to cornerSubPix are 1/2 window size and dead band in the middle
        # of the window. The subpixel function uses gradients over that area to estimate sub-pixel corner locations  
        if ret1:
            cv2.cornerSubPix(img1gray, corners1, (5,5), (-1,-1), (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.01))
            cv2.drawChessboardCorners(img1gray, self.boardSize, corners1, ret1);
    
        if ret2:
            cv2.cornerSubPix(img2gray, corners2, (5,5), (-1,-1), (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.01))
            cv2.drawChessboardCorners(img2gray, self.boardSize, corners2, ret2);

        #Draw the images for debugging and feedback
        cv2.imshow("Image 1", img1gray)
        cv2.imshow("Image 2", img2gray)
        cv2.waitKey(3)
        
        #If both images have chessboard points, store them 
        if ret1 and ret2:
            if len(corners1) == len(corners2) and len(corners1) == len(self.objp):
                self.imagePoints1.append(corners1)
                self.imagePoints2.append(corners2)
                self.objectPoints.append(self.objp)
            else:
                print "One of the images didn't have enough points."
        else:
            print "No checkerboard found"
        
        #Have enough points, so do the calibration
        if len(self.imagePoints2) > self.sampleCount:
            objectPoints32 = np.array(self.objectPoints, dtype=np.float32)
            image1Points32 = np.array(self.imagePoints1, dtype=np.float32)
            image2Points32 = np.array(self.imagePoints2, dtype=np.float32)
            
            retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv2.stereoCalibrate(objectPoints32, image1Points32, image2Points32, (640,480))
            if not retval:
                print "Calibration failed, so sad"
                sys.exit()
            else:
                print "  ---  "
                print "cameraMatrix1: {0}".format(cameraMatrix1)
                print "distCoeffs1: {0}".format(distCoeffs1)
                print "cameraMatrix2: {0}".format(cameraMatrix2)
                print "distCoeffs2: {0}".format(distCoeffs2)
                print "R: {0}".format(R)
                print "T: {0}".format(T)
                print "E: {0}".format(E)
                print "F {0}".format(F)
                
                fname = time.strftime("./stereo_cal_%b-%d-%y-%H.%M.%S.pickle")
                camCal = {"cameraMatrix1" : cameraMatrix1,
                          "distCoeffs1" : distCoeffs1,
                          "cameraMatrix2" : cameraMatrix2,
                          "distCoeffs2" : distCoeffs2,
                          "R" : R, "T" : T, "E" : E, "F" : F}
                with open(fname, "w") as outFile:
                    pickle.dump(camCal, outFile)
                    print "Saved as {0}".format(fname)
                    sys.exit()
        else:
            print "Have {0} out of {1} frames".format(len(self.imagePoints2), self.sampleCount)

if __name__ == '__main__':
    
    rospy.init_node("dual_checkerboard_cal")
    
    #Launch the launch/test/openni_dual_cam.launch file
    imageSubA = message_filters.Subscriber('/first_device/rgb/image_rect_color', Image)
    imageSubB = message_filters.Subscriber('/second_device/rgb/image_rect_color', Image)

    sr = StereoRegistrator()
    
    #Args are list of topics to subscribe to, buffer length, and how close is "good enough" for synch (in seconds)
    ts = message_filters.ApproximateTimeSynchronizer([imageSubA, imageSubB], 10, 0.02)
    ts.registerCallback(sr.capturePoints)
    rospy.spin()
    cv2.destroyAllWindows()
    