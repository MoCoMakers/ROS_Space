#!/usr/bin/env python

from __future__ import print_function
from scipy.spatial import distance as dist
from collections import OrderedDict

import roslaunch

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["srcsim", "qual1.launch"])
 
import subprocess,os

import roslib
'''
roslib.load_manifest('switch_control_mode')
roslib.load_manifest('ihmc_ros_core')
roslib.load_manifest('ihmc_ros_common')
roslib.load_manifest('ihmc_ros_java_adapter')
roslib.load_manifest('ihmc_msgs')
'''
import rospy
import time
from srcsim.msg import Console
from ihmc_msgs.msg import HeadTrajectoryRosMessage 
from std_msgs.msg import Empty
from sensor_msgs.msg import Imu
import PyKDL
from tf_conversions import posemath


#import rospy
import numpy as np
import cv2
import time

import sensor_msgs.msg
from cv_bridge import CvBridge     

class Find_leds:
    
    def __init__(self):
	self.cnt=0
        # find all the 'black' shapes in the image
        self.greenlower = np.array([0, 120, 0])
        self.greenupper = np.array([20, 255, 20])
        self.bluelower = np.array([120, 0, 0])
        self.blueupper = np.array([255, 20, 20])
        self.redlower = np.array([0, 0, 120])
        self.redupper = np.array([20, 20, 255])

        self.NUM_JOINTS = 28
        # Latest message from /atlas/atlas_sim_interface_state
        self.asis_msg = None
        # Latest message from /atlas/imu
        self.imu_msg = None

        # Set up publishers / subscribers
        src_start_pub = rospy.Publisher('/srcsim/qual1/start', Empty, queue_size=1)
        self.src_light_pub = rospy.Publisher('/srcsim/qual1/light', Console, queue_size=1)
        self.src_light_pub = rospy.Publisher('/ihmc_ros/valkyrie/control/head_trajectory', HeadTrajectoryRosMessage, queue_size=1)

        '''
        self.asic_pub = rospy.Publisher('atlas/atlas_sim_interface_command', 
          AtlasSimInterfaceCommand, queue_size=1)
        self.asis_sub = rospy.Subscriber('atlas/atlas_sim_interface_state', 
          AtlasSimInterfaceState, self.state_cb)
        self.imu_sub = rospy.Subscriber('atlas/imu', 
          Imu, self.imu_cb)
        '''
        rospy.init_node('ros_video')
        # Wait for subscribers to hook up, lest they miss our commands
        time.sleep(2.0)
        print ('init')
        src_start_pub.publish()
        print ('sent start')

    def show_img(self, imgmsg):
        #print 'go**************************'
        global roi,roi2
        img = br.imgmsg_to_cv2(imgmsg, "bgr8")
        blue, green, red = cv2.split(img)
        #cv2.imshow('green',green)
        #img = cv2.imread('img.jpg')
        img1 = cv2.imread('img2.jpg')
        img2 = img.copy()
	frame = img.copy()
        img = cv2.flip(frame,-1)
        #cv2.imshow('f',frame)
 
        # allocate temporary images
        #new_size = (int(img.shape[1] / image_scale), int(img.shape[0] / image_scale))
        
	# find all the 'blue' shapes in the image

	green = cv2.inRange(img, self.greenlower, self.greenupper)
	blue = cv2.inRange(img, self.bluelower, self.blueupper)
	red = cv2.inRange(img, self.redlower, self.redupper)
        #cv2.bitwise_or(shapeMask,img,img2)
        # convert color input image to grayscale
        #gray = cv2.cvtColor(shapeMask, cv2.COLOR_BGR2GRAY) 

        #frame = cv2.GaussianBlur(green, (9,9), 2, 2)
        grn = cv2.Canny(green,100,200)
        blu = cv2.Canny(blue,100,200)
        rd = cv2.Canny(red,100,200)
        #both = cv2.bitwise_or(grn,blu)
        cv2.imshow('red',rd)
        cv2.imshow('green',grn)
        cv2.imshow('blue',blu)
        contours,h = cv2.findContours(red,1,2)
        for ct in contours:
            #print('approx',approx)
	
            #if len(approx)>2 and len(approx)<7:
            #if cv2.contourArea(ct)>20 and cv2.isContourConvex(ct)==True:
            if cv2.contourArea(ct)>60:
                print('contour= ',ct)

        smblur = cv2.GaussianBlur(red, (9,9), 2, 2)
        edges_sm = cv2.Canny(smblur,100,200)
 
        edges2 = cv2.Canny(frame,100,200) 
        cv2.imshow("console", img)
        #cv2.imshow("console lit", img1)
	#cv2.imshow('sm1',roi2)img2)
        #cv2.imshow("black shapes", img2)
        #cv2.imshow('outer edges',edges2)
        #cv2.imshow('all edges', red)
        #lights =cv2.medianBlur(edges1, 3)
        '''
        if cnt==0:
           roi2=edges_sm.copy()
           roi1=img2.copy()
        #cv2.imshow('roi2',roi2)
        '''
        #cv2.imshow('edges_sm',edges_sm)
        
        if self.cnt>0:
	   #cv2.imshow('sm1',roi2)
           cv2.bitwise_xor(roi2,edges_sm,roi2)       
	   #  cv2.imshow('sm2',roi2)
        #roi2=edges_sm.copy()
 
        rows,cols = edges_sm.shape
        roi2 = edges_sm[0:rows, 0:cols ]

        #ret,thresh = cv2.threshold(gray,127,255,1)

        contours,h = cv2.findContours(edges_sm,1,2)

        #print 'len contours = ', len(contours)
        img2[:] = (0,0,0)

        cv2.drawContours(img2,contours,-1,(255,255,255),1)
        if self.cnt>0:

           cv2.bitwise_xor(roi,img2,roi)       
	   #  cv2.imshow('contours',roi)

        rows,cols,channels = img2.shape
        roi = img2[0:rows, 0:cols ]

        img2[:] = (0,0,0)

     
        for ct in contours:
            #print('approx',approx)
	
            #if len(approx)>2 and len(approx)<7:
            #if cv2.contourArea(ct)>20 and cv2.isContourConvex(ct)==True:
            if cv2.contourArea(ct)>60:
	       #print (ct)
	       approx = cv2.approxPolyDP(ct,0.01*cv2.arcLength(ct,True),True)
               if len(approx)>1 and len(approx)<400:

	          x1,y1 = ct[0][0][0],ct[0][0][1]
	          x2,y2 =ct[2][0][0],ct[2][0][1]

	          area = (x2-x1)*(y2-y1)

	          cv2.drawContours(img2,[ct],-1,(255,255,255),-1)
                  #  cv2.imshow('contour',img2)
                  
		  #newcontours.append( cv2.convexHull(ct))
                  
        #  cv2.imshow('Leds', img2)
        '''
        if self.cnt ==5:
           cv2.imwrite( "console.jpg", img )
  	   cv2.imwrite( "console_edges.jpg", edges2 )
           cv2.imwrite("panel_edges.jpg", edges1)
           cv2.imwrite( "contour.jpg", img2 )
           cv2.imwrite( "leds2.jpg", shapeMask )
           cv2.imwrite( "contours.jpg", roi )
           cv2.imwrite( "debounced_shapeMask_edges.jpg", roi2 )
        '''
        self.cnt+=1
        
	cv2.waitKey(2)



if __name__ == '__main__':

    f_led = Find_leds()

    br = CvBridge()
    
    #rospy.Publisher('/srcsim/qual1/start', Empty, queue_size=1)

    #rospy.Subscriber('multisense_sl/camera/left/image_raw', sensor_msgs.msg.Image, show_img)
    rospy.Subscriber('multisense/camera/left/image_raw', sensor_msgs.msg.Image, f_led.show_img)

    #rospy.Subscriber(options.ctopic, sensor_msgs.msg.CompressedImage, compressed_detect_and_draw)
    rospy.spin()       




def sendHeadTrajectory():
    msg = ArmTrajectoryRosMessage()

    msg.robot_side = ArmTrajectoryRosMessage.RIGHT

    msg = appendTrajectoryPoint(msg, 2.0, ZERO_VECTOR)
    msg = appendTrajectoryPoint(msg, 3.0, ELBOW_BENT_UP)
    msg = appendTrajectoryPoint(msg, 4.0, ZERO_VECTOR)

    msg.unique_id = -1

    rospy.loginfo('publishing right trajectory')
    armTrajectoryPublisher.publish(msg)

def appendTrajectoryPoint(arm_trajectory, time, positions):
    if not arm_trajectory.joint_trajectory_messages:
        arm_trajectory.joint_trajectory_messages = [copy.deepcopy(OneDoFJointTrajectoryRosMessage()) for i in range(len(positions))]
    for i, pos in enumerate(positions):
        point = TrajectoryPoint1DRosMessage()
        point.time = time
        point.position = pos
        point.velocity = 0
        arm_trajectory.joint_trajectory_messages[i].trajectory_points.append(point)
    return arm_trajectory


