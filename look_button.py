#!/usr/bin/env python

from __future__ import print_function
#from scipy.spatial import distance as dist
from sensor_msgs.msg import PointCloud2 
from sensor_msgs.msg import Image 
from sensor_msgs.msg import LaserScan

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
from std_msgs.msg import Int32
from std_msgs.msg import Float64
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
        self.light_on = False
	self.cnt=0
        self.light_cnt =0
        # find all the 'color' shapes in the image
        self.greenlower = np.array([0, 120, 0])
        self.greenupper = np.array([20, 255, 20])
        self.bluelower = np.array([120, 0, 0])
        self.blueupper = np.array([255, 20, 20])
        self.redlower = np.array([0, 0, 50])
        self.redupper = np.array([30, 30, 255])
        self.colors=['red','green','blue']
        self.color_val=[(0,0,255),(0,255,0),(255,0,0)]

        self.NUM_JOINTS = 28
        # Latest message from /atlas/atlas_sim_interface_state
        self.asis_msg = None
        # Latest message from /atlas/imu
        self.imu_msg = None

        # Set up publishers / subscribers
        src_start_pub = rospy.Publisher('/srcsim/qual1/start', Empty, queue_size=1)
        spindle_speed = rospy.Publisher('/multisense/set_spindle_speed', Float64, queue_size=1)
        self.src_light_pub = rospy.Publisher('/srcsim/qual1/light', Console, queue_size=1)
        #self.src_head_pub = rospy.Publisher('/ihmc_ros/valkyrie/control/head_trajectory', HeadTrajectoryRosMessage, queue_size=1)

        rospy.init_node('ros_video')
        # Wait for subscribers to hook up, lest they miss our commands
        time.sleep(2.0)
        print ('init')
        src_start_pub.publish()
        spindle_speed.publish(int(50))
        print ('sent start')

    def show_img(self, imgmsg):
        #print 'go**************************'
        global roi,roi2
        img = br.imgmsg_to_cv2(imgmsg, "bgr8")
        #blue, green, red = cv2.split(img)
        #cv2.imshow('green',green)
        #img = cv2.imread('img.jpg')
        img1 = cv2.imread('img2.jpg')
        img2 = img.copy()
	frame = img.copy()
        img = cv2.flip(frame,-1)

        #cv2.imshow('f',frame)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges2 = cv2.Canny(gray,100,200)
        # allocate temporary images
        #new_size = (int(img.shape[1] / image_scale), int(img.shape[0] / image_scale))
        
	# find all the 'red' shapes in the image

	green = cv2.inRange(img, self.greenlower, self.greenupper)
	blue = cv2.inRange(img, self.bluelower, self.blueupper)
	red = cv2.inRange(img, self.redlower, self.redupper)
        #cv2.bitwise_or(shapeMask,img,img2)
        # convert color input image to grayscale
        #gray = cv2.cvtColor(shapeMask, cv2.COLOR_BGR2GRAY) 

        #frame = cv2.GaussianBlur(green, (9,9), 2, 2)
        #grn = cv2.Canny(green,100,200)
        #blu = cv2.Canny(blue,100,200)
        rd = cv2.Canny(red,100,200)
        #both= blu.copy()
        #cv2.bitwise_or(grn,blu,both)
        #cv2.bitwise_or(rd,both,both)
        #cv2.bitwise_or(edges2,both,both)
        #cv2.imshow('red',rd)
        #cv2.imshow('green',grn)
        #cv2.imshoalew('blue',blu)
        #cv2.imshow('blue_green',both)

        color_index, x,y = self.get_color((red,green,blue)) 
        if color_index!=-1:
           if self.light_on==False:
              rgb = self.color_val[color_index]
              #publish color x,y,z r,g,b
              self.src_light_pub.publish(float(x),5.0,float(y),rgb[0],rgb[1],rgb[2])
	      if self.light_cnt==20:
                 rospy.is_shutdown(True)
              self.light_cnt+=1
              print('color, cnt & x y is:',self.colors[color_index],self.light_cnt,x,y)
              
            
              self.light_on = True
        else:
           if self.light_on==True:
              print('Light is off')
              self.light_on = False
        cv2.rectangle(img,(x-10,y-20),(x+10,y+20),(0,255,0),3)
        cv2.imshow("console", img)
        #cv2.imshow("console lit", img1)
	#cv2.imshow('sm1',roi2)img2)
        #cv2.imshow("black shapes", img2)
        #cv2.imshow('outer edges',edges2)
        #cv2.imshow('all edges', red)
        #lights =cv2.medianBlur(edges1, 3)

        self.cnt+=1
        
	cv2.waitKey(2)

        smblur = cv2.GaussianBlur(red, (9,9), 2, 2)
        edges_sm = cv2.Canny(smblur,100,200)           

    def get_color(self,colors):
        for i in range(3):
            contours,h = cv2.findContours(colors[i],1,2)
            for ct in contours:
                if cv2.contourArea(ct)>60:
                    #print('contour= ',ct)
		   
		    # find center
                    M = cv2.moments(ct)
		    cX = int(M["m10"] / M["m00"])
	            cY = int(M["m01"] / M["m00"])
                    return i,cX, cY
        return -1,0,0

class Lidar():

   def on_scan(self, scan):
       print(scan.ranges)

   def on_cloud(self, cloud):
       print(cloud.ranges)
def show_head(head_msg):
    print ('head ', head_msg)

if __name__ == '__main__':

    f_led = Find_leds()
    lscan = Lidar()

    br = CvBridge()
    
    #rospy.Publisher('/srcsim/qual1/start', Empty, queue_size=1)

    #rospy.Subscriber('multisense_sl/camera/left/image_raw', sensor_msgs.msg.Image, show_img)
    rospy.Subscriber('multisense/camera/left/image_raw', sensor_msgs.msg.Image, f_led.show_img)
    #rospy.Subscriber('multisense/image_point2', sensor_msgs.msg.PointCloud2, lscan.on_cloud)

    #rospy.Subscriber('multisense/lidar_scan', sensor_msgs.msg.LaserScan, lscan.on_scan)
    rospy.Subscriber('ihmc_ros/valkyrie/control/head_trajectory',HeadTrajectoryRosMessage, show_head)
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


