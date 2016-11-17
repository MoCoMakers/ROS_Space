#!/usr/bin/env python

from __future__ import print_function
from scipy.spatial import distance as dist
#from collections import OrderedDict
from matplotlib import pyplot as plt

import roslaunch

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["srcsim", "qual1.launch"])
 
import subprocess,os

import roslib

import rospy
import time
from srcsim.msg import Console
from ihmc_msgs.msg import HeadTrajectoryRosMessage 
from std_msgs.msg import Empty
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image

import PyKDL
from tf_conversions import posemath


#import rospy
import numpy as np
import cv2
import time

import sensor_msgs.msg
from cv_bridge import CvBridge     

ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
'''

def write_ply(fn, verts, colors):
    verts = verts.reshape(-1, 3)
    colors = colors.reshape(-1, 3)
    verts = np.hstack([verts, colors])
    with open(fn, 'wb') as f:
        f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
        np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')


class Find_leds:
    #Q = np.array(4, 4, cv2.CV_64F)
    
        
    def __init__(self):
        
        self.light_on = False
	self.cnt=0
        self.light_cnt =0
        # find all the 'color' shapes in the image
        self.greenlower = np.array([0, 120, 0])
        self.greenupper = np.array([20, 255, 20])
        self.bluelower = np.array([120, 0, 0])
        self.blueupper = np.array([255, 20, 20])
        self.redlower = np.array([0, 0, 120])
        self.redupper = np.array([20, 20, 255])
        self.colors=['red','green','blue']
        self.color_val=[(0,0,255),(0,255,0),(255,0,0)]

        self.NUM_JOINTS = 28
        # Latest message from /atlas/atlas_sim_interface_state
        self.asis_msg = None
        # Latest message from /atlas/imu
        self.imu_msg = None

        # Set up publishers / subscribers
        src_start_pub = rospy.Publisher('/srcsim/qual1/start', Empty, queue_size=1)
        self.src_light_pub = rospy.Publisher('/srcsim/qual1/light', Console, queue_size=1)
        self.src_head_pub = rospy.Publisher('/ihmc_ros/valkyrie/control/head_trajectory', HeadTrajectoryRosMessage, queue_size=1)

        #self.rt_cam = rospy.Subscriber('/multisense/camera/right/image_raw', Image)

        rospy.init_node('ros_video')
        # Wait for subscribers to hook up, lest they miss our commands
        time.sleep(2.0)
        #print ('init')
        src_start_pub.publish()
        #print ('sent start')
        self.img = cv2.imread('img.jpg',0)
        self.rt_img = self.img.copy()


    def show_img(self, imgmsg,lf_rt):
        #print('lf rt',lf_rt)
        self.cnt+=1
        #print 'go**************************'
        global roi,roi2
        #print ('l or r', lf_rt)
        if lf_rt=='left':

            self.img = br.imgmsg_to_cv2(imgmsg, "bgr8")
            img  = self.img
        else:
            #print ( lf_rt)
            self.rt_img = br.imgmsg_to_cv2(imgmsg, "bgr8")
        #print('cnt = ',self.cnt)
        #get right camera image
        
        if self.cnt%2==0 and self.cnt==8:

            cv2.imwrite('left.jpg',self.img)
            cv2.imwrite('right.jpg',self.rt_img)

            #print('cnt = 4 ',self.cnt)
            #stereo = cv2.StereoBM(1,16, 15)

            # disparity range is tuned for 'aloe' image pair
            window_size = 3
            min_disp = 16
            num_disp = 112-min_disp
            
            stereo = cv2.StereoSGBM()
            stereo.minDisparity = min_disp 
            stereo.numberOfDisparities = num_disp
            #stereo.blockSize = 16
            stereo.P1 = 8*3*window_size**2
            stereo.P2 = 32*3*window_size**2
            stereo.disp12MaxDiff = 1
            stereo.uniquenessRatio = 10
            stereo.speckleWindowSize = 100
            stereo.speckleRange = 32
            
	    frame1_new = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
	    frame2_new = cv2.cvtColor(self.rt_img, cv2.COLOR_BGR2GRAY)
            disp = stereo.compute(frame1_new,frame2_new)
            points = disp.copy()

            h, w = frame1_new.shape[:2]
            f = 0.622*w    # guess for focal length - width ~4.5m depth ~2.8m
            #print ('width, focal len =', w, f)

            Q = np.float32([[1, 0, 0, -0.5*w],
                    [0,-1, 0,  0.5*h], # turn points 180 deg around x-axis,
                    [0, 0, 0,     -f], # so that y-axis looks up
                    [0, 0, 1, 0]])
            
            points = cv2.reprojectImageTo3D(disp, Q)     
            f=open('points','w')
            f.write(points)
            f.close()

            cv2.imshow('depth',points)
            plt.imshow(points,'gray')
            
            colors = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
            mask = disp > disp.min()
            out_points = points[mask]
            out_colors = colors[mask]
            out_fn = 'out.ply'
            write_ply('out.ply', out_points, out_colors)

            #plt.show()

        if lf_rt=='left':        

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
            both= blu.copy()
            cv2.bitwise_or(grn,blu,both)
            cv2.bitwise_or(rd,both,both)
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
                  self.light_cnt+=1
                  #print('color, cnt & x y is:',self.colors[color_index],self.light_cnt,x,y)
                  print(self.colors[color_index],self.light_cnt,x,y)
                  self.light_on = True
            else:
               if self.light_on==True:
                  #print('Light is off')
                  self.light_on = False

            cv2.imshow("console", img)
            
            cv2.waitKey(2)
  

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



if __name__ == '__main__':

    f_led = Find_leds()

    br = CvBridge()
    
    #rospy.Publisher('/srcsim/qual1/start', Empty, queue_size=1)

    #rospy.Subscriber('multisense_sl/camera/left/image_raw', sensor_msgs.msg.Image, show_img)
    rospy.Subscriber('multisense/camera/right/image_raw', sensor_msgs.msg.Image, f_led.show_img,'right')

    rospy.Subscriber('multisense/camera/left/image_raw', sensor_msgs.msg.Image, f_led.show_img,'left')

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


