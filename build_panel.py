#!/usr/bin/env python

from __future__ import print_function
#from scipy.spatial import distance as dist
from sensor_msgs.msg import PointCloud2 
from sensor_msgs.msg import Image 
from sensor_msgs.msg import LaserScan

'''
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
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
import PyKDL
from tf_conversions import posemath
'''

#import rospy
import numpy as np
import cv2
import time

#import sensor_msgs.msg
from cv_bridge import CvBridge     


def show_head(head_msg):
    print ('head ', head_msg)

if __name__ == '__main__':

    header ='''#!/usr/bin/env python
import copy
import time
import rospy
import tf
import tf2_ros
import numpy
import time
'''
    print(header)
    
    with open('top_msg.txt') as f:
        msgs = f.readlines()
        
    #print( 'len(msgs) = ', len(msgs))
    msg_arr=[]
    for i in range(len(msgs)-1):
        msg1 = str(msgs[i].lstrip(' ').split(':')[0])
        msg = str(msgs[i].lstrip(' ').split(':')[1]).lstrip(' ')
        topic = msg1.split(' ')[0].split(' /')
        msg = msg.split("/")
        ms =msg[1].split(' ')[0].rstrip('\n')
        if ms not in msg_arr:
            print('from '+msg[0]+'.msg import '+ms)
            msg_arr.append(ms)
    print ('' )   

    print('class vcp():')

    msg_arr=[]    
    for i in range(len(msgs)-1):
        msg1 = str(msgs[i].lstrip(' ').split(':')[0])
        msg = str(msgs[i].lstrip(' ').split(':')[1]).lstrip(' ')
        topic = msg1.split(' ')[0].split(' /')
        msg = msg.split("/")
        ms = msg[1].split(' ')[0].rstrip('\n')
        #print('from '+msg[0]+' import '+msg[1].split(' ')[0].rstrip('\n'))
        if ms not in msg_arr:
            msg_arr.append(ms)

            if ms=='Image':
                print('    def '+ms+'(self,msg,lf_rt):')
                print('        print("'+ms+' = Video object goes here")')
            else:  
                print('    def '+ms+'(self,msg):')
                print('        print("'+ms+' =", msg)')
            #print('        rospy.sleep(.1)\n        time.sleep(.1)')
            #print('        rospy.sleep(.001)')
            print('        time.sleep(.1)')
            print('')

    print("if __name__ == '__main__':")
    print('    cp = vcp()\n')

    print("    rospy.init_node('Valkyrie_Control_Panel')")

    msg_arr=[]
    for i in range(len(msgs)-1):
        msg1 = str(msgs[i].lstrip(' ').split(':')[0])
        msg = str(msgs[i].lstrip(' ').split(':')[1]).lstrip(' ')
        topic = msg1.split(' ')[0].split(' /')
        msg = msg.split("/")
        #print ("messge =", msg)
        ms = msg[len(msg)-1].split(' ')[0].rstrip('\n')
        if ms not in msg_arr:
            msg_arr.append(ms)
            if 'control' not in (topic[0].split('/')):
                if ms =="Image":
                    print('    lf_rt = "LEFT"')
                    print('    rospy.Subscriber("'+str(topic[0].lstrip('/'))+'", '+ms+', cp.'+ms+',lf_rt)')
                else:
                    print('    rospy.Subscriber("'+str(topic[0].lstrip('/'))+'", '+ms+', cp.'+ms+')')
            else:
                print('    #Valkyrie control '+ms+' throw away for now * write input driver later')

    #print ('    rate = rospy.Rate(200) # 10hz\n    while True:\n        time.sleep(3)\n        rate.sleep()')       
    print('    rospy.spin()')
