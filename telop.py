#!/usr/bin/env python

import time
import rospy
import tf
import tf2_ros
import numpy

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

from ihmc_msgs.msg import FootstepStatusRosMessage
from ihmc_msgs.msg import FootstepDataListRosMessage
from ihmc_msgs.msg import FootstepDataRosMessage

################################# pygame ##################################
import pygame

pygame.init()
clock = pygame.time.Clock()
pygame.joystick.init()
white = (255,255,255)
black=(0,0,0)

screen = pygame.display.set_mode((640, 480))
joy = pygame.joystick.Joystick(0)

bk = pygame.Surface(screen.get_size())
bk = bk.convert()
bk.fill((0,0,0))

joy.init()
running = 1

xy = [320,240,320,240]
lxy =[320,240,320,240]

pygame.draw.circle(screen,white,(320,240), 2)

#################################################################################

LEFT = 0
RIGHT = 1

ROBOT_NAME = None
LEFT_FOOT_FRAME_NAME = None
RIGHT_FOOT_FRAME_NAME = None

def walkThru(x,y):
    msg = FootstepDataListRosMessage()
    msg.transfer_time = 0.5    # demo val 1.5
    msg.swing_time = 0.5     # demo val 1.5
    msg.execution_mode = 0
    msg.unique_id = -1

    # walk forward starting LEFT
    '''
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.0, -0.1, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.0,  -0.8, 0.0]))    

    '''
    if x<0 or y<0:
        msg.footstep_data_list.append(createFootStepOffset(RIGHT,[y, x, 0.0]))     
        msg.footstep_data_list.append(createFootStepOffset(LEFT, [y, x, 0.0]))
    else:   
        msg.footstep_data_list.append(createFootStepOffset(LEFT,[y, x, 0.0]))     
        msg.footstep_data_list.append(createFootStepOffset(RIGHT, [y, x, 0.0]))

    '''
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.4, 0.3, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.6, 0.28, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.8, 0.3, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [1.0, 0.28, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [1.2, 0.3, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [1.4, 0.28, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [1.6, 0.3, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [1.8, 0.28, 0.0]))   
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [2.0, 0.3, 0.0]))
    
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [2.25, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [2.45, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [2.65, 0.0, 0.0]))
    
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [2.75, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [2.82, 0.0, 0.0]))
    '''
    footStepListPublisher.publish(msg)
    rospy.loginfo('walk forward...')
    waitForFootsteps(len(msg.footstep_data_list))

# Creates footstep with the current position and orientation of the foot.
def createFootStepInPlace(stepSide):
    footstep = FootstepDataRosMessage()
    footstep.robot_side = stepSide

    if stepSide == LEFT:
        foot_frame = LEFT_FOOT_FRAME_NAME
    else:
        foot_frame = RIGHT_FOOT_FRAME_NAME

    footWorld = tfBuffer.lookup_transform('world', foot_frame, rospy.Time())
    footstep.orientation = footWorld.transform.rotation
    footstep.location = footWorld.transform.translation

    return footstep

# Creates footstep offset from the current foot position. The offset is in foot frame.
def createFootStepOffset(stepSide, offset):
    footstep = createFootStepInPlace(stepSide)

    # transform the offset to world frame
    quat = footstep.orientation
    rot = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
    transformedOffset = numpy.dot(rot[0:3, 0:3], offset)

    footstep.location.x += transformedOffset[0]
    footstep.location.y += transformedOffset[1]
    footstep.location.z += transformedOffset[2]

    return footstep

def waitForFootsteps(numberOfSteps):
    global stepCounter
    stepCounter = 0
    while stepCounter < numberOfSteps:
        rate.sleep()
    rospy.loginfo('finished set of steps')

def recievedFootStepStatus(msg):
    global stepCounter
    if msg.status == 1:
        stepCounter += 1

if __name__ == '__main__':
    try:
        rospy.init_node('ihmc_walk_test')

        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.logerr("Cannot run walk_test.py, missing parameters!")
            rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")

        else:
            ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')

            right_foot_frame_parameter_name = "/ihmc_ros/{0}/right_foot_frame_name".format(ROBOT_NAME)
            left_foot_frame_parameter_name = "/ihmc_ros/{0}/left_foot_frame_name".format(ROBOT_NAME)

            if rospy.has_param(right_foot_frame_parameter_name) and rospy.has_param(left_foot_frame_parameter_name):
                RIGHT_FOOT_FRAME_NAME = rospy.get_param(right_foot_frame_parameter_name)
                LEFT_FOOT_FRAME_NAME = rospy.get_param(left_foot_frame_parameter_name)

                footStepStatusSubscriber = rospy.Subscriber("/ihmc_ros/{0}/output/footstep_status".format(ROBOT_NAME), FootstepStatusRosMessage, recievedFootStepStatus)
                footStepListPublisher = rospy.Publisher("/ihmc_ros/{0}/control/footstep_list".format(ROBOT_NAME), FootstepDataListRosMessage, queue_size=1)

                tfBuffer = tf2_ros.Buffer()
                tfListener = tf2_ros.TransformListener(tfBuffer)

                rate = rospy.Rate(5) # 10hz
                time.sleep(1)

                # make sure the simulation is running otherwise wait
                if footStepListPublisher.get_num_connections() == 0:
                    rospy.loginfo('waiting for subsciber...')
                    while footStepListPublisher.get_num_connections() == 0:
                        rate.sleep()

                if not rospy.is_shutdown():
                   x=0
                   y=0
                   while running:
                       for event in pygame.event.get():
                           if event.type == pygame.QUIT:
                               running = 0
                       
                       for i in range(joy.get_numbuttons()):
                           if joy.get_button(i) == 1:
                               print 'button pressed', i
                               if i==7:
                                   running = 0
                       
                       for i in range(joy.get_numaxes()):
                           axis = joy.get_axis(i)
                           if axis != 0:
                               if axis > 0 and i ==0 :
                                   x=-0.1
                                   y=0.0
                               if axis > 0 and i== 1:
                                   x=0.0
                                   y=-0.1
                               if axis < 0 and i == 0:
                                   x=0.1
                                   y=0.0
                               if axis < 0 and i==1:
                                   x=0.0
                                   y=0.1

                               lxy[i] = xy[i]
                               print 'i= ', i, axis, x, y
                               move = axis * 4
                               xy[i] = int(move) + xy[i]
                       if not ( x==0 and y==0):
                          print '******************************'
                          print 'i= ', i, axis, x, y
                          time.sleep(3)
                          walkThru(x,y)
                          x=0
                          y=0 
                       screen.fill(black)
                       pygame.draw.circle(screen,white,(xy[0],xy[1]), 2)     

  
                       #screen.blit(bk, (0,0))
                       pygame.display.flip()
                       clock.tick(120)

            else:
                if not rospy.has_param(left_foot_frame_parameter_name):
                    rospy.logerr("Missing parameter {0}".format(left_foot_frame_parameter_name))
                if not rospy.has_param(right_foot_frame_parameter_name):
                    rospy.logerr("Missing parameter {0}".format(right_foot_frame_parameter_name))

    except rospy.ROSInterruptException:
        pass

pygame.joystick.quit()
pygame.quit()
