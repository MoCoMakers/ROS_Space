#!/usr/bin/env python

import copy
import time
import rospy

from numpy import append

from ihmc_msgs.msg import ArmTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage

from ihmc_msgs.msg import HeadTrajectoryRosMessage
from ihmc_msgs.msg import SO3TrajectoryPointRosMessage
from ihmc_msgs.msg import HeadTrajectoryRosMessage
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

from ihmc_msgs.msg import NeckTrajectoryRosMessage
from ihmc_msgs.msg import SE3TrajectoryPointRosMessage

from ihmc_msgs.msg import PelvisTrajectoryRosMessage
from ihmc_msgs.msg import ChestTrajectoryRosMessage


ZERO_VECTOR1 = [0.0, -1.2, 2.0, 1.0, 0.0, 0.0, 0.0]

ZERO_VECTOR = [-1.1, .75, 1.0, 1.0, 0.0, 0.0, 0.0]
ZERO_VECTOR2 = [-1.1, .70, 1.0, 1.5, 0.0, 0.0, 0.0]

LOWER_ARM = [-0.0, 1.5, 0.5, 1.0, 0.0, 0.0, 0.0]
LOWER_ARM_LF = [-0.0, -1.5, 0.5, 0.0, 0.0, 0.0, 0.0]
LOWER_ARM2 = [-0.5, -1.0, 0.0, 2.0, 0.0, 0.0, 0.3]
#ARM_UP0 =   [-1.1, 0.2, 1.35, 0.3, 0.0, 0.0, 0.0]
ARM_UP0 =   [-0.5, 0.1, 1.0, 2.0, 0.0, 0.0, 0.0]

ARM_UP1 =   [-1.5, -0.5, 1.5, 1.0, 0.0, 0.0, 0.0]
#ARM_UP2 =   [-1.1, 0.37, 1.35, 0.0, 0.0, 0.0, 0.0]
ARM_UP2 =   [-1.5, 0.2, 1.25, 0.6, 0.0, 0.0, -0.1]
PUNCH =     [-1.5, 1.5, 1.0, 1.0, 0.0, 0.0, 0.0]

ELBOW_BENT_UP = [0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0]
#ELBOW_BENT_UP = [0.0, 2.0, 0.0, 0.0, 0.0, 2.0, 0.0]

HEAD_DOWN = [0.0,0.0,0.2]
ROBOT_NAME = None
uid = -1

def sendRightArmTrajectory():
    global uid
    msg = ArmTrajectoryRosMessage()

    msg.robot_side = ArmTrajectoryRosMessage.RIGHT

    #msg = appendTrajectoryPoint(msg, 2.0, ZERO_VECTOR)
    #msg = appendTrajectoryPoint(msg, 2.0, ELBOW_BENT_UP)
    msg = appendTrajectoryPoint(msg, 0.2, ARM_UP0)
    #time.sleep(1)
    msg = appendTrajectoryPoint(msg, 0.2, ARM_UP2)
    #time.sleep(2)
    #msg = appendTrajectoryPoint(msg, 0.5, ARM_UP2)
    #time.sleep(2)
    #msg = appendTrajectoryPoint(msg, 1.0, PUNCH)

    uid -=1
    msg.unique_id = uid
    print uid
    rospy.loginfo('publishing right trajectory')
    armTrajectoryPublisher.publish(msg)

def sendLeftArmDown():
    global uid
    msg = ArmTrajectoryRosMessage()

    msg.robot_side = ArmTrajectoryRosMessage.LEFT

    msg = appendTrajectoryPoint(msg, 0.5, LOWER_ARM_LF)

    msg.unique_id = time.time()

    rospy.loginfo('publishing left dn trajectory')
    armTrajectoryPublisher.publish(msg)

def sendRightArmDown():
    global uid
    msg = ArmTrajectoryRosMessage()

    msg.robot_side = ArmTrajectoryRosMessage.RIGHT

    msg = appendTrajectoryPoint(msg, 0.5, LOWER_ARM)

    uid +=1
    msg.unique_id = time.time()

    rospy.loginfo('publishing right dn trajectory')
    armTrajectoryPublisher.publish(msg)

def lower_head():
    msg = HeadTrajectoryRosMessage()
    
    point = SO3TrajectoryPointRosMessage()
    point.time = 2.0
    point.orientation.x = 2.0
    point.orientation.y = 2.0
    point.orientation.z = 2.0
    point.orientation.w = 2.0
    point.unique_id=time.time()
 
    point.angular_velocity.x =2.2
    point.angular_velocity.y =2.4
    point.angular_velocity.z =2.6
  
    msg.taskspace_trajectory_points.append(point)

    msg.unique_id = time.time()

    #msg.unique_id = uid
    #msg.previous_message_id = msg.unique_id+1
    #msg.execution_mode = 0

    print 'head trajectory ',msg
    print time.time()+1
    print ''

    rospy.loginfo('publishing head trajectory')
    headTrajectoryPublisher.publish(msg)

def turn_pelvis():
    msg = PelvisTrajectoryRosMessage()
    
    point = SE3TrajectoryPointRosMessage()
    point.time = 1.0
    point.position.x = 0.0
    point.position.y = 0.0
    point.position.z = 0.0

    point.orientation.x = 0.0
    point.orientation.y = 0.0
    point.orientation.z = 0.0
    point.orientation.w = 0.0
    point.unique_id=time.time()
 
    point.angular_velocity.x =0.0
    point.angular_velocity.y =0.0
    point.angular_velocity.z =0.0

    point.linear_velocity.x =0.0
    point.linear_velocity.y =0.0
    point.linear_velocity.z =0.0
  
    msg.taskspace_trajectory_points.append(point)

    msg.unique_id = time.time()

    print 'pelvis trajectory ',msg
    print ''

    rospy.loginfo('publishing pelvis trajectory')
    pelvisTrajectoryPublisher.publish(msg)

def turn_chest():
    msg = ChestTrajectoryRosMessage()
    
    point = SO3TrajectoryPointRosMessage()
    point.time = 0.5
    point.orientation.x = 0.0
    point.orientation.y = 0.00
    point.orientation.z = -0.15
    point.orientation.w = 1.0

    point.unique_id=time.time()
 
    point.angular_velocity.x =0.0
    point.angular_velocity.y =0.0
    point.angular_velocity.z =0.05

  
    msg.taskspace_trajectory_points.append(point)

    msg.unique_id = time.time()

    #print 'chest trajectory ',msg
    #print ''

    rospy.loginfo('publishing chest trajectory')
    chestTrajectoryPublisher.publish(msg)


NECK1 = [0.3, 0.5, -0.5]

def sendNeckTrajectory():
    global uid
    msg = NeckTrajectoryRosMessage()

    msg = appendTrajectoryPoint(msg, 2.0, NECK1)
    time.sleep(2)
    #msg = appendTrajectoryPoint(msg, 0.5, ARM_UP2)
    #time.sleep(2)
    #msg = appendTrajectoryPoint(msg, 0.5, ARM_UP2)
    #time.sleep(2)
    #msg = appendTrajectoryPoint(msg, 1.0, PUNCH)

    
    msg.unique_id = time.time()

    rospy.loginfo('publishing neck trajectory')
    neckTrajectoryPublisher.publish(msg)


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
'''
def appendTrajectoryPoint(neck_trajectory, time, positions):
    if not neck_trajectory.joint_trajectory_messages:
        neck_trajectory.joint_trajectory_messages = [copy.deepcopy(OneDoFJointTrajectoryRosMessage()) for i in range(len(positions))]
    for i, pos in enumerate(positions):
        point = TrajectoryPoint1DRosMessage()
        point.time = time
        point.position = pos
        point.velocity = 0

        neck_trajectory.joint_trajectory_messages[i].trajectory_points.append(point)
    return arm_trajectory
'''
if __name__ == '__main__':
    try:
        rospy.init_node('ihmc_arm_demo1')

        ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')
       
        armTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/arm_trajectory".format(ROBOT_NAME), ArmTrajectoryRosMessage, queue_size=1)
        neckTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/neck_trajectory".format(ROBOT_NAME), NeckTrajectoryRosMessage, queue_size=1)
        
	headTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/head_trajectory".format(ROBOT_NAME), HeadTrajectoryRosMessage, queue_size=1)

	pelvisTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/pelvis_trajectory".format(ROBOT_NAME), PelvisTrajectoryRosMessage, queue_size=1)
	chestTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/chest_trajectory".format(ROBOT_NAME), ChestTrajectoryRosMessage, queue_size=1)
	
        rate = rospy.Rate(10) # 10hz
        time.sleep(1)

        # make sure the simulation is running otherwise wait
        if armTrajectoryPublisher.get_num_connections() == 0:
            rospy.loginfo('waiting for subscriber...')
            while armTrajectoryPublisher.get_num_connections() == 0:
                rate.sleep()

        if not rospy.is_shutdown():
            #lower_head()
            #sendRightArmTrajectory()
            #lower_head()
	    #sendNeckTrajectory()
            #turn_pelvis()
            #sendLeftArmDown()
            
            #time.sleep(1)
            #turn_chest()
            #time.sleep(1)
            sendRightArmTrajectory()
            time.sleep(2)
            sendRightArmDown()
            
    except rospy.ROSInterruptException:
        pass
