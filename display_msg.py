#!/usr/bin/env python
import copy
import time
import rospy
import tf
import tf2_ros
import numpy
import time

from rosgraph_msgs.msg import Clock
from val_hardware_msgs.msg import valAtiSensor
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import ModelStates
from dynamic_reconfigure.msg import ConfigDescription
from dynamic_reconfigure.msg import Config
from sensor_msgs.msg import JointState
from ihmc_msgs.msg import ArmTrajectoryRosMessage
from ihmc_msgs.msg import ChestTrajectoryRosMessage
from ihmc_msgs.msg import FootstepDataListRosMessage
from ihmc_valkyrie_ros.msg import ValkyrieLowLevelControlModeRosMessage
from std_msgs.msg import Int32
from ihmc_msgs.msg import Point2dRosMessage
from geometry_msgs.msg import Point32
from std_msgs.msg import Bool
from geometry_msgs.msg import WrenchStamped
from ihmc_msgs.msg import FootstepStatusRosMessage
from ihmc_msgs.msg import HighLevelStateChangeStatusRosMessage
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from ihmc_msgs.msg import WalkingStatusRosMessage
from val_hardware_msgs.msg import valImuSensor
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Log
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32

class vcp():
    def Clock(self,msg):
        print("Clock =", msg)
        time.sleep(.1)

    def valAtiSensor(self,msg):
        print("valAtiSensor =", msg)
        time.sleep(.1)

    def LinkStates(self,msg):
        print("LinkStates =", msg)
        time.sleep(.1)

    def ModelStates(self,msg):
        print("ModelStates =", msg)
        time.sleep(.1)

    def ConfigDescription(self,msg):
        print("ConfigDescription =", msg)
        time.sleep(.1)

    def Config(self,msg):
        print("Config =", msg)
        time.sleep(.1)

    def JointState(self,msg):
        print("JointState =", msg)
        time.sleep(.1)

    def ArmTrajectoryRosMessage(self,msg):
        print("ArmTrajectoryRosMessage =", msg)
        time.sleep(.1)

    def ChestTrajectoryRosMessage(self,msg):
        print("ChestTrajectoryRosMessage =", msg)
        time.sleep(.1)

    def FootstepDataListRosMessage(self,msg):
        print("FootstepDataListRosMessage =", msg)
        time.sleep(.1)

    def ValkyrieLowLevelControlModeRosMessage(self,msg):
        print("ValkyrieLowLevelControlModeRosMessage =", msg)
        time.sleep(.1)

    def Int32(self,msg):
        print("Int32 =", msg)
        time.sleep(.1)

    def Point2dRosMessage(self,msg):
        print("Point2dRosMessage =", msg)
        time.sleep(.1)

    def Point32(self,msg):
        print("Point32 =", msg)
        time.sleep(.1)

    def Bool(self,msg):
        print("Bool =", msg)
        time.sleep(.1)

    def WrenchStamped(self,msg):
        print("WrenchStamped =", msg)
        time.sleep(.1)

    def FootstepStatusRosMessage(self,msg):
        print("FootstepStatusRosMessage =", msg)
        time.sleep(.1)

    def HighLevelStateChangeStatusRosMessage(self,msg):
        print("HighLevelStateChangeStatusRosMessage =", msg)
        time.sleep(.1)

    def Imu(self,msg):
        print("Imu =", msg)
        time.sleep(.1)

    def String(self,msg):
        print("String =", msg)
        time.sleep(.1)

    def Odometry(self,msg):
        print("Odometry =", msg)
        time.sleep(.1)

    def WalkingStatusRosMessage(self,msg):
        print("WalkingStatusRosMessage =", msg)
        time.sleep(.1)

    def valImuSensor(self,msg):
        print("valImuSensor =", msg)
        time.sleep(.1)

    def CameraInfo(self,msg):
        print("CameraInfo =", msg)
        time.sleep(.1)

    def Image(self,msg,lf_rt):
        print("Image = Video object goes here")
        time.sleep(.1)

    def LaserScan(self,msg):
        print("LaserScan =", msg)
        time.sleep(.1)

    def Log(self,msg):
        print("Log =", msg)
        time.sleep(.1)

    def Float64(self,msg):
        print("Float64 =", msg)
        time.sleep(.1)

    def TFMessage(self,msg):
        print("TFMessage =", msg)
        time.sleep(.1)

    def Float32(self,msg):
        print("Float32 =", msg)
        time.sleep(.1)

if __name__ == '__main__':
    cp = vcp()

    rospy.init_node('Valkyrie_Control_Panel')
    rospy.Subscriber("clock", Clock, cp.Clock)
    rospy.Subscriber("force_torque_states", valAtiSensor, cp.valAtiSensor)
    rospy.Subscriber("gazebo/link_states", LinkStates, cp.LinkStates)
    rospy.Subscriber("gazebo/model_states", ModelStates, cp.ModelStates)
    rospy.Subscriber("gazebo/parameter_descriptions", ConfigDescription, cp.ConfigDescription)
    rospy.Subscriber("gazebo/parameter_updates", Config, cp.Config)
    rospy.Subscriber("hardware_joint_commands", JointState, cp.JointState)
    #Valkyrie control ArmTrajectoryRosMessage throw away for now * write input driver later
    #Valkyrie control ChestTrajectoryRosMessage throw away for now * write input driver later
    #Valkyrie control FootstepDataListRosMessage throw away for now * write input driver later
    #Valkyrie control ValkyrieLowLevelControlModeRosMessage throw away for now * write input driver later
    rospy.Subscriber("ihmc_ros/valkyrie/output/behavior", Int32, cp.Int32)
    rospy.Subscriber("ihmc_ros/valkyrie/output/capturability/capture_point", Point2dRosMessage, cp.Point2dRosMessage)
    rospy.Subscriber("ihmc_ros/valkyrie/output/capturability/center_of_mass", Point32, cp.Point32)
    rospy.Subscriber("ihmc_ros/valkyrie/output/capturability/is_in_double_support", Bool, cp.Bool)
    rospy.Subscriber("ihmc_ros/valkyrie/output/foot_force_sensor/left", WrenchStamped, cp.WrenchStamped)
    rospy.Subscriber("ihmc_ros/valkyrie/output/footstep_status", FootstepStatusRosMessage, cp.FootstepStatusRosMessage)
    rospy.Subscriber("ihmc_ros/valkyrie/output/high_level_state_change", HighLevelStateChangeStatusRosMessage, cp.HighLevelStateChangeStatusRosMessage)
    rospy.Subscriber("ihmc_ros/valkyrie/output/imu/pelvis_pelvisMiddleImu", Imu, cp.Imu)
    rospy.Subscriber("ihmc_ros/valkyrie/output/robot_motion_status", String, cp.String)
    rospy.Subscriber("ihmc_ros/valkyrie/output/robot_pose", Odometry, cp.Odometry)
    rospy.Subscriber("ihmc_ros/valkyrie/output/walking_status", WalkingStatusRosMessage, cp.WalkingStatusRosMessage)
    rospy.Subscriber("imu_states", valImuSensor, cp.valImuSensor)
    rospy.Subscriber("multisense/camera/left/camera_info", CameraInfo, cp.CameraInfo)
    lf_rt = "LEFT"
    rospy.Subscriber("multisense/camera/left/image_raw", Image, cp.Image,lf_rt)
    rospy.Subscriber("multisense/lidar_scan", LaserScan, cp.LaserScan)
    rospy.Subscriber("rosout", Log, cp.Log)
    rospy.Subscriber("rtt_period_overflow", Float64, cp.Float64)
    rospy.Subscriber("tf", TFMessage, cp.TFMessage)
    rospy.Subscriber("valkyrie/harness/velocity", Float32, cp.Float32)
    rospy.spin()
