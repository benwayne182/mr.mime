#!/usr/bin/env python

# running nao...
# ~/software/src/naoqi/naoqi-sdk/naoqi --verbose --broker-ip 127.0.0.1
# LD_LIBRARY_PATH=~/software/src/naoqi/naoqi-sdk/lib/:$LD_LIBRARY_PATH NAO_IP=169.254.129.144 roslaunch nao_driver nao_driver.launch
# roslaunch nao_description nao_state_publisher.launch
# rosrun rviz rviz
# rosservice call /body_stiffness/enable "{}"

import roslib; roslib.load_manifest('nao_package')
import rospy
from std_msgs.msg import Header
from std_msgs.msg import String
from nao_msgs.msg import JointAnglesWithSpeed 
from nao_msgs.msg import JointAnglesWithSpeedAction
from nao_msgs.msg import JointAngleTrajectory
from nao_msgs.msg import JointTrajectoryGoal
from nao_msgs.msg import JointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from rospy.rostime import Duration


import tf
from tf import TransformListener
from std_msgs.msg import Float64MultiArray
import math
import collections

import std_srvs.srv


import actionlib







# ['HeadYaw', 'HeadPitch',
#  'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll',
#  'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll',
#  'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll',
#  'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']
JointNames = ["HeadPitch","HeadYaw","LAnklePitch","LAnkleRoll","LElbowRoll","LElbowYaw","LHand","LHipPitch","LHipRoll","LHipYawPitch","LKneePitch","LShoulderPitch","LShoulderRoll","LWristYaw","RAnklePitch","RAnkleRoll","RElbowRoll","RElbowYaw","RHand","RHipPitch","RHipRoll","RHipYawPitch","RKneePitch","RShoulderPitch","RShoulderRoll","RWristYaw"]


State = collections.namedtuple('State', 'name, duration, gaze_tf, joints, speech, next')
States =[
    State( 'Gaze_ahead', 2, '', [ 0,0], '', 6),
     State( 'Gaze_right', 2, '', [-1,0], '', 4),
    State( 'Gaze_left',  2, '', [+1,0], '', 6),

 ]

def init(beginTime,wait):
    duration = rospy.Time.now()-beginTime
    while(duration.secs<wait):
        duration = rospy.Time.now()-beginTime
        continue
    header = Header()
    joints = States[0].joints
    msg_joints = JointAnglesWithSpeed( header, JointNames, joints, .2, 0)
    pub_joints = rospy.Publisher('/joint_angles', JointAnglesWithSpeed)
    pub_joints.publish(msg_joints)


def act(req):
    beginTime = rospy.Time.now()

    if req.command=="gaze_ahead":
        gaze(0)
    elif req.command=="gaze_right":
        gaze(1)
        init(beginTime,States[1].duration)

    elif req.command=="gaze_left":
        gaze(2)
        init(beginTime,States[2].duration)

    elif req.command=="nod":
        nod()
    elif req.command=="shake":
        shake()
    elif req.command=="move":
        move()        
        
    print "Done action :  %s "%(req.command)
    return "done"

def gaze(state):
    header = Header()
    joints = States[state].joints
    
    msg_joints = JointAnglesWithSpeed( header, JointNames, joints, .2, 0)
    pub_joints = rospy.Publisher('/joint_angles', JointAnglesWithSpeed)
    pub_joints.publish(msg_joints)



def nod():
    client = actionlib.SimpleActionClient("/joint_trajectory", JointTrajectoryAction)
    
    goal = JointTrajectoryGoal()
    
    # move head: single joint, multiple keypoints
    goal.trajectory.joint_names = ["HeadPitch"]
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(0.5), positions = [0.0]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(1), positions = [0.5]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(1.5), positions = [0.0]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(2), positions = [0.5]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(2.5), positions = [0.0]))
    
    
    rospy.loginfo("Sending goal...")
    client.send_goal(goal)
    #client.wait_for_result()
    #result = client.get_result()
    #rospy.loginfo("Results: %s", str(result.goal_position.position))



def inc():
    client = actionlib.SimpleActionClient("/joint_trajectory", JointTrajectoryAction)
    
    goal = JointTrajectoryGoal()
    
    goal.trajectory.joint_names = ["HeadPitch","HeadYaw","LAnklePitch","LAnkleRoll","LElbowRoll","LElbowYaw","LHand","LHipPitch","LHipRoll","LHipYawPitch","LKneePitch","LShoulderPitch","LShoulderRoll","LWristYaw","RAnklePitch","RAnkleRoll","RElbowRoll","RElbowYaw","RHand","RHipPitch","RHipRoll","RHipYawPitch","RKneePitch","RShoulderPitch","RShoulderRoll","RWristYaw"]
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(0.96), positions = [-0.15651, -0.01845, 0.09046, -0.13035, -0.38806, -1.18582, 0.00, 0.13043, 0.09975, -0.17023, -0.09055, 1.53856, 0.12728, 0.07359, 0.09055, 0.13043, 0.38201, 1.17347, 0.00, 0.12881, -0.11041, -0.17023, -0.08893, 1.54785, -0.11509, 0.13495]))#0.96

    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(1.32), positions = [-0.15651, -0.01845, 0.09046, -0.13035, -0.34204, -1.19043, 0.0, 0.13043, 0.09975, -0.17023, -0.09055, 1.60299, 0.10887, -0.14577, 0.09055, 0.13043, 0.26082, 1.18574, 0.00, 0.12881, -0.11041, -0.17023, -0.08893, 1.58160, -0.12736, 0.62890]))#1.32

    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(1.8), positions = [0.08893, -0.95419, 0.09046, -0.13035, -0.56294, -1.30394, 0.0, 0.13043, 0.09975, -0.17023, -0.09055, 1.03694, 0.07512, -1.55859, 0.09055, 0.13043, 0.94038, 1.61833, 0.00, 0.12881, -0.11041, -0.17023, -0.08893, 1.20423, 0.05825, 1.62446]))#1.8

    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(2.24), positions = [0.09200, 1.07376, 0.09046, -0.13035, -0.63503, -1.30394, 0.8, 0.13043, 0.09975, -0.17023, -0.09055, 0.90962, 0.04905, -1.50336, 0.09055, 0.13043, 0.96033, 1.62293, 0.8, 0.12881, -0.11041, -0.17023, -0.08893, 1.09532, 0.17790, 1.59839]))#2.24

    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(2.68), positions = [0.09200, 1.07376, 0.09046, -0.13035, -0.63657, -2.08567, 0.8, 0.13043, 0.09975, -0.17023, -0.09055, 0.58748, 0.38039, -1.82387, 0.09055, 0.13043, 0.96339, 2.08567, 0.8, 0.12881, -0.11041, -0.17023, -0.08893, 0.71335, -0.36513, 1.82387]))#2.68

    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(3.32), positions = [0.09200, -0.59523, 0.09046, -0.13035, -0.63657, -2.08567, 0.8, 0.13043, 0.09975, -0.17023, -0.09055, 0.39573, 0.58441, -1.82387, 0.09055, 0.13043, 0.91277, 2.08567, 0.8, 0.12881, -0.11041, -0.17023, -0.08893, 0.55075, -0.47405, 1.82387]))#3.32

    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(3.92), positions = [0.09046, 0.15489, 0.09046, -0.13035, -0.63197, -2.02186, 0.8, 0.13043, 0.09975, -0.17023, -0.09055, 0.86360, -0.07061, -1.73193, 0.09055, 0.13043, 0.86675, 2.03097, 0.8, 0.12881, -0.11041, -0.17023, -0.08893, 0.93885, 0.23159, 1.72417]))#3.92
    
    rospy.loginfo("Sending goal...")
    client.send_goal(goal)
    #client.wait_for_result() #???
    #result = client.get_result() #???
    #rospy.loginfo("Results: %s", str(result.goal_position.position))#???



# main loop
def main():
    
    rospy.init_node('head_controller', anonymous=True)
    r = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        inc()
        r.sleep()

    # s = rospy.Service('head_action', Action, act)
    #print "Ready to control head movement."
    rospy.spin()

if __name__ == '__main__':
  main()
