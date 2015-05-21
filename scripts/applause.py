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



def applause():
    client = actionlib.SimpleActionClient("/joint_trajectory", JointTrajectoryAction)
    
    goal = JointTrajectoryGoal()
    
    # move head: single joint, multiple keypoints
    goal.trajectory.joint_names = ["HeadPitch","HeadYaw","LAnklePitch","LAnkleRoll","LElbowRoll","LElbowYaw","LHand","LHipPitch","LHipRoll","LHipYawPitch","LKneePitch","LShoulderPitch","LShoulderRoll","LWristYaw","RAnklePitch","RAnkleRoll","RElbowRoll","RElbowYaw","RHand","RHipPitch","RHipRoll","RHipYawPitch","RKneePitch","RShoulderPitch","RShoulderRoll","RWristYaw"]
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(0.40000), positions = [-0.17645,-0.00618,0.09046,-0.13035,-0.42181,-1.18276,0.00,0.13043,0.10129,-0.17023,-0.09055,1.47873,0.09660,0.07973,0.09055,0.12890,0.41422,1.18574,0.00000,0.13188,-0.09967,-0.17023,-0.08740,1.46808,-0.11049,0.07512]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(1.20000), positions = [-0.17645,-0.00618,0.09046,-0.13035,-0.90348,-1.45121,1.0,0.13043,0.10129,-0.17023,-0.09055,0.45095,0.02757,-0.06294,0.09055,0.12890,1.38985,1.48794,1.0,0.13188,-0.09967,-0.17023,-0.08740,0.72716,-0.30224,0.04751]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(1.60000), positions = [-0.17645,-0.00618,0.09046,-0.13035,-1.22716,-1.10912,1.0,0.13043,0.10129,-0.17023,-0.09055,0.39420,-0.31416,0.01990,0.09055,0.12890,1.54462,1.12898,1.0,0.13188,-0.09967,-0.17023,-0.08740,0.63512,0.31416,-0.13197]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(2.00000), positions = [-0.17645,-0.00618,0.09046,-0.13035,-0.90348,-1.45121,1.0,0.13043,0.10129,-0.17023,-0.09055,0.45095,0.02757,-0.06294,0.09055,0.12890,1.38985,1.48794,1.0,0.13188,-0.09967,-0.17023,-0.08740,0.72716,-0.30224,0.04751]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(2.40000), positions = [-0.17645,-0.00618,0.09046,-0.13035,-1.22716,-1.10912,1.0,0.13043,0.10129,-0.17023,-0.09055,0.39420,-0.31416,0.01990,0.09055,0.12890,1.54462,1.12898,1.00,0.13188,-0.09967,-0.17023,-0.08740,0.63512,0.31416,-0.13197]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(2.80000), positions = [-0.17645,-0.00618,0.09046,-0.13035,-0.90348,-1.45121,1.0,0.13043,0.10129,-0.17023,-0.09055,0.45095,0.02757,-0.06294,0.09055,0.12890,1.38985,1.48794,1.0,0.13188,-0.09967,-0.17023,-0.08740,0.72716,-0.30224,0.04751]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(3.20000), positions = [-0.17645,-0.00618,0.09046,-0.13035,-1.22716,-1.10912,1.0,0.13043,0.10129,-0.17023,-0.09055,0.39420,-0.31416,0.01990,0.09055,0.12890,1.54462,1.12898,1.0,0.13188,-0.09967,-0.17023,-0.08740,0.63512,0.31416,-0.13197]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(3.60000), positions = [-0.17645,-0.00618,0.09046,-0.13035,-0.90348,-1.45121,1.0,0.13043,0.10129,-0.17023,-0.09055,0.45095,0.02757,-0.06294,0.09055,0.12890,1.38985,1.48794,1.0,0.13188,-0.09967,-0.17023,-0.08740,0.72716,-0.30224,0.04751]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(4.00000), positions = [-0.17645,-0.00618,0.09046,-0.13035,-1.22716,-1.10912,1.0,0.13043,0.10129,-0.17023,-0.09055,0.39420,-0.31416,0.01990,0.09055,0.12890,1.54462,1.12898,1.0,0.13188,-0.09967,-0.17023,-0.08740,0.63512,0.31416,-0.13197]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(4.80000), positions = [-0.17645,-0.00618,0.09046,-0.13035,-0.42181,-1.18276,0.00,0.13043,0.10129,-0.17023,-0.09055,1.47873,0.09660,0.07973,0.09055,0.12890,0.41422,1.18574,0.00,0.13188,-0.09967,-0.17023,-0.08740,1.46808,-0.11049,0.07512]))


    
    
    rospy.loginfo("Sending goal...")
    client.send_goal(goal)
    #client.wait_for_result() #???
    #result = client.get_result() #???
    #rospy.loginfo("Results: %s", str(result.goal_position.position))#???



# main loop
def main():
    
    rospy.init_node('head_controller', anonymous=True)
    r = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        applause()
        r.sleep()

    # s = rospy.Service('head_action', Action, act)
    #print "Ready to control head movement."
    rospy.spin()

if __name__ == '__main__':
  main()
