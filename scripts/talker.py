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
JointNames = [ 'HeadYaw', 'HeadPitch']

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



def shake():
    client = actionlib.SimpleActionClient("/joint_trajectory", JointTrajectoryAction)
    
    goal = JointTrajectoryGoal()
    
    # move head: single joint, multiple keypoints
    goal.trajectory.joint_names = ["HeadPitch","HeadYaw","LElbowRoll","LElbowYaw","LHand","LShoulderPitch","LShoulderRoll","LWristYaw","RElbowRoll","RElbowYaw","RHand","RShoulderPitch","RShoulderRoll","RWristYaw"]
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(0.64000), positions = ['','','',,,,,,,,,,,]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(0.72000), positions = ['','',-1.37902,,,,,,,,,,,]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(0.80000), positions = [0.29602,-0.13503,'',,,,,,,,,,,]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(1.40000), positions = ['','','',,,,,,,,,,,]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(1.48000), positions = ['','',-1.29005,'',,,,,,,,,,]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(1.56000), positions = [-0.17032,-0.35133,'',,,,,,,,,,,]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(1.68000), positions = ['','','',,,,,,,,,,,]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(2.08000), positions = ['','','',,,,,,,,,,,]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(2.16000), positions = ['','',-1.18267,,,,,,,,,,,]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(2.24000), positions = [-0.34059,-0.41576,'',,,,,,,,,,,]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(2.40000), positions = ['','','',,,,,,,,,,,]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(2.64000), positions = ['','','',,,,,,,,,,,]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(2.72000), positions = ['','',-1.24863,,,,,,,,,,,]))
	goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(2.80000), positions = [-0.05987,-0.41882,'',,,,,,,,,,,]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(3.04000), positions = ['','','',,,,,,,,,,,]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(3.32000), positions = ['','','',,,,,,,,,,,]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(3.40000), positions = ['','',-1.31920,,,,,,,,,,,]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(3.48000), positions = [-0.19333,-0.52007,'',,,,,,,,,,,]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(3.72000), positions = ['','','',,,,,,,,,,,]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(4.44000), positions = ['','','',,,,,,,,,,,]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(4.52000), positions = ['','',-1.18421,,,,,,,,,,,]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(4.60000), positions = [-0.01078,-0.37587,'',,,,,,,,,,,]))
    
    
    rospy.loginfo("Sending goal...")
    client.send_goal(goal)
    #client.wait_for_result() #???
    #result = client.get_result() #???
    #rospy.loginfo("Results: %s", str(result.goal_position.position))#???



# main loop
def main():
    
    rospy.init_node('head_controller', anonymous=True)
    r = rospy.Rate(0.4)
    while not rospy.is_shutdown():
        shake()
        r.sleep()

    # s = rospy.Service('head_action', Action, act)
    #print "Ready to control head movement."
    rospy.spin()

if __name__ == '__main__':
  main()
