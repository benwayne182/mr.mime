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



def comeAtMe():
    client = actionlib.SimpleActionClient("/joint_trajectory", JointTrajectoryAction)
    
    goal = JointTrajectoryGoal()
    
    # move head: single joint, multiple keypoints
    goal.trajectory.joint_names = ["HeadPitch","HeadYaw","LAnklePitch","LAnkleRoll","LElbowRoll","LElbowYaw","LHand","LHipPitch","LHipRoll","LHipYawPitch","LKneePitch","LShoulderPitch","LShoulderRoll","LWristYaw","RAnklePitch","RAnkleRoll","RElbowRoll","RElbowYaw","RHand","RHipPitch","RHipRoll","RHipYawPitch","RKneePitch","RShoulderPitch","RShoulderRoll","RWristYaw"]
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(0.40000), positions = [-0.21634,-0.02152,0.08740,-0.13342,-0.42488,-1.18429,0.00539,0.13043,0.10129,-0.17177,-0.09233,1.48180,0.09507,0.07512,0.09208,0.13043,0.43110,1.18421,0.00538,0.13188,-0.09967,-0.17177,-0.08740,1.45888,-0.10742,0.13188]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(1.12000), positions = [-0.21634,-0.02152,0.08740,-0.13342,-0.25920,-2.31928,0.01227,0.13043,0.10129,-0.17177,-0.09233,0.08893,-0.19333,-0.01845,0.09208,0.13043,0.42956,1.18421,0.00538,0.13188,-0.09967,-0.17177,-0.08740,1.45734,-0.06140,0.13188]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(1.76000), positions = [-0.21634,-0.02152,0.08740,-0.13342,-1.48334,0.12575,0.01227,0.13043,0.10129,-0.17177,-0.09233,0.08586,0.14876,-1.51257,0.09208,0.13043,0.36974,1.18574,0.00538,0.13188,-0.09967,-0.17177,-0.08740,1.33769,-0.07521,0.13188]))    
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(2.40000), positions = [-0.21634,-0.02152,0.08740,-0.13342,-0.26381,-0.10282,0.01227,0.13043,0.10129,-0.17177,-0.09233,0.31596,0.01376,-1.52791,0.09208,0.13043,0.36974,1.18574,0.00538,0.13188,-0.09967,-0.17177,-0.08740,1.35303,-0.08748,0.13188]))    
#    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(3.08000), positions = [-0.21634,-0.02152,0.08740,-0.13342,-1.54462,-0.28537,0.01227,0.13043,0.10129,-0.17177,-0.09233,0.22852,0.05978,-1.54171,0.09208,0.13043,0.36974,1.18574,0.00538,0.13188,-0.09967,-0.17177,-0.08740,1.36070,-0.08748,0.13188]))  
#    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(3.80000), positions = [-0.21634,-0.02152,0.08740,-0.13342,-0.51538,-0.36513,0.01227,0.13043,0.10129,-0.17177,-0.09233,0.15336,0.08126,-1.54478,0.09208,0.13043,0.36974,1.18574,0.00538,0.13188,-0.09967,-0.17177,-0.08740,1.36837,-0.08748,0.13188]))
#    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(4.32000), positions = [-0.21634,-0.02152,0.08740,-0.13342,-1.54462,-0.14884,0.01227,0.13043,0.10129,-0.17177,-0.09233,0.26381,0.24386,-1.58620,0.09208,0.13043,0.36974,1.18421,0.00538,0.13188,-0.09967,-0.17177,-0.08740,1.38678,-0.08595,0.13188]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(3.08000), positions = [-0.21634,-0.02152,0.08740,-0.13342,-0.25920,-2.31928,0.01227,0.13043,0.10129,-0.17177,-0.09233,0.08893,-0.19333,-0.01845,0.09208,0.13043,0.42956,1.18421,0.00538,0.13188,-0.09967,-0.17177,-0.08740,1.45734,-0.06140,0.13188]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(3.80000), positions = [-0.21634,-0.02152,0.08740,-0.13342,-1.48334,0.12575,0.01227,0.13043,0.10129,-0.17177,-0.09233,0.08586,0.14876,-1.51257,0.09208,0.13043,0.36974,1.18574,0.00538,0.13188,-0.09967,-0.17177,-0.08740,1.33769,-0.07521,0.13188]))    
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(4.8000), positions = [-0.21634,-0.02152,0.08740,-0.13342,-0.42488,-1.18429,0.00539,0.13043,0.10129,-0.17177,-0.09233,1.48180,0.09507,0.07512,0.09208,0.13043,0.43110,1.18421,0.00538,0.13188,-0.09967,-0.17177,-0.08740,1.45888,-0.10742,0.13188]))




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
        comeAtMe()
        r.sleep()

    # s = rospy.Service('head_action', Action, act)
    #print "Ready to control head movement."
    rospy.spin()

if __name__ == '__main__':
  main()
