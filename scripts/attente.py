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
JointNames = ["HeadPitch","HeadYaw","LAnklePitch","LAnkleRoll","LElbowRoll","LElbowYaw","LHand","LHipPitch","LHipRoll","LHipYawPitch","LKneePitch","LShoulderPitch","LShoulderRoll","LWristYaw","RAnkleRoll","RElbowRoll","RElbowYaw","RHand","RHipPitch","RHipRoll","RHipYawPitch","RKneePitch","RKneePitch","RShoulderPitch","RShoulderRoll","RWristYaw"]

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
    elif req.command=="attente":
        attente()
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



def attente():
    client = actionlib.SimpleActionClient("/joint_trajectory", JointTrajectoryAction)
    
    goal = JointTrajectoryGoal()
    
    # move head: single joint, multiple keypoints
    goal.trajectory.joint_names = ["HeadPitch","HeadYaw","LAnklePitch","LAnkleRoll","LElbowRoll","LElbowYaw","LHand","LHipPitch","LHipRoll","LHipYawPitch","LKneePitch","LShoulderPitch","LShoulderRoll","LWristYaw","RAnklePitch","RAnkleRoll","RElbowRoll","RElbowYaw","RHand","RHipPitch","RHipRoll","RHipYawPitch","RKneePitch","RShoulderPitch","RShoulderRoll","RWristYaw"]
#    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(0.5), positions = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(0.4), positions = [-0.16418,-0.01845,0.09660,-0.13035,-0.38806,-1.16742,0.00510,0.13043,0.10589,-0.16870,-0.08901,1.53856,0.11654,0.06899,0.09055,0.13043,0.37587,1.17500,0.00539,0.12881,-0.11501,-0.16870,-0.09046,1.56012,-0.11816, 0.1303]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(0.8), positions = [0.15182,-0.05066,0.12575,-0.09200,-1.25170,0.13188,0.01480,0.00464,0.05987,-0.01990,-0.09233,1.28545,0.41107,-0.49706,0.09055,0.15498,1.08458,0.01683,0.01463,0.02297,-0.17023,-0.01990,-0.09233,1.42359,-0.50780,0.20705]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(1.48), positions = [-0.32218,-0.94652, 0.14415,-0.09200,-1.20722,0.04905,0.01470,-0.05978,0.05987,-0.02910,-0.09233, 1.17500,0.43101,-0.47558,0.11663,0.15498,1.01708,0.05518,0.01455,-0.03379,-0.17023, -0.02910,-0.09233,1.38678,-0.49552, 0.18404]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(2.36), positions = [-0.32372,0.85440,0.14415,-0.09200,-0.03491,0.02143,0.01470,-0.05978,0.05987,-0.02910,-0.09233,1.67202,0.01837,-0.64739,0.11663,0.15498,1.54462,0.16716,0.01455,-0.03379,-0.17023,-0.02910,-0.09233,1.47728,-0.47405,0.37425]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(3.28), positions = [0.40800,0.47703,0.14415,-0.09200,-1.49868,-0.37434,0.01470,-0.05978,0.05987,-0.02910,-0.09233,0.93877,0.32210,1.60452,0.11663,0.15498,1.54462,0.17177,0.01455,-0.03379,-0.17023,-0.02910,-0.09233,1.47882,-0.47405,0.37425]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(3.6), positions = [0.40800,0.47703,0.14415,-0.09200,-1.49868,-0.37434,0.01470,-0.05978,0.05987,-0.02910,-0.09233,0.93877,0.32210,1.60452,0.11663,0.15498,1.54462,0.17177,0.01455,-0.03379,-0.17023,-0.02910,-0.09233,1.47882,-0.47405,0.37425]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(4.48), positions = [-0.33752,-0.10742,0.14415,-0.09200,-1.28698,0.04751,0.01470,-0.05978,0.05987,-0.02910,-0.09233,1.32687,0.43868,-0.47251,0.11663,0.15498,1.37451,-0.39275,0.01455,-0.03379,-0.17023,-0.02910,-0.09233,1.07384,-0.44950,0.44635]))
	
	
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
        attente()
        r.sleep()

    # s = rospy.Service('head_action', Action, act)
    #print "Ready to control head movement."
    rospy.spin()

if __name__ == '__main__':
  main()
