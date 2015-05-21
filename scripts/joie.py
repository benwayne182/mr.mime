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



def joie():
    client = actionlib.SimpleActionClient("/joint_trajectory", JointTrajectoryAction)
    
    goal = JointTrajectoryGoal()
    
    goal.trajectory.joint_names = ["HeadPitch","HeadYaw","LAnklePitch","LAnkleRoll","LElbowRoll","LElbowYaw","LHand","LHipPitch","LHipRoll","LHipYawPitch","LKneePitch","LShoulderPitch","LShoulderRoll","LWristYaw","RAnklePitch","RAnkleRoll","RElbowRoll","RElbowYaw","RHand","RHipPitch","RHipRoll","RHipYawPitch","RKneePitch","RShoulderPitch","RShoulderRoll","RWristYaw"]
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(0.4), positions = [ -0.18105, -0.01385, 0.09046, -0.13035, -0.41414, -1.19656, 0.00526, 0.13197, 0.09975, -0.17023, -0.09055, 1.49407, 0.12421, 0.07512, 0.09055, 0.13043, 0.41576, 1.19494, 0.00537, 0.13035, -0.09967, -0.17023, -0.09046, 1.47728, -0.12583, 0.13955]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(1.8), positions = [ -0.18105, 0.01385, 0.09046, -0.13035, -0.47703, -1.46655, 0.00527, 0.13197, 0.09975, -0.17023, -0.09055, -0.88669, 0.44022, -0.78698, 0.09055, 0.13043, 0.46331, 1.52629, 0.00537, 0.13035, -0.09967, -0.17023, -0.09046, -0.91269, -0.53541, 0.76082]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(3.08), positions = [ -0.18105, -0.01385, -0.55228, -0.14415, -0.79457, -0.27003, 0.00527, -0.25460, 0.09975, -0.15182, 0.91115, -1.28860, 0.87434, -0.82687, -0.62430, 0.13197, 0.82687, -0.07674, 0.00537, -0.23628, -0.16716, -0.15182, 0.96339, -1.51248, -0.96953, 1.00013]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(4.36), positions = [ -0.18105, -0.01385, 0.09200, -0.13035, -0.03491, -0.30071, 0.01241, 0.13197, 0.09975, -0.17023, -0.09055, -1.48495, 0.11961, -0.82687, 0.09055, 0.13043, 0.04760, -0.07214, 0.01362, 0.13188, -0.09967, -0.17023, -0.09046, -1.49561, -0.14424, 1.00013]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(5.6), positions = [ -0.18105, -0.01385, -0.59217, -0.14262, -0.79457, -0.27003, 0.00527, -0.26534, 0.09975, -0.14876, 0.99399, -1.28860, 0.87434, -0.82687, -0.66571, 0.13043, 0.82687, -0.07674, 0.00537, -0.23781, -0.16870, -0.14876, 1.03856, -1.51248, -0.96953, 1.00013]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(6.92), positions = [ -0.18105, -0.01385, 0.08893, -0.13035, -0.03491, -0.30071, 0.01241, 0.13197, 0.10129, -0.17177, -0.09055, -1.48495, 0.11961, -0.82687, 0.08901, 0.13197, 0.04760, -0.07214, 0.01362, 0.13035, -0.09967, -0.17177, -0.09200, -1.49561, -0.14424, 1.00013]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(8.36), positions = [ -0.18105, -0.01385, 0.09046, -0.13035, -0.41414, -1.19656, 0.00526, 0.13197, 0.09975, -0.17023, -0.09055, 1.49407, 0.12421, 0.07512, 0.09055, 0.13043, 0.41576, 1.19494, 0.00537, 0.13035, -0.09967, -0.17023, -0.09046, 1.47728, -0.12583, 0.13955]))

    
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
        joie()
        r.sleep()

    # s = rospy.Service('head_action', Action, act)
    #print "Ready to control head movement."
    rospy.spin()

if __name__ == '__main__':
  main()
