#!/usr/bin/env python

import sys
import rospy
from exploration.srv import *

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion

import actionlib
import path_follower.msg


def move_order(pose):
    '''initiates a copter move action (for testing purposes)'''
    # Creates the SimpleActionClient, passing the type of the action to the constructor.
    client = actionlib.SimpleActionClient('/follow_path', path_follower.msg.FollowPathAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = path_follower.msg.FollowPathGoal()
    goal.path.header.stamp = rospy.Time.now()
    goal.path.header.frame_id = 'odom'
    goal.path.poses = [pose]

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    print client.get_state(), client.get_goal_status_text()

    # Prints out the result of executing the action
    return client.get_result()  # final pose

def pose_to_string(pose):
    o = pose.pose.orientation
    q = o.x, o.y, o.z, o.w
    return '(%s, %s, %s) @ %s' % (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, euler_from_quaternion(q)[2])

def get_waypoint_client():
    rospy.wait_for_service('get_current_waypoint')
    try:
        get_waypoint = rospy.ServiceProxy('get_current_waypoint', GetCurrentWaypoint)
        resp1 = get_waypoint()
        return resp1.waypoint
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e

if __name__ == "__main__":
    rospy.init_node('exploration_service_test')
    while True:
        print 'Requesting waypoint...'
        pose = get_waypoint_client()
        print 'Waypoint:\n%s' % pose_to_string(pose)
        move_order(pose)
        print 'arrived'
