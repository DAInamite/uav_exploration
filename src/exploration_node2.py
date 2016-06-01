#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int64
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf

# occupancy grid service
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid

from exploration.srv import *

import threading, math, os

import sampling_based_decidor


# TODO:
# - make slam_to_odom_coords configurable

tf_listener = None
grid_service = None
position = None
rotation = None

def handle_get_current_waypoint(request):
	'''GetWaypoint service handler'''
	if position == None or rotation == None or grid_service == None:
		# called before TF data is available? nope.
		return None
	# get world grid
	grid = grid_service().map
	# decide
	decidor = sampling_based_decidor.Decidor(grid, position, rotation)
	result = decidor.decide_waypoint()
	if result:
		wp, util = result
		print 'returning waypoint %s with util %.2f' % (wp, util)
		return slam_to_odom_coords(get_pose(wp))
	else:
		print 'result = None'
		return None

def quaternion_from_yaw(yaw):
	q = quaternion_from_euler(0, 0, yaw)
	return Quaternion(*q)

def get_pose(planar_coords):
	'''given planar coordinates, returns a PoseStamped object'''
	x, y = planar_coords
	alpha = math.atan2(y-position[1], x-position[0])
	pose = PoseStamped()
	pose.pose.position.x = x
	pose.pose.position.y = y
	pose.pose.orientation = quaternion_from_yaw(alpha)
	return pose

def slam_to_odom_coords(pose):
	'''converts a slam world pose to a odometry world pose'''
	# whatever the values are, define that they come from SLAM
	pose.header.frame_id = '/ORB_base_link'
	tf_listener.waitForTransform('/ORB_base_link', '/base_link', pose.header.stamp, rospy.Duration(0.))
	return tf_listener.transformPose('/base_link', pose)

def main():
	print "starting up..."
	rospy.init_node('exploration')

	# get occupancy grid service handler
	global grid_service
	print "waiting for occupancy grid service"
	rospy.wait_for_service('/orb_slam/occupancy_grid')
	grid_service = rospy.ServiceProxy('/orb_slam/occupancy_grid', GetMap)

	# register service
	print 'registering waypoint service'
	s = rospy.Service('get_current_waypoint', GetCurrentWaypoint, handle_get_current_waypoint)

	global tf_listener
	tf_listener = tf.TransformListener()
	rate = rospy.Rate(10.)
	global position, rotation
	got_tf = False
	print 'entering TF getter loop'
	# TODO: waitForTransform()
	while not rospy.is_shutdown():
		try:
			position, rot = tf_listener.lookupTransform('/ORB_SLAM/World', '/ORB_SLAM/Camera', rospy.Time(0))
			#position, rot = tf_listener.lookupTransform('world', 'robot/videocamera', rospy.Time(0))
			rotation = euler_from_quaternion(rot)[2]
			if not got_tf:
				print 'got TF data'
				got_tf = True
		except (tf.LookupException, tf.ExtrapolationException, tf.ConnectivityException):
			pass
		rate.sleep()


if __name__ == '__main__':
	main()
