#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int64
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

from exploration.srv import *

import threading, math, os

# an unstable looking hack to fix broken python path things,
# so that the following imports work. sorry.
sys.path.insert(1, os.path.join(sys.path[0], 'simulation/code'))

import simulation
from util import *


# TODO:
# - too many waypoints wtf!!!1111 ✓
# - config ✓
# - publish ✓
# - git submodule ✓
# - safety padding
# - exploration simulation uses absolute rotation. what about this one? where's 0?
# - metric units?
# - where is (0, 0)? which is the axis orientation?

def handle_get_waypoint(request):
	'''GetWaypoint service handler'''
	#print 'SERVICE!!!!!!'
	index = request.index
	if index >= len(SimWorker.path):
		print 'index too large: %i length is: %i' % (index, len(SimWorker.path))
		return None		# TODO: ?
	else:
		print 'returning waypoint with index %i: (%.2f, %.2f)' % (index, SimWorker.path[index].pose.position.x, SimWorker.path[index].pose.position.y)
		return SimWorker.path[index]

def quaternion_from_yaw(yaw):
	q = quaternion_from_euler(0, 0, yaw)
	return  Quaternion(*q)

class SimWorker(threading.Thread):

	path = []

	def __init__(self):
		super(SimWorker, self).__init__()

	def run(self):
		# declare publisher for "finished" announcement
		pub = rospy.Publisher('exploration_path_complete', Int64, queue_size=10)
		# lets's go
		simulation.init()
		old_len = 0
		while simulation.world.robot.busy:
			simulation.tick()
			# convert new waypoints
			if len(simulation.world.robot.position_history) > old_len:
				for i in xrange(old_len, len(simulation.world.robot.position_history)):
					self.append_position(i)
				old_len = len(simulation.world.robot.position_history)
		# publish final waypoint count
		rospy.loginfo('finished computation of exploration path')
		pub.publish(len(SimWorker.path))

	def append_position(self, index):
		'''converts an exploration simulation waypoint to a ROS waypoint and stores it'''
		(px, py, _), beta = simulation.world.robot.position_history[index]
		if index == 0:
			rx, ry, _ = simulation.world.robot.position
			rot = simulation.world.robot.rotation
		else:
			rx, ry, _ = simulation.world.robot.position_history[index-1][0]
			rot = simulation.world.robot.position_history[index-1][1]

		alpha = math.atan2(py-ry, px-rx)

		# first rotate towards target position
		if abs(rotation_diff(rot, alpha)) > 0.04:	# allow slight arcish flight
			p1 = PoseStamped()
			p1.pose.position.x = rx
			p1.pose.position.y = ry
			p1.pose.orientation = quaternion_from_yaw(alpha)
			SimWorker.path.append(p1)
			#print 'appended position (%.2f, %.2f)' % (rx, ry)

		# fly to
		p2 = PoseStamped()
		p2.pose.position.x = px
		p2.pose.position.y = py
		p2.pose.orientation = quaternion_from_yaw(alpha)
		SimWorker.path.append(p2)
		#print 'appended position (%.2f, %.2f)' % (px, py)

		# possible secondary rotation when on target point
		if abs(rotation_diff(alpha, beta)) > 0.01:
			p3 = PoseStamped()
			p3.pose.position.x = px
			p3.pose.position.y = py
			p3.pose.orientation = quaternion_from_yaw(beta)
			SimWorker.path.append(p3)
			#print 'appended position (%.2f, %.2f)' % (px, py)


def main():
	rospy.init_node('exploration')

	# CONFIG
	# robot
	x, y = rospy.get_param('~start_position', [2.5, 2.5])
	z = rospy.get_param('~flight_height', 1.199)
	simulation.configure('start_position', [x, y, z])
	rot = rospy.get_param('~start_orientation', 0.)
	simulation.configure('start_rotation', rot)
	# world
	w, h = rospy.get_param('~world_size', [10., 10.])
	simulation.configure('world_width', w)
	simulation.configure('world_height', h)
	vsize = rospy.get_param('~voxel_size', .1)
	simulation.configure('voxel_size', vsize)

	# spawn simulation thread
	rospy.loginfo('starting computation of exploration path')
	SimWorker().start()
	# register service
	rospy.loginfo('registering waypoint service')
	s = rospy.Service('get_waypoint', GetWaypoint, handle_get_waypoint)
	# don't leave
	rospy.spin()

if __name__ == '__main__':
	main()
