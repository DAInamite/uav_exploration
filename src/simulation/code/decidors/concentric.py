# -*- coding: utf-8 -*-

import math
from util import *
from abstract_decidor import AbstractDecidor


PLANNING_DISTANCE = 0.25	# a reasonable "epsilon" in meters
OVERLAPPING = .03


class Decidor(AbstractDecidor):
	def __init__(self, drawables_object):
		super(Decidor, self).__init__(drawables_object)
		self.center = None

	def get_current_rotation(self):
		'''returns the current rotation from center pointing to both
		   theoretical vision corners w/o being dependent on actual robot
		   rotation.'''
		# rotation from robot to center
		rx, ry, _ = self.position
		cx, cy = self.center
		rot1 = math.atan2(cy-ry, cx-rx)
		# rotation to face optimally straight
		l = self.get_vision_range()
		w = self.get_vision_width()
		x = math.sqrt(l**2 - (w/2)**2)
		try:
			rot2 = math.acos(x/self.get_current_radius())
		except ValueError:
			print 'WARNING: too close to center for precise computations', x, self.get_current_radius()
			rot2 = math.pi / 2 # math.acos(1)
		offx, offy = x*math.cos(rot1-rot2), x*math.sin(rot1-rot2)
		return math.atan2(ry+offy-cy, rx+offx-cx)

	def get_current_radius(self):
		'''returns the distance of the robot to the center'''
		rx, ry, _ = self.position
		cx, cy = self.center
		return math.sqrt((cx-rx)**2 + (cy-ry)**2)

	def get_vision_corner(self, side=-1, pos = None, rot = None):
		'''returns coords of vision cone corner. -1 is left side, 1 right side.
		   alternative robot pos can be spefified.'''
		if pos:
			rx, ry = pos
		else:
			rx, ry, _ = self.position
		if not rot:
			rot = self.rotation
		l = self.get_vision_range()		# length
		fovh = self.robot_config.FIELD_OF_VIEW_H
		r = rot - side*fovh/2.						# rotation
		offx, offy = l * math.cos(r), l * math.sin(r)
		return rx+offx, ry+offy

	def get_next_waypoint(self, rotation, radius):
#		self.drawables.debug_coords = None

		# basic values setup
		rx, ry, rz = self.position
		cx, cy = self.center
		rotation = rotation % (math.pi*2)
		w = self.get_vision_width()
		vr = self.get_vision_range()
		padding = (w * (1-OVERLAPPING)) / 2.

		# coords of left vision corner
		lcx, lcy = cx + radius * math.cos(rotation), cy + radius * math.sin(rotation)
		alpha = (math.pi - self.robot_config.FIELD_OF_VIEW_H) / 2.
		# robot position and rotation
		rx = lcx + vr * math.cos(rotation - alpha)
		ry = lcy + vr * math.sin(rotation - alpha)
		bot_rotation = rotation + math.pi/2.

		# almost done... lol. following computations for out-of-map situations

		# where robot is out of area
		rleft = rx < padding
		rright = rx >= self.world_width - padding
		rbottom = ry < padding
		rtop = ry > self.world_height - padding

		if rleft or rright or rbottom or rtop:
			# robot is out of area somewhere
			# prioritize (copter flies counter-clock-wise)
			if rleft and rbottom:
				rleft = False
			if rbottom and rright:
				rbottom = False
			if rright and rtop:
				rright = False
			if rtop and rleft:
				rtop = False

			# push back to padding and continue travelling along
			if rleft:
				rx = padding
				ry = self.position[1] - PLANNING_DISTANCE
			if rright:
				rx = self.world_width - padding
				ry = self.position[1] + PLANNING_DISTANCE
			if rbottom:
				ry = padding
				rx = self.position[0] + PLANNING_DISTANCE
			if rtop:
				ry = self.world_height - padding
				rx = self.position[0] - PLANNING_DISTANCE

		# right vision corner depending on (new) robot position
		rot = bot_rotation - self.robot_config.FIELD_OF_VIEW_H / 2.
		rcx, rcy = rx + vr * math.cos(rot), ry + vr * math.sin(rot)

		# where right vision corner is out of area
		rcleft = rcx < 0
		rcright = rcx >= self.world_width
		rcbottom = rcy < 0
		rctop = rcy > self.world_height

		# nested function
		def get_rotation_correction_lc():
			'''returns rotation additionally needed to keep left vision corner
			   on track (touching previous circle) when travelling along a map
			   padding edge.'''
			rot = bot_rotation + self.robot_config.FIELD_OF_VIEW_H / 2.
			lcx, lcy = rx + vr * math.cos(rot), ry + vr * math.sin(rot)
#			self.drawables.debug_coords = (lcx, lcy)
			r = math.sqrt((cx-lcx)**2 + (cy-lcy)**2)
			if r > radius:
				# cosines law: c² = a² + b² - 2ab cos gamma
				#              cos gamma = (a² + b² - c²) / 2ab
				return math.acos(((vr**2)*2 - (r - radius)**2) / (2 * (vr**2)))
				#return math.asin((r - radius) / vr)
			return 0.

		if rcleft:	# right corner is out (on the left side of the map)
			bot_rotation = math.acos(rx / vr) - math.pi/2. + math.pi/2.*3. + self.robot_config.FIELD_OF_VIEW_H/2.
		elif rleft:	# right corner is in already, but robot is out
			bot_rotation = math.pi/2.*3.
			bot_rotation += get_rotation_correction_lc()
		if rcright:
			bot_rotation = math.acos((self.world_width-rx) / vr) - math.pi/2. + math.pi/2. + self.robot_config.FIELD_OF_VIEW_H/2.
		elif rright:
			bot_rotation = math.pi/2.
			bot_rotation += get_rotation_correction_lc()
		if rctop:
			bot_rotation = math.acos((self.world_height-ry) / vr) - math.pi/2. + math.pi + self.robot_config.FIELD_OF_VIEW_H/2.
		elif rtop:
			bot_rotation = math.pi
			bot_rotation += get_rotation_correction_lc()
		if rcbottom:
			bot_rotation = math.acos(ry / vr) - math.pi/2. + self.robot_config.FIELD_OF_VIEW_H/2.
		elif rbottom:
			bot_rotation = 0.
			bot_rotation += get_rotation_correction_lc()

		return (rx, ry), bot_rotation

	def check_center_padding(self):
		'''checks if the center is too close to a border (if already visible)
		   and moves it away if necessary.'''
		padding = (max(self.get_vision_width(), self.get_vision_range()) + self.get_vision_range()/2.) * (1 + OVERLAPPING)
		x, y = self.center
		cx, cy = self.grid.get_cell_index(self.center)
		w, h = self.grid.get_size_metric()
		cw, ch = self.grid.get_size()
		if x < padding:
			x = padding
		if y < padding:
			y = padding
		if x > w - padding:
			x = w-padding
		if y > h - padding:
			y = h-padding
		self.center = x, y
		self.drawables.debug_coords = tuple(self.center)

	def decide(self, grid, position, rotation):
		super(Decidor, self).decide(grid, position, rotation)

		if self.center is None:
			# first call: init
			self.world_width = len(self.grid.matrix[0]) * self.grid.cell_size
			self.world_height = len(self.grid.matrix) * self.grid.cell_size
			self.center = self.get_vision_corner()
			self.check_center_padding()
			cx, cy = self.center
			self.max_radius = max(
				math.sqrt(cx**2+cy**2),
				math.sqrt((self.world_width-cx)**2+cy**2),
				math.sqrt(cx**2+(self.world_height-cy)**2),
				math.sqrt((self.world_width-cx)**2+(self.world_height-cy)**2)
				)
			# exploration status
			self.radius = 0.
			self.rotation = self.get_current_rotation()
			self.old_rotation_diff = 0.
			self.initial_rotation = self.rotation
			self.circle = -1	# inner circle

		if self.radius >= self.max_radius:
			return
		# if not any(self.iter_frontiers()):
		# 	return

		# figure out current rotation
		self.rotation = self.get_current_rotation()
		rot_diff = (self.rotation - self.initial_rotation) % (math.pi*2)
		# finished a circle?
		if rot_diff < self.old_rotation_diff:
			self.circle += 1
		# first circle
		if self.circle == -1:
			self.radius = 0.
		else:
			self.radius = (self.circle + rot_diff/(math.pi*2)) * self.get_vision_width() * (1-OVERLAPPING)
		# MOVE
		c = self.get_current_radius() * 2 * math.pi	# circumference
		# planned rotation
		r = self.rotation + (PLANNING_DISTANCE/c) * (math.pi*2)
		wp = self.get_next_waypoint(r, self.radius)
		self.old_rotation_diff = rot_diff
		return wp
