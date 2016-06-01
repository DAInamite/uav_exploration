import math
from util import *

# fuck
#import simulation
WORLD_WIDTH = 40
WORLD_HEIGHT = 30

OVERLAPPING = .075

# Status enums
TRANSLATE_RIGHT = 0
ROTATE_RIGHT1 = 1
TRANSLATE_RIGHT_DOWN = 2
ROTATE_RIGHT2 = 3
TRANSLATE_LEFT = 4
ROTATE_LEFT1 = 5
TRANSLATE_LEFT_DOWN = 6
ROTATE_LEFT2 = 7

class Decidor(object):
	def __init__(self, world):
		self.world = world
		self.row_width = world.robot.get_vision_width() * (1. - OVERLAPPING)
		rx, ry, rz = world.robot.position
		rx = -world.robot.get_vision_range()
		ry = self.row_width / 2.
		print world.robot.get_vision_range()
		world.robot.position = [rx, ry, rz]
		world.robot.rotation = 0.
		world.robot.move_order()	# stop!
		# exploration status
		self.status = ROTATE_LEFT2

	def decide(self):
		if not self.world.robot.is_moving():
			rx, ry, rz = self.world.robot.position
			if self.status == ROTATE_LEFT2:
				self.status = TRANSLATE_RIGHT
				self.world.robot.move_order((WORLD_WIDTH - (self.row_width)/2., ry, rz))
			elif self.status == TRANSLATE_RIGHT:
				if self.world.robot.position[1] >= WORLD_HEIGHT - self.row_width / 2.:
					self.world.robot.busy = False
					return
				self.status = ROTATE_RIGHT1
				self.world.robot.move_order(rotation=math.pi/2.)
			elif self.status == ROTATE_RIGHT1:
				self.status = TRANSLATE_RIGHT_DOWN
				self.world.robot.move_order((rx, ry + self.row_width, rz))
			elif self.status == TRANSLATE_RIGHT_DOWN:
				self.status = ROTATE_RIGHT2
				self.world.robot.move_order(rotation=math.pi)
			elif self.status == ROTATE_RIGHT2:
				self.status = TRANSLATE_LEFT
				self.world.robot.move_order(((self.row_width)/2., ry, rz))
			elif self.status == TRANSLATE_LEFT:
				if self.world.robot.position[1] >= WORLD_HEIGHT - self.row_width / 2.:
					self.world.robot.busy = False
					return
				self.status = ROTATE_LEFT1
				self.world.robot.move_order(rotation=math.pi/2.)
			elif self.status == ROTATE_LEFT1:
				self.status = TRANSLATE_LEFT_DOWN
				self.world.robot.move_order((rx, ry + self.row_width, rz))
			elif self.status == TRANSLATE_LEFT_DOWN:
				self.status = ROTATE_LEFT2
				self.world.robot.move_order(rotation=0.)
