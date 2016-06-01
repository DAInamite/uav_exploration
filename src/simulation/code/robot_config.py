'''ROBOT AND CAMERA SETUP'''

import math

class RobotConfig(object):
	'''class to globally hold robot specific simulation config'''
	def __init__(self):
		self.CAMRAYS_H = 160				# 640 x 480 @30 FPS
		self.CAMRAYS_V = 120
		self.CAM_FPS = 8
		self.CAMRAY_LENGTH = 3.5			# meters
		self.FIELD_OF_VIEW_H = 58 / 360. * math.pi * 2
		self.FIELD_OF_VIEW_V = 45 / 360. * math.pi * 2
		self.CAMERA_ROTATION = -45 / 360. * math.pi * 2 # vertical camera adjustment
		#self.ROTATION_SPEED = math.pi / 2	# radians per second
		#self.TRANSLATION_SPEED = 2			# meters per second
		self.ROTATION_SPEED = 5. / 360 * math.pi * 2
		self.TRANSLATION_SPEED = .2			# meters per second
		self.BOT_RADIUS = .25				# meters
