import math

def get_rotation_diff(a, b):
	'''returns relative angle difference from b to a'''
	return math.atan2(math.sin(a-b), math.cos(a-b))

def frange(start, end, step):
	'''float version of xrange'''
	f = start
	while f < end:
		yield f
		f += step
