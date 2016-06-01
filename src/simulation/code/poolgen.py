#!/usr/bin/env python
# -*- coding: utf-8 -*-

import random, math

PADDING = 2.		# metric for robot
W, H = 40., 30.

GW, GH = 400, 300	# grid cells for objects
GPADDING = 20


with open('scenario-pool.txt', 'w') as f:
	for i in xrange(100):
		# robot config
		rx = random.uniform(PADDING, W - PADDING)
		ry = random.uniform(PADDING, H - PADDING)
		rot = random.uniform(0., math.pi*2)
		f.write('%.2f %.2f %.4f\n' % (rx, ry, rot))
		# hidden objects
		objects = []
		for j in xrange(3):
			while True:
				voxel = random.randrange(GPADDING, GW - GPADDING), random.randrange(GPADDING, GH - GPADDING)
				if voxel not in objects:	# distinct positions
					objects.append(voxel)
					break
		o = ' '.join(['%i %i' % (o[0], o[1]) for o in objects])
		f.write(o + '\n')
