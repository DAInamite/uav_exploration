#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import sys

if '--latex' in sys.argv:
	sys.argv.remove('--latex')
	output_latex = True
else:
	output_latex = False

times = []
distances = []
rotations = []
objects1 = []
objects2 = []
objects3 = []

for filename in sys.argv[1:]:
	with open(filename, 'r') as f:
		while '+++ STATS +++' not in f.readline():
			pass
		times.append(float(f.readline().split()[2]))
		distances.append(float(f.readline().split()[2]))
		rotations.append(float(f.readline().split()[2][:-2]))
		o1, _, o2, _, o3 = f.readline().split()[3:8]
		objects1.append(float(o1))
		objects2.append(float(o2))
		objects3.append(float(o3))

if output_latex:
	print '\tbenötigte Zeit & \SI{%.2f}{\second} \\\\ \hline' % (sum(times) / len(times))
	print '\tEntfernung & \SI{%.2f}{\meter} \\\\ \hline' % (sum(distances) / len(distances))
	print '\tRotation & \SI{%.2f}{\degree} \\\\ \hline' % (sum(rotations) / len(rotations))
	print '\tObjekte gefunden & \SI{%.2f}{\second}, \SI{%.2f}{\second}, \SI{%.2f}{\second} \\\\ \hline' % (sum(objects1) / len(objects1), sum(objects2) / len(objects3), sum(objects3) / len(objects3))
else:
	print 'Time needed: %.2f s, Min: %.2f s, Max: %.2f s' % (sum(times) / len(times), min(times), max(times))
	print 'Distance travelled: %.2f m, Min: %.2f m, Max: %.2f m' % (sum(distances) / len(distances), min(distances), max(distances))
	print 'Total rotation: %.2f°, Min: %.2f°, Max: %.2f°' % (sum(rotations) / len(rotations), min(rotations), max(rotations))
	print '1st object found after: %.2f s, Min: %.2f s, Max: %.2f s' % (sum(objects1) / len(objects1), min(objects1), max(objects1))
	print '2nd object found after: %.2f s, Min: %.2f s, Max: %.2f s' % (sum(objects2) / len(objects2), min(objects2), max(objects2))
	print '3rd object found after: %.2f s, Min: %.2f s, Max: %.2f s' % (sum(objects3) / len(objects3), min(objects3), max(objects3))
