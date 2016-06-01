#!/usr/bin/env python2

import sys, re, math
import numpy as np
from matplotlib.patches import Circle
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt


def get_rotation_diff(a, b):
	'''returns relative angle difference from b to a'''
	return math.atan2(math.sin(a-b), math.cos(a-b))


MAX_CIRCLE_RADIUS = .4
regex_entry = re.compile(r"\(\[(\d+\.\d+),\s+(\d+\.\d+)\],\s+(\d+.\d+)\)")

entries = []

#with file('16-03-01/frontier_util_path3.txt', 'r') as f:
with file('../code/benchmark/frontier_util_4fps_1.txt', 'r') as f:
	while True:
		if '+++ position history +++' in f.readline():
			break
	history = f.readline()
	for match in regex_entry.finditer(history):
		entries.append([float(match.group(1)), float(match.group(2)), float(match.group(3))])

# numpyify
entries = np.array(entries)

fig, ax = plt.subplots()

plt.plot(entries[:,0], entries[:,1], color='b', zorder=1)

circles = []
px, py, pr = entries[0] # 'previous' values
for x, y, r in entries[1:]:
	if px == x and py == y:
		circles.append(Circle((x, y), get_rotation_diff(pr, r)/math.pi*MAX_CIRCLE_RADIUS))
	px, py = x, y
p = PatchCollection(circles, facecolors='b', edgecolors='black', zorder=2)
ax.add_collection(p)

plt.ylim(0, 30)
plt.xlim(0, 40)
plt.show()
