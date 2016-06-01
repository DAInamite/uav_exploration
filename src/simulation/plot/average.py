#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys

valuess = []

for filename in sys.argv[1:]:
	with open(filename, 'r') as f:
		values = []
		skipping = True
		for line in f:
			if not skipping:
				values.append(float(line))
			if line[:3] == '+++':
				skipping = False
		valuess.append(values)

max_len = max(map(len, valuess))
max_value = max(map(lambda values: values[-1], valuess))
for values in valuess:
	if len(values) < max_len:
		values += [max_value] * (max_len - len(values))

avg_values = []
for i in xrange(max_len):
	cur_value = 0.
	for values in valuess:
		cur_value += values[i]
	avg_values.append(cur_value / len(valuess))

print '\n'.join(map(str, avg_values))
