#!/usr/bin/env python2
# -*- coding: utf-8 -*-

CELLS = 300*400
OPT_CELLS_PER_SEC = 62.7

import glob, re, os, sys, random, math
import matplotlib.pyplot as plt
import numpy as np

TICKS_PER_SECOND = 2.


COLORS = ['b', 'g', 'r', 'c', 'm', 'y', 'darkblue', 'w']
FILENAMES = ['benchmark/frontier_util_nopen_*.txt', 'benchmark/frontier_util_penalty_*.txt', 'benchmark/concentric_*.txt', 'benchmark/random_*.txt', 'benchmark/sampling_*.txt', 'benchmark/srt_*.txt', 'benchmark/concentric-free.txt']
LABELS = ['Frontier-Utility', 'Frontier-Utility (wenig Rotation)', 'konzentrische Kreise', u'Zufälliger Flug', 'Sampling-basiert', 'SRT', 'konzentrische Kreise (begrenzungslos)']
SHORT_LABELS = ['Frontier', 'Frontier rot.pen', 'Kreise', u'Zufällig', 'Sampling', 'SRT', 'Kreise (frei)']

#LEGEND_ORDER = [4, 0, 1, 2, 3, 5]		# 14000 s
#LEGEND_ORDER = [4, 0, 3, 1, 2, 5]		# 600 s
#LEGEND_ORDER = [4, 6, 2]				# concentric_free
LEGEND_ORDER = [3, 2, 5, 0, 1, 4]		# bars

#FILENAMES = ['../code/benchmark/sampling_2010*.txt', '../code/benchmark/sampling_3010*.txt', '../code/benchmark/sampling_2015*.txt']
#LEGEND = ['2010', '3010', '2015']
#FILENAMES = ['../code/benchmark/test_4fps*.txt', '../code/benchmark/test_5fps*.txt']
#LEGEND = ['4', '5']

re_overall_time = re.compile(r"Time needed: (\d+\.\d+) s")
re_distance = re.compile(r"Distance travelled: (\d+\.\d+) m")
re_rotation = re.compile(r"Total rotation: (\d+\.\d+)°")
re_object_times = re.compile(r"Objects found after: (\d+\.\d+) s, (\d+\.\d+) s, (\d+.\d+) s")


class Benchmark(object):
	def __init__(self, filename):
		super(Benchmark, self).__init__()
		self.identifier = os.path.basename(filename)
		with open(filename, 'r') as f:
			self.raw = f.read()

	def get_cells_progress(self):
		skipping = True
		cells = []
		for line in self.raw.splitlines():
			if not skipping:
				if '+++' in line:
					break
				cells.append(int(line))
			if '+++ following progress ticks +++' in line:
				skipping = False
		return cells

	def get_overall_time(self):
		match = next(re_overall_time.finditer(self.raw))
		return float(match.group(1))

	def get_distance(self):
		match = next(re_distance.finditer(self.raw))
		return float(match.group(1))

	def get_rotation(self):
		match = next(re_rotation.finditer(self.raw))
		return float(match.group(1))

	def get_object_times(self):
		match = next(re_object_times.finditer(self.raw))
		return float(match.group(1)), float(match.group(2)), float(match.group(3))


def get_variance(values):
	'''returns the variance of a list of values'''
	avg = float(sum(values)) / len(values)
	print avg
	result = 0.
	for value in values:
		print value, avg, (value - avg)**2, len(values), (value - avg)**2 / len(values)
		result += (value - avg)**2 / len(values)
	return result

def get_average_list(lists):
	'''gets a list of lists and computes an average list of them.'''
	avg = []
	all_end = False		# indicates wheather all lines already ended
	x = 0
	while not all_end:
		all_end = True
		value = 0
		for y in xrange(len(lists)):
			if len(lists[y]) > x:
				all_end = False
				value += lists[y][x]
			else:
				value += lists[y][-1]
		if not all_end:
			avg.append(float(value) / len(lists))
		x += 1
	return avg

def get_rate_from_progress(progress):
	'''gets a progress list with a cell count at each index and returns a list of cell rates'''
	width = 30	# 60 ticks = 30 seconds
	prefix = []
	for i in xrange(1, width):
		prefix.append(float(progress[i] - progress[0]) / (i/TICKS_PER_SECOND))
	return prefix + [float(progress[i] - progress[i-width]) / (width/TICKS_PER_SECOND) for i in xrange(width, len(progress))]
	#return [progress[i] - progress[i-1] if i > 0 else progress[0] for i in xrange(len(progress))]

def smooth_values(values, width, iterations):
	'''smoothes a list of values'''
	width = int(width)
	new_values = list(values)
	for _ in xrange(iterations):
		for i in xrange(len(values)):
			value_sum = 0
			weight_sum = 0
			for j in xrange(max(i - width/2, 0), min(i + width/2, len(values))):
				weight = float(width/2-abs(i - j))/(width/2)
				weight_sum += weight
				if j < 0:
					value = values[0]
				elif j >= len(values):
					value = values[-1]
				else:
					value = values[j]
				value_sum += weight * value
			new_values[i] = value_sum / weight_sum
		# swap to avoid garbage collection and avoid re-creating the list
		temp = values
		values = new_values
		new_values = temp
	return values

def log(text):
	sys.stdout.write(text)
	sys.stdout.flush()

benchmarkz = []

log('reading benchmark logs: ')
for filestring in FILENAMES:
	log(os.path.basename(filestring) + ': ')
	benchmarks = []
	for filename in glob.glob(filestring):
		benchmarks.append(Benchmark(filename))
	benchmarkz.append(benchmarks)
	log('%i ' % len(benchmarks))

#plt.figure(figsize=(11,6))
#plt.rc('text', usetex=True)

# # plot averaged progress
# log('\nAveraging and plotting... ')
# legend = []
# # optimal line
# seconds = [x for x in xrange(int(CELLS/OPT_CELLS_PER_SEC))]
# plt.plot(seconds, [s * OPT_CELLS_PER_SEC / CELLS * 100 for s in seconds], '--', color='0.5', linewidth=2)
# legend.append('Idealzeit')
# # benchmarks
# for progress, i in enumerate(LEGEND_ORDER):
# 	benchmarks = benchmarkz[i]
# 	legend.append(LABELS[i])
# 	cellz = map(lambda b: b.get_cells_progress(), benchmarks)
# 	avg_cells = map(lambda c: float(c) / CELLS * 100, get_average_list(cellz))
# 	plt.plot([x/TICKS_PER_SECOND for x in xrange(len(avg_cells))], avg_cells, color=COLORS[i], linewidth=2)
# 	log('%i%% ' % (100. * (progress+1) / len(LEGEND_ORDER)))
# plt.xlabel('Zeit in s')
# plt.ylabel('Entdecktes Gebiet in %')
# ax = plt.gca()
# ax.set_xlim([0, 6000]) # 600 14000
# ax.set_ylim([0, 100]) # 50
# plt.legend(legend, 4) # 2


# # plot averaged exploration rate
# log('\nAveraging and plotting... ')
# legend = []
# # optimal line
# seconds = [x for x in xrange(int(CELLS/OPT_CELLS_PER_SEC))]
# plt.plot(seconds, [OPT_CELLS_PER_SEC for s in seconds], '--', color='0.5', linewidth=2)
# legend.append('Idealzeit')
# # benchmarks
# for progress, i in enumerate(LEGEND_ORDER):
# 	benchmarks = benchmarkz[i]
# 	legend.append(LABELS[i])
# 	cellz = map(lambda b: b.get_cells_progress(), benchmarks)
# 	avg_cells = get_average_list(cellz)
# 	avg_rates = get_rate_from_progress(avg_cells)
# 	#print sum(avg_rates) / len(avg_rates)
# 	avg_rates = smooth_values(avg_rates, 120, 8)
# 	plt.plot([x/TICKS_PER_SECOND for x in xrange(len(avg_rates))], avg_rates, color=COLORS[i], linewidth=2)
# 	log('%i%% ' % (100. * (progress+1) / len(LEGEND_ORDER)))
# plt.xlabel('Zeit in s')
# plt.ylabel('Explorationsrate in Zellen pro s')
# ax = plt.gca()
# ax.set_xlim([0, 6000])
# ax.set_ylim([0, 100])
# plt.legend(legend, 1)


# # plot object find probabilities
# log('\nSmoothing and plotting... ')
# bin_width = 10
# legend = []
# for progress, i in enumerate(LEGEND_ORDER):
# 	benchmarks = benchmarkz[i]
# 	legend.append(LABELS[i])
# 	times = []
# 	for benchmark in benchmarks:
# 		times += benchmark.get_object_times()
# 	print len(times)
# 	objects_found = [0.] * (int(max(times) / bin_width) + 1)
# 	for time in times:
# 		objects_found[int(time / bin_width)] += 100. / bin_width / len(benchmarks)
# 	objects_found = smooth_values(objects_found, 25, 8)
# 	seconds = [x * bin_width for x in xrange(len(objects_found))]
# 	plt.plot(seconds, objects_found, color=COLORS[i], linewidth=2)
#  	log('%i%% ' % (100. * (progress+1) / len(LEGEND_ORDER)))
# plt.xlabel('Zeit in s')
# plt.ylabel('Wahrscheinlichkeit Objekt zu finden in %')
# ax = plt.gca()
# ax.set_xlim([0, 12000])
# plt.legend(legend, 1)


# # plot bars
# log('\nAveraging... ')
# legend = []
# colors = []
# # benchmarks
# avgs = []
# stds = []
# mins = []
# maxes = []
# for progress, i in enumerate(LEGEND_ORDER):
# 	benchmarks = benchmarkz[i]
# 	legend.append(LABELS[i])
# 	colors.append(COLORS[i])
# 	values = map(lambda b: b.get_overall_time(), benchmarks)
# 	#values = map(lambda b: b.get_distance(), benchmarks)
# 	#values = map(lambda b: b.get_rotation(), benchmarks)
# 	avgs.append(sum(values) / len(values))
# 	stds.append(math.sqrt(get_variance(values)))
# 	mins.append(min(values))
# 	maxes.append(max(values))
# 	log('%i%% ' % (100. * (progress+1) / len(LEGEND_ORDER)))
# log('Plotting...')
# lefts = [x+.2 for x in xrange(len(LEGEND_ORDER))]
# bars = plt.bar(lefts, avgs, yerr=stds, color=colors)
# for i, (mi, ma) in enumerate(zip(mins, maxes)):
# 	plt.plot([i+.2, i+1], [mi]*2, '--', color='gray')
# 	plt.plot([i+.2, i+1], [ma]*2, '--', color='gray')
# ax = plt.gca()
# ax.set_xlim([0, len(LEGEND_ORDER)+.2])
# plt.ylabel(u'Benötigte Zeit in s')
# ax.set_ylim([0, 20000])	# time
# #plt.ylabel(u'Zurückgelegte Strecke in m')
# #ax.set_ylim([0, 4000])		# distance
# #plt.ylabel(u'Rotationssumme in °')
# #ax.set_ylim([0, 60000])		# distance
# ax.set_xticks([])
# #figlegend = plt.figure(figsize=(3.9,1.875))
# #figlegend.legend(bars, legend)
# #figlegend.show()


# plot bars in groups of three
log('\nAveraging... ')
legend = []
colors = []
# benchmarks
avgs = []
stds = []
mins = []
maxes = []
lefts = []
x = .2
for progress, i in enumerate(LEGEND_ORDER):
	benchmarks = benchmarkz[i]
	valuez = map(lambda b: b.get_object_times(), benchmarks)
	valuez = np.asarray(valuez).T.tolist()
	for j in xrange(3):
		lefts.append(x)
		x += 0.9
		colors.append(COLORS[i])
		values = valuez[j]
		avgs.append(sum(values) / len(values))
		stds.append(math.sqrt(get_variance(values)))
		mins.append(min(values))
		maxes.append(max(values))
	x += .2
	log('%i%% ' % (100. * (progress+1) / len(LEGEND_ORDER)))
log('Plotting...')
plt.bar(lefts, avgs, yerr=stds, color=colors)
for i, (mi, ma) in enumerate(zip(mins, maxes)):
	plt.plot([lefts[i], lefts[i]+.8], [mi]*2, '--', color='gray')
	plt.plot([lefts[i], lefts[i]+.8], [ma]*2, '--', color='gray')
ax = plt.gca()
ax.set_xlim([0, x])
plt.ylabel(u'Fundzeiten der Objekte in s')
ax.set_ylim([0, 14000])
ax.set_xticks([])


# # plot object times histogram
# f, axes = plt.subplots(len(benchmarkz), sharex=True)
# bins = range(0, 14000, 300) #range(0, 14000, 250)
# for i, (benchmarks, ax) in enumerate(zip(benchmarkz, axes)):
# 	times = []
# 	for benchmark in benchmarks:
# 		times += benchmark.get_object_times()
# 	print max(times)
# 	ax.hist(times, bins=bins)
# 	ax.set_title(LEGEND[i])
# 	ax.set_ylim([0, 50])
# plt.xlabel('Zeit in s')
# plt.ylabel('Entdeckte Objekte')


log('\n')
plt.show()
