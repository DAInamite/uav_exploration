#!/usr/bin/env python

import sys
import rospy
from exploration.srv import *

def pose_to_string(pose):
    return '(%s, %s, %s)' % (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)

def get_waypoint_client(index):
    rospy.wait_for_service('get_waypoint')
    try:
        get_waypoint = rospy.ServiceProxy('get_waypoint', GetWaypoint)
        resp1 = get_waypoint(index)
        return resp1.waypoint
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e

def usage():
    return '%s [index]' % sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        index = int(sys.argv[1])
    else:
        print usage()
        sys.exit(1)
    print 'Requesting waypoint %s' % index
    print 'waypoint %s:\n%s' % (index, pose_to_string(get_waypoint_client(index)))
