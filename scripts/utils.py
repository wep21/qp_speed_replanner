#!/usr/bin/env python
import numpy as np
import math

def area(wa, wb, wc):
    a = wa.pose.pose.position
    b = wb.pose.pose.position
    c = wc.pose.pose.position
    return abs((b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x))
    
    
def dist(wa, wb):
    a = wa.pose.pose.position
    b = wb.pose.pose.position
    dx = a.x - b.x
    dy = a.y - b.y
    return (dx**2 + dy**2)**0.5


def calc_curvature(wa, wb, wc):
    try:
        return 2 * area(wa, wb, wc)/(dist(wa, wb) * dist(wb, wc) * dist(wc, wa))
    except ZeroDivisionError:
        print('cannot calcurate curvature')


def convert_euler_to_monotonic(yaw_arr):
    l = len(yaw_arr)
    for i in range(1, l):
        if yaw_arr[i] - yaw_arr[i-1] > math.pi:
            yaw_arr[i:l] = yaw_arr[i:l] - 2.0 * math.pi
        elif yaw_arr[i] - yaw_arr[i-1] < -math.pi:
            yaw_arr[i:l] = yaw_arr[i:l] + 2.0 * math.pi
    return yaw_arr


def calc_waypoints_curvature(lane):
    wp_size = len(lane.waypoints)
    curvature = np.zeros((wp_size, 1))
    for i in range(1, wp_size-1):
        curvature[i] = calc_curvature(lane.waypoints[i-1],
                                      lane.waypoints[i], lane.waypoints[i+1])
    curvature[0] = curvature[1]
    curvature[wp_size-1] = curvature[wp_size-2]
    return curvature
