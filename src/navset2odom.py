#!/usr/bin/env python3
# - *- coding: utf- 8 - *-
import math
import os
import time
import pymap3d as pm
import rospy
import tf
import pyproj
import numpy as np

import rospy
from mavros_msgs.msg import GPSRAW
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point


# origin point
longitude = 55.747349899999996
latitude = 48.743282699999995
z0 = 217


inProj = pyproj.Proj(init='epsg:4326')
outProj = pyproj.Proj(init='epsg:32639')

x0, y0 = pyproj.transform(inProj, outProj, latitude, longitude)

print(x0, y0, z0)

x, y, z = 0, 0, 0

local_pose = Point()
navset_msg = NavSatFix()
global_pose = NavSatFix()
odom_msg = Odometry()

quat_lidar = tf.transformations.quaternion_from_euler(0, math.radians(93), 0)
px4_rpy = []
lidar_rpy = []

# def gpsraw_clb(data):
#     global navset_msg, pub_fix, x, y, z

#     navset_msg.header.frame_id = "base_link"
#     navset_msg.header.stamp = data.header.stamp
#     navset_msg.latitude = data.lat * 0.0000001
#     navset_msg.longitude = data.lon * 0.0000001
#     navset_msg.altitude = data.alt * 0.001

#     _x, _y = pyproj.transform(inProj, outProj, navset_msg.longitude, navset_msg.latitude)
    
#     x = _x - x0
#     y = _y - y0
#     z = navset_msg.altitude - z0
#     # print("origin", x0, y0, z0)
#     print("origin epsg:32639", x0, y0, z0)

#     pub_fix.publish(navset_msg)

# def imu_px_clb(data):
#     global x, y, z, br, quat_lidar, px4_rpy
    
#     px4_rpy = tf.transformations.euler_from_quaternion((data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))

#     print("px4_rpy:", np.degrees(px4_rpy))

#     br.sendTransform((x, y, z),
#                          (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w),     
#                          data.header.stamp,
#                          "base_link",
#                           "map")

#     br.sendTransform((-0.02035, 0, -0.19),
#                         (0.,0.,0.,1.),
#                          data.header.stamp,
#                          "livox_link",
#                          "base_link")

#     br.sendTransform((0.0, 0, 0.0469),
#                     quat_lidar,
#                         data.header.stamp,
#                         "livox_frame",
#                         "livox_link")

# def imu_lidar_clb(data):
#     global x, y, z, br, quat_lidar, px4_rpy, lidar_rpy
    
#     lidar_rpy = tf.transformations.euler_from_quaternion((data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
#     print("px4_rpy:", np.degrees(px4_rpy))
#     print("lidar_rpy:", np.degrees(lidar_rpy))
#     diff = [lidar_rpy[0]-px4_rpy[0],lidar_rpy[1]-px4_rpy[1],lidar_rpy[2]-px4_rpy[2]]
#     print("diff:",  np.degrees(diff))
    

def navsat_clb(data):
    global global_pose, local_pose, x, y, z, pub_odom, odom_msg
    global_pose = data
    odom_msg.header = data.header
    odom_msg.header.frame_id = "map"
    
    _x, _y = pyproj.transform(inProj, outProj, global_pose.longitude, global_pose.latitude)

    x = _x - x0
    y = _y - y0
    z = data.altitude - z0

    odom_msg.pose.pose.position.x = x
    odom_msg.pose.pose.position.y = y
    odom_msg.pose.pose.position.z = z
    odom_msg.pose.covariance[0] = data.position_covariance[0]
    odom_msg.pose.covariance[4] = data.position_covariance[4]
    odom_msg.pose.covariance[8] = data.position_covariance[8]
    
    
    print("origin WGS84", latitude, longitude, z0)
    print("origin epsg:32639", x0, y0, z0)
    pub_odom.publish(odom_msg)
 
if __name__ == '__main__':

    rospy.init_node("gpsraw2navsat_node", anonymous=True)
    br = tf.TransformBroadcaster()
    
    # rospy.Subscriber("/mavros/gpsstatus/gps2/raw", GPSRAW, gpsraw_clb)
    rospy.Subscriber("/gnss", NavSatFix, navsat_clb)
    

    pub_odom = rospy.Publisher("/odometry/gps", Odometry, queue_size=10)
    print("init node")

    rospy.spin()
