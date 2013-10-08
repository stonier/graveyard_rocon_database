#!/usr/bin/env python

import rospy
import actionlib
import yaml
import tf
import copy

from worldlib.msg import *
from world_msgs.msg import *
from geometry_msgs.msg import *
from semantic_region_handler.srv import *
from semantic_region_handler.msg import *
from rospy_message_converter import json_message_converter, message_converter
from semantic_region_handler import RegionLoader
from visualization_msgs.msg import *
from yocs_msgs.msg import *

def insert_wall(wall):
    req = AddSemanticRegionRequest()
    req.name = wall['name']

    req.pose_stamped.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',wall['pose'])
    req.pose_stamped.header.frame_id = wall['frame_id']
    req.region.radius = float(wall['radius'])

    return req

def publish(wall):
    wall_list = WallList()
    # Markers
    marker_list = MarkerArray()    

    marker_id = 1
    for t in wall:
        tp = Wall()
        tp.name = t['name']
        tp.length = float(t['length'])
        tp.width  = float(t['width'])
        tp.height = float(t['height'])
        tp.pose.header.frame_id = t['frame_id']
        tp.pose.header.stamp = rospy.Time.now()
        tp.pose.pose.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',t['pose'])
        wall_list.obstacles.append(tp)

        marker = Marker()
        marker.id = marker_id
        marker.header = tp.pose.header
        marker.type = Marker.CUBE
        marker.ns = "wall_obstacles"
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration.from_sec(0)
        marker.pose = copy.deepcopy(tp.pose.pose.pose)
        marker.pose.position.z += tp.height/2.0
        marker.scale.x = tp.width
        marker.scale.y = tp.length
        marker.scale.z = tp.height
        marker.color.r = 0.2
        marker.color.g = 0.4
        marker.color.b = 0.4
        marker.color.a = 0.5
                                                                                                                                                    
        marker_list.markers.append(marker)
                                                                                                                                                    
        marker_id = marker_id + 1

    wall_pub.publish(wall_list)
    marker_pub.publish(marker_list)
    
    return

if __name__ == '__main__':
    global wall_pub
    rospy.init_node('wall_loader')
    filename = rospy.get_param('~filename')
    srv_name = 'add_wall_region'
    
    marker_pub = rospy.Publisher('wall_marker',MarkerArray,latch=True)
    wall_pub = rospy.Publisher('wall_pose_list',WallList,latch=True)
    tf_pub = tf.TransformBroadcaster()
    
    rl = RegionLoader(insert_wall,srv_name,filename,publish,True)
    rospy.loginfo('Initialized')
    rl.spin()
    rospy.spin()
    rospy.loginfo('Bye Bye')

