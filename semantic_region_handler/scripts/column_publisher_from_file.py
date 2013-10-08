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

def insert_column(column):
    req = AddSemanticRegionRequest()
    req.name = column['name']

    req.pose_stamped.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',column['pose'])
    req.pose_stamped.header.frame_id = column['frame_id']
    req.region.radius = float(column['radius'])

    return req

def publish(column):
    column_list = ColumnList()
    # Markers
    marker_list = MarkerArray()    

    marker_id = 1
    for t in column:
        tp = Column()
        tp.name = t['name']
        tp.radius = float(t['radius'])
        tp.height = float(t['height'])
        tp.pose.header.frame_id = t['frame_id']
        tp.pose.header.stamp = rospy.Time.now()
        tp.pose.pose.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',t['pose'])
        column_list.obstacles.append(tp)
        
        marker = Marker()
        marker.id = marker_id
        marker.header = tp.pose.header
        marker.type = Marker.CYLINDER
        marker.ns = "column_obstacles"
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration.from_sec(0)
        marker.pose = copy.deepcopy(tp.pose.pose.pose)
        marker.pose.position.z += tp.height/2.0
        marker.scale.x = tp.radius * 2
        marker.scale.y = tp.radius * 2  
        marker.scale.z = tp.height
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1.0
        marker.color.a = 0.5
                                                                                                                                                    
        marker_list.markers.append(marker)
                                                                                                                                                    
        marker_id = marker_id + 1

    column_pub.publish(column_list)
    marker_pub.publish(marker_list)
    
    return

if __name__ == '__main__':
    global column_pub
    rospy.init_node('column_loader')
    filename = rospy.get_param('~filename')
    srv_name = 'add_column_region'
    
    marker_pub = rospy.Publisher('column_marker',MarkerArray,latch=True)
    column_pub = rospy.Publisher('column_pose_list',ColumnList,latch=True)
    tf_pub = tf.TransformBroadcaster()
    
    rl = RegionLoader(insert_column,srv_name,filename,publish,True)
    rospy.loginfo('Initialized')
    rl.spin()
    rospy.spin()
    rospy.loginfo('Bye Bye')

