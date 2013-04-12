#!/usr/bin/env python

import rospy
import actionlib
import yaml

from worldlib.msg import *
from world_msgs.msg import *
from geometry_msgs.msg import *
from semantic_region_handler.srv import *
from rospy_message_converter import json_message_converter, message_converter
from semantic_region_handler import RegionLoader

def insert_marker(marker):
    req = AddSemanticRegionRequest()
    req.name = marker['name']

    req.pose_stamped.pose = Pose() 
    req.pose_stamped.header.frame_id = marker['frame_id']
    req.region.marker.header.frame_id = marker['frame_id']
    req.region.marker.id = marker['id']
    req.region.marker.confidence = marker['confidence']
    req.region.marker.pose.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',marker['pose'])
        
    return req


if __name__ == '__main__':
    rospy.init_node('ar_loader')
    filename = rospy.get_param('~filename')
    srv_name = 'add_ar_marker'
    rl = RegionLoader(insert_marker,srv_name,filename)
    rospy.loginfo('Initialized')
    rl.spin()
    rospy.loginfo('Bye Bye')

