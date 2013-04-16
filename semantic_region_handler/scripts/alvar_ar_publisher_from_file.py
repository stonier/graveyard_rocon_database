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
from ar_track_alvar.msg import AlvarMarkers, AlvarMarker

def insert_marker(marker):
    req = AddSemanticRegionRequest()
    req.name = marker['name']

    req.pose_stamped.pose = Pose() 
    req.pose_stamped.header.frame_id = marker['frame_id']
    req.region.marker.header.frame_id = marker['frame_id']
    req.region.marker.id = marker['id']
    req.region.marker.confidence = marker['confidence']
    req.region.marker.pose.header.frame_id = marker['frame_id']
    req.region.marker.pose.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',marker['pose'])
        
    return req

def publish(markers):
    ar_list = AlvarMarkers()
    
    for m in markers:
        ar = AlvarMarker()
        ar.id = m['id']
        ar.confidence = m['confidence']
        ar.pose.header.frame_id = m['frame_id']
        ar.pose.header.stamp = rospy.Time.now()
        ar.pose.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',m['pose'])
        ar_list.markers.append(ar)
    
    ar_pub.publish(ar_list)
    
    return
    


if __name__ == '__main__':
    global ar_pub
    rospy.init_node('ar_loader')
    filename = rospy.get_param('~filename')
    srv_name = 'add_ar_marker'
    ar_pub = rospy.Publisher('global_ar_list',AlvarMarkers,latch=True)

    rl = RegionLoader(insert_marker,srv_name,filename,publish,True)
    rospy.loginfo('Initialized')
    rl.spin()
    rospy.spin()

    rospy.loginfo('Bye Bye')

