#!/usr/bin/env python

import rospy
import actionlib
import yaml

from worldlib.msg import *
from world_msgs.msg import *
from geometry_msgs.msg import *
from semantic_region_handler.srv import *
from semantic_region_handler.msg import *
from rospy_message_converter import json_message_converter, message_converter
from semantic_region_handler import RegionLoader

def insert_table(table):
    req = AddSemanticRegionRequest()
    req.name = table['name']

    req.pose_stamped.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',table['pose'])
    req.pose_stamped.header.frame_id = table['frame_id']
    req.region.radius = float(table['radius'])

    return req

def publish(table):
    table_list = TablePoseList()
    
    for t in table:
        tp = TablePose()
        tp.name = t['name']
        tp.pose_cov_stamped.header.frame_id = t['frame_id']
        tp.radius = float(t['radius'])
        tp.pose_cov_stamped.pose.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',t['pose'])
        table_list.tables.append(tp)
        
    table_pub.publish(table_list)
    
    return

if __name__ == '__main__':
    global table_pub
    rospy.init_node('table_loader')
    filename = rospy.get_param('~filename')
    srv_name = 'add_table_region'
    
    table_pub = rospy.Publisher('table_pose_list',TablePoseList,latch=True)
    
    rl = RegionLoader(insert_table,srv_name,filename,publish,True)
    rospy.loginfo('Initialized')
    rl.spin()
    rospy.spin()
    rospy.loginfo('Bye Bye')

