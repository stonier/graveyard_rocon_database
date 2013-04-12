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

def insert_table(table):
    req = AddSemanticRegionRequest()
    req.name = table['name']

    req.pose_stamped.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',table['pose'])
    req.pose_stamped.header.frame_id = table['frame_id']
    req.region.radius = float(table['radius'])

    return req

if __name__ == '__main__':
    rospy.init_node('table_loader')
    filename = rospy.get_param('~filename')
    srv_name = 'add_table_region'
    rl = RegionLoader(insert_table,srv_name,filename)
    rospy.loginfo('Initialized')
    rl.spin()
    rospy.loginfo('Bye Bye')

