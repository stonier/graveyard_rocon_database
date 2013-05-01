#!/usr/bin/env python

import rospy
import actionlib
import yaml
import tf

from worldlib.msg import *
from world_msgs.msg import *
from geometry_msgs.msg import *
from semantic_region_handler.srv import *
from semantic_region_handler.msg import *
from rospy_message_converter import json_message_converter, message_converter
from semantic_region_handler import RegionLoader
from visualization_msgs.msg import *

def insert_table(table):
    req = AddSemanticRegionRequest()
    req.name = table['name']

    req.pose_stamped.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',table['pose'])
    req.pose_stamped.header.frame_id = table['frame_id']
    req.region.radius = float(table['radius'])

    return req

def publish(table):
    table_list = TablePoseList()
    # Markers
    marker_list = MarkerArray()    

    marker_id = 1
    for t in table:
        tp = TablePose()
        tp.name = t['name']
        tp.pose_cov_stamped.header.frame_id = t['frame_id']
        tp.radius = float(t['radius'])
        tp.pose_cov_stamped.pose.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',t['pose'])
        table_list.tables.append(tp)
        

        p = tp.pose_cov_stamped.pose.pose.position
        position = (p.x,p.y,p.z)
        o= tp.pose_cov_stamped.pose.pose.orientation
        orientation = (o.x,o.y,o.z,o.w)
        tf_pub.sendTransform(position,orientation,rospy.Time.now(),str(tp.name),'map')
                                                                                                                                                    
        marker = Marker()
        marker.id = marker_id
        marker.header = tp.pose_cov_stamped.header
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.CYLINDER
        marker.ns = "concert"
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration.from_sec(0)
        marker.pose = tp.pose_cov_stamped.pose.pose
        marker.scale.x = tp.radius * 2
        marker.scale.y = tp.radius * 2  
        marker.scale.z = 0.1
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1.0
        marker.color.a = 0.5
                                                                                                                                                    
        marker_list.markers.append(marker)
                                                                                                                                                    
        marker_id = marker_id + 1

    table_pub.publish(table_list)
    marker_pub.publish(marker_list)
    
    return

if __name__ == '__main__':
    global table_pub
    rospy.init_node('table_loader')
    filename = rospy.get_param('~filename')
    srv_name = 'add_table_region'
    
    marker_pub = rospy.Publisher('table_marker',MarkerArray,latch=True)
    table_pub = rospy.Publisher('table_pose_list',TablePoseList,latch=True)
    tf_pub = tf.TransformBroadcaster()
    
    rl = RegionLoader(insert_table,srv_name,filename,publish,True)
    rospy.loginfo('Initialized')
    rl.spin()
    rospy.spin()
    rospy.loginfo('Bye Bye')

