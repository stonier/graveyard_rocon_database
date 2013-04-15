#!/usr/bin/env python
import rospy
import json

from semantic_region_handler import RegionPoller
from semantic_region_handler.msg import *
from visualization_msgs.msg import *

def parse(instances,region_poller):
    # TablePostList preparation
    table_pose_list = TablePoseList()
    # Markers
    marker_list = MarkerArray()

    marker_id = 1
    for i in instances:
        table = TablePose()
        table.name = i.name
        table.pose_cov_stamped = i.pose
        region = region_poller.get_region(i.description_id)
        table.radius = region.radius
        table_pose_list.tables.append(table)

        marker = Marker()
        marker.id = marker_id
        marker.header = i.pose.header
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE
        marker.ns = region_poller.concert_name
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration.from_sec(1)
        marker.pose = i.pose.pose.pose
        marker.scale.x = table.radius
        marker.scale.y = table.radius
        marker.scale.z = 0.1
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker_list.markers.append(marker)

        marker_id = marker_id + 1

    return [marker_list, table_pose_list]

def publisher(lists):
    global marker_pub
    global table_pub

    marker_pub.publish(lists[0])
    table_pub.publish(lists[1])


if __name__ == '__main__':
    global marker_pub
    global table_pub

    rospy.init_node('polling_table')
    spatial_world_model_ns = 'spatial_world_model'
    concert_name = "concert"
    instance_tags = [concert_name,'table']
    description_tags = [concert_name,'table','radius']
    descriptor_ref = json.dumps({'type':'semantic_circle'}) 

    marker_pub = rospy.Publisher('table_marker',MarkerArray,latch=True)
    table_pub = rospy.Publisher('table_pose_list',TablePoseList,latch=True)

    sph = RegionPoller(spatial_world_model_ns,concert_name,instance_tags,description_tags,descriptor_ref,parse,publisher)
    rospy.loginfo('Initialized')
    sph.spin()
    rospy.loginfo('Bye Bye')

