#!/usr/bin/env python
import rospy
import json

from semantic_region_handler import RegionPoller
from semantic_region_handler.msg import *
from visualization_msgs.msg import *
from ar_track_alvar.msg import *

def parse(instances,region_poller):
    # ar marker List preparation
    ar_marker_array= AlvarMarkers()
    ar_marker_array.header.frame_id = 'map'
    ar_marker_array.header.stamp = rospy.Time.now()

    # Markers
    marker_list = MarkerArray()

    marker_id = 1
    for i in instances:
        region = region_poller.get_region(i.description_id)
        ar_marker = region.marker
        ar_marker_array.markers.append(ar_marker)
        
        marker = Marker()
        marker.id = marker_id
        marker.header = ar_marker.header
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.CUBE
        marker.ns = region_poller.concert_name
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration.from_sec(1)
        marker.pose = ar_marker.pose.pose
        marker.scale.x = 0.1 
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker_list.markers.append(marker)

        marker_id = marker_id + 1

    return [ar_marker_array, marker_list]

def publisher(lists):
    global ar_marker_pub
    global viz_marker_pub 

    ar_marker_pub.publish(lists[0])
    viz_marker_pub.publish(lists[1])


if __name__ == '__main__':
    global ar_marker_pub 
    global viz_marker_pub

    rospy.init_node('alvar_ar_poller')
    spatial_world_model_ns = '/spatial_world_model'
    concert_name = "concert"
    instance_tags = [concert_name,'landmarks']
    description_tags = [concert_name,'table','landmarks']
    descriptor_ref = json.dumps({'type':'landmarks'}) 

    ar_marker_pub = rospy.Publisher('ar_marker_list',AlvarMarkers,latch=True)
    viz_marker_pub = rospy.Publisher('viz_ar_list',MarkerArray,latch=True)

    sph = RegionPoller(spatial_world_model_ns,concert_name,instance_tags,description_tags,descriptor_ref,parse,publisher)
    rospy.loginfo('Initialized')
    sph.spin()
    rospy.loginfo('Bye Bye')
