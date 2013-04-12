#!/usr/bin/env python

import rospy
import json

from semantic_region_handler import *
from ar_track_alvar.msg import *

def marker_comparison(a,b):
    return a.marker.id == b.marker.id

if __name__ == '__main__':
    rospy.init_node('alvar_ar_handler')

    spatial_world_model_ns = '/spatial_world_model'
    concert_name = "concert"
    instance_tags = [concert_name,'landmarks']
    description_tags = [concert_name,'table','landmarks']
    descriptor_ref = json.dumps({'type':'landmarks'}) 

    srv_name = {}
    srv_name['add'] = 'add_ar_marker'
    srv_name['get'] = 'get_ar_marker'
    srv_name['remove'] = 'remove_ar_marker'

    sph = SemanticRegionHandler(spatial_world_model_ns,concert_name,instance_tags,description_tags,descriptor_ref,marker_comparison,srv_name)
    rospy.loginfo('Initialized')
    sph.spin()
    rospy.loginfo('Bye Bye')
