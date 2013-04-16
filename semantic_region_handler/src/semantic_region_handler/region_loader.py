#!/usr/bin/env python

import rospy
import actionlib
import yaml

from worldlib.msg import *
from world_msgs.msg import *
from geometry_msgs.msg import *
from semantic_region_handler.srv import *
from rospy_message_converter import json_message_converter, message_converter

class RegionLoader(object):
    def __init__(self,insert_func,srv_name,filename,publish,no_store):
        self.filename = filename
        self.insert = insert_func
        self.publisher = publish
        self.no_store = no_store

        self.add_srv = rospy.ServiceProxy(srv_name,AddSemanticRegion)

    def load_file(self,filename):
        yaml_data = None 
        with open(filename) as f:
            yaml_data = yaml.load(f)

        return yaml_data

    def spin(self):
        data= self.load_file(self.filename)
        self.publisher(data)

        if not self.no_store:
            for d in data:
                req = self.insert(d)
                resp = self.add_srv(req)
                rospy.loginfo("%s is inserted as %s"%(req.name,resp.instance_id))
