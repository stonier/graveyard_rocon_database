#!/usr/bin/env python
import rospy
import actionlib

from worldlib.msg import *
from world_msgs.msg import *
from geometry_msgs.msg import *
from semantic_region_handler.msg import *
from rospy_message_converter import json_message_converter

class RegionPoller(object):
    def __init__(self,spatial_world_model_ns,concert_name,instance_tags,description_tags,descriptor_ref,parser,publisher):
        self.spatial_world_model_ns = spatial_world_model_ns
        self.concert_name = concert_name
        self.instance_tags = instance_tags
        self.description_tags = description_tags
        self.descriptor_ref =descriptor_ref

        rospy.loginfo('Connecting to World Model handler...')
        # Setting actions to spaital world model handler
        self._action = {}
        self._action['woits'] = actionlib.SimpleActionClient(self.spatial_world_model_ns + '/world_object_instance_tag_search',WorldObjectInstanceTagSearchAction)
        self._action['gwod'] = actionlib.SimpleActionClient(self.spatial_world_model_ns + '/get_world_object_description',GetWorldObjectDescriptionAction)
#        self._action['wodts'] = actionlib.SimpleActionClient(self.spatial_world_model_ns + '/world_object_description_tag_search',WorldObjectDescriptionTagSearchAction)
        self._action['woits'].wait_for_server()
        self._action['gwod'].wait_for_server()
#        self._action['wodts'].wait_for_server()
        rospy.loginfo('World Model Handler Ready')

        self.publisher = publisher
        self.parser = parser

        self.descriptions = {}

    def get_region(self,description_id):
        description = self.descriptions[description_id]
        message = json_message_converter.convert_json_to_ros_message(description.descriptors[0].type, description.descriptors[0].data)
        
        return message


    def get_world_object_description(self,description_id):
        self._action['gwod'].send_goal_and_wait(GetWorldObjectDescriptionGoal(description_id))
        resp = self._action['gwod'].get_result()

        if not resp.exists:
            rospy.logwarn("Descriptions with id [%s] does not exist"%description_id)

        return resp.description
        """
        self._action['wodts'].send_goal_and_wait(WorldObjectInstanceTagSearchGoal(self.instance_tags))
        resp = self._action['wodts'].get_result()

        descriptions = [ d for d in resp.descriptions if d.description_id == description_id]

        if len(descriptions) > 1:
            rospy.logwarn("There is more than one descriptions with id [%s]. Selecting the first one"%description_id)
        description = descriptions[0]

        return description
        """

    def get_region_instances(self):
        self._action['woits'].send_goal_and_wait(WorldObjectInstanceTagSearchGoal(self.instance_tags))
        resp = self._action['woits'].get_result()                                                               

        return resp.instances

    def update_region_descriptions(self,instances):
        for i in instances:
            if not (i.description_id in self.descriptions): 
                self.descriptions[i.description_id] = self.get_world_object_description(i.description_id)


    def spin(self):
        while not rospy.is_shutdown():
            # Get Instances from database
            instances = self.get_region_instances()
            self.update_region_descriptions(instances)

            lists = self.parser(instances,self)

            self.publisher(lists)

            rospy.sleep(1)


