#!/usr/bin/env python

import rospy
import actionlib
import copy
import json

from worldlib.msg import *
from world_msgs.msg import *
from geometry_msgs.msg import *
from semantic_region_handler.srv import *
from semantic_region_handler.msg import *
from rospy_message_converter import json_message_converter

class SemanticRegionHandler(object):
    def __init__(self,spatial_world_model_ns,concert_name,instance_tags,description_tags,descriptor_ref,data_comparision,srv_name):
        self.data_type = 'semantic_region_handler/Region'
        self.data_comparision = data_comparision
        
        self.spatial_world_model_ns = spatial_world_model_ns
        self.concert_name = concert_name
        self.instance_tags = instance_tags 
        self.description_tags = description_tags 
        self.descriptor_ref = descriptor_ref 

        rospy.loginfo('Connecting to World Model handler...')
        # Setting actions to spaital world model handler
        self._action = {}
        self._action['cwoi'] = actionlib.SimpleActionClient(self.spatial_world_model_ns + '/create_world_object_instance',CreateWorldObjectInstanceAction)
        self._action['cwod'] = actionlib.SimpleActionClient(self.spatial_world_model_ns + '/create_world_object_description',CreateWorldObjectDescriptionAction)
        self._action['rwoi'] = actionlib.SimpleActionClient(self.spatial_world_model_ns + '/remove_world_object_instance',RemoveWorldObjectInstanceAction)
        self._action['uwoi'] = actionlib.SimpleActionClient(self.spatial_world_model_ns + '/update_world_object_instance',UpdateWorldObjectInstanceAction)
        self._action['woits'] = actionlib.SimpleActionClient(self.spatial_world_model_ns + '/world_object_instance_tag_search',WorldObjectInstanceTagSearchAction)
        self._action['wodts'] = actionlib.SimpleActionClient(self.spatial_world_model_ns + '/world_object_description_tag_search',WorldObjectDescriptionTagSearchAction)

        self._action['cwoi'].wait_for_server()
        self._action['cwod'].wait_for_server()
        self._action['rwoi'].wait_for_server()
        self._action['uwoi'].wait_for_server()
        self._action['woits'].wait_for_server()
        self._action['wodts'].wait_for_server()
        rospy.loginfo('World Model Handler Ready')

        self._service = {}
        self._service['add_semantic_region'] = rospy.Service(srv_name['add'],AddSemanticRegion,self.process_add_semantic_region)
        self._service['get_semantic_region'] = rospy.Service(srv_name['get'],GetSemanticRegion,self.process_get_semantic_region)
        self._service['remove_semantic_region'] = rospy.Service(srv_name['remove'],RemoveSemanticRegion,self.process_remove_semantic_region)

    def process_add_semantic_region(self,req):
        # Search database to check whether it exists already
        description_id = self.get_region_description(req.region)

        # if no matched description
        if description_id == None:
            description_id = self.create_region_world_description(req.region)

        instance_id = self.add_region_instance(req.name,req.pose_stamped.header,req.pose_stamped.pose,description_id)
        return AddSemanticRegionResponse("Whoola",instance_id)


    def get_region_description(self,region):
        # Check whether this raiuus descriptor is already in the database
        goal = WorldObjectDescriptionTagSearchGoal(self.description_tags)
        self._action['wodts'].send_goal_and_wait(goal)
        resp = self._action['wodts'].get_result()

        description_id = None
        for d in resp.descriptions: 
            data = json_message_converter.convert_json_to_ros_message(self.data_type,d.descriptors[0].data)
    
            if self.data_comparision(data,region):
                break

        return description_id
        
    def create_region_world_description(self,region):
        region_descriptor = self.create_region_descriptor(region)

        description = WorldObjectDescription()
#description.name = str(radius)
        description.tags = copy.deepcopy(self.description_tags)
        description.tags.append(description.name)
        description.descriptors.append(region_descriptor)

        self._action['cwod'].send_goal_and_wait(CreateWorldObjectDescriptionGoal(description))
        result = self._action['cwod'].get_result()

        return result.description_id

    def create_region_descriptor(self,region):
        d = Descriptor()
        d.type = self.data_type
        d.data = json_message_converter.convert_ros_message_to_json(region)
        d.ref = self.descriptor_ref
        d.tags = self.description_tags

        return d

    def add_region_instance(self,name,header,pose,description_id):
        instance = WorldObjectInstance()
        instance.name = name
        instance.description_id = description_id
        instance.expected_ttl = rospy.Duration(1)
        instance.pose = self.create_pose_cov(header,pose)
        instance.source.origin = self.concert_name
        instance.source.creator = rospy.get_name()
        instance.tags = copy.deepcopy(self.instance_tags)
        instance.tags.append(instance.name)

        self._action['cwoi'].send_goal_and_wait(CreateWorldObjectInstanceGoal(instance))
        result = self._action['cwoi'].get_result()
        return result.instance_id

    def create_pose_cov(self,header,pose):
        pcs = PoseWithCovarianceStamped()
        pcs.pose.pose = pose
        pcs.header = header
        pcs.pose.covariance =  [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        return pcs


    def process_remove_semantic_region(self,req):
        self._action['rwoi'].send_goal_and_wait(RemoveWorldObjectInstanceGoal(req.instance_id))
        resp = self._action['rwoi'].get_result()

        return RemoveSemanticRegionResponse(resp.result)

    def process_get_semantic_region(self,req):
        self._action['woits'].send_goal_and_wait(WorldObjectInstanceTagSearchGoal(self.instance_tags))
        resp = self._action['woits'].get_result()
        
        # Get the correct instance
        instances = [i for i in resp.instances if i.name == req.name]

        if len(instances) > 1:
            rospy.logwarn("There is  more than one instance with current name[%s]. Assigning the first one"%req.name)
        instance = instances[0]
        
        # search WorldObjectDescription 
        description_tags = copy.deepcopy(self.description_tags)
        self._action['wodts'].send_goal_and_wait(WorldObjectDescriptionTagSearchGoal(description_tags))
        description_resp = self._action['wodts'].get_result()
        descriptions = [d for d in description_resp.descriptions if d.description_id == instance.description_id]

        if len(descriptions) > 1:
            rospy.logwarn("There is  more than one descriptions with current id[%s]. Assigning the first one"%instance.description_id)
        description = descriptions[0]

        # parsing data in the descriptor
        message = json_message_converter.convert_json_to_ros_message(description.descriptors[0].type, description.descriptors[0].data)
        region = message
        return GetSemanticRegionResponse(instance.instance_id,description.description_id,instance.pose,region)

    def spin(self):
        rospy.spin()

