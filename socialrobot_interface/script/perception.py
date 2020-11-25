#!/usr/bin/env python
import rospy
import rospkg

from socialrobot_perception_msgs.msg import Object, Objects, Voice
from socialrobot_perception_msgs.srv import *
from interface import InterfaceBase
from vision_msgs.msg import *
from std_msgs.msg import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from mongodb_store.message_store import MessageStoreProxy
 
class PerceptionInterface(InterfaceBase):
    def __init__(self):  
        super(PerceptionInterface, self).__init__() 
        rospy.loginfo("Initializing PerceptionInterface...")

        # set the data label name for DB
        self.visual_data_name = "/vision/objects"
        self.voice_data_name = "/voice/command"
        
        # init data format
        self.msg_store = MessageStoreProxy()
        self.msg_store.insert_named(self.visual_data_name, Objects())        
        #self.voice_data_id = self.msg_store.insert_named(self.voice_data_name, Voice())
        self.msg_store.update_named(self.visual_data_name, Objects()) 

        # Subscribe data from PerceptionManager
        topic_object = "/perception/objects"
        topic_voice = "/perception/voice"
        sub_obj = rospy.Subscriber(topic_object, Objects, self._callback_objects)
        sub_voice = rospy.Subscriber(topic_voice, Voice, self._callback_voice)

        # Publish data 
        self.pub_obj = rospy.Publisher("~perception/objects", Objects, queue_size=10)
        self.pub_voice = rospy.Publisher("~perception/voice", Voice, queue_size=10)
        self.pub_to_context = rospy.Publisher("/object_info", Float32MultiArray, queue_size=10)
  
    def get_object_info(self, object_id):

        msg_object = self.msg_store.query_named(self.visual_data_name, Objects._type)
        for obj in msg_object[0].detected_objects:                
            # specific object
            if(obj.name.data == object_id):
                return obj
    
    def _callback_objects(self, data):
        '''
        Get the recognition results from topic
        And store the data into the mongoDB
            data : ROS merssage
        '''
        
        # Re-publish vision data (frame_id: base_footprint)
        vision_data = data
        self.pub_obj.publish(vision_data)

        # # Re-publish for context_manager (frame_id: map)
        # for idx, obj in enumerate(vision_data.detected_objects):
        #     float_array = self._obj2float_array(idx, obj)
        #     self.pub_to_context.publish(float_array)

        # update data into DB        
        self.msg_store.update_named(self.visual_data_name, vision_data)        
        
        # get it back with a name
        msg_object =  self.msg_store.query_named(self.visual_data_name, Objects._type)
        
        return 

 
    def _obj2float_array(self, idx, obj):
        '''
        convert msg format
        from: socialrobot_perception_msgs/Object 
        to  : std_msgs/Float32MultiArray 
        '''

        float_array = Float32MultiArray()
        temp = MultiArrayDimension()
        temp.label = obj.name.data                      # object name
        float_array.layout.dim.append(temp)
        pos_x = obj.bb3d.center.position.x              # object center position
        pos_y = obj.bb3d.center.position.y
        pos_z = obj.bb3d.center.position.z
        # rpy from quat
        orientation_list = [obj.bb3d.center.orientation.x, obj.bb3d.center.orientation.y, obj.bb3d.center.orientation.z, obj.bb3d.center.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)  # object center orientation

        # fill the data 
        float_array.data.append(pos_x)
        float_array.data.append(pos_y)
        float_array.data.append(pos_z)
        float_array.data.append(roll)
        float_array.data.append(pitch)
        float_array.data.append(yaw)
        float_array.data.append(idx)

        return float_array
	
    def _callback_voice(self, data):
        '''
        Get the voice recofnition results from topic
        And store the data into the mongoDB
            data : ROS merssage
        '''
        #rospy.loginfo("User voice command is accepted.")
        # update data into DB
        self.msg_store.update_named(self.voice_data_name, data)

        return  
