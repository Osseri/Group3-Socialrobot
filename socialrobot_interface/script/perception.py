#!/usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion
from mongodb_store.message_store import MessageStoreProxy
from std_msgs import msg as std_msg
from socialrobot_perception_msgs import msg as perception_msg
from interface import InterfaceBase


class PerceptionInterface(InterfaceBase):

    # set the data label name for DB
    VISUAL_DATA_NAME = "/vision/objects"
    VOICE_DATA_NAME = "/voice/command"

    def __init__(self):
        super(PerceptionInterface, self).__init__()
        rospy.loginfo("Initializing PerceptionInterface...")

        # init data format
        self.detected_objects = []
        self.use_mongodb = False
        self.msg_store = None
        if rospy.has_param("/use_mongodb"):
            self.use_mongodb = rospy.get_param("/use_mongodb")
        if self.use_mongodb:
            self.msg_store = MessageStoreProxy()
            self.msg_store.insert_named(self.VISUAL_DATA_NAME, perception_msg.Objects())
            self.msg_store.update_named(self.VISUAL_DATA_NAME, perception_msg.Objects())

        # Publish data
        self.pub_obj = rospy.Publisher(
            "~perception/objects", perception_msg.Objects, queue_size=10
        )
        self.pub_voice = rospy.Publisher(
            "~perception/voice", perception_msg.Voice, queue_size=10
        )
        self.pub_to_context = rospy.Publisher(
            "/object_info", std_msg.Float32MultiArray, queue_size=10
        )

        # Subscribe data from PerceptionManager
        rospy.Subscriber(
            "/perception/objects", perception_msg.Objects, self._callback_objects
        )
        rospy.Subscriber(
            "/perception/voice", perception_msg.Voice, self._callback_voice
        )


    def get_object_info(self, object_id):

        msg_object = self.msg_store.query_named(
            self.VISUAL_DATA_NAME, perception_msg.Objects._type
        )
        for obj in msg_object[0].detected_objects:
            # specific object
            if obj.name.data == object_id:
                return obj

    def _callback_objects(self, data):
        """
        Get the recognition results from topic
        And store the data into the mongoDB
            data : ROS merssage
        """
        self.detected_objects = list(data.detected_objects)

        # Re-publish vision data (frame_id: base_footprint)
        vision_data = data
        self.pub_obj.publish(vision_data)

        # # Re-publish for context_manager (frame_id: map)
        # for idx, obj in enumerate(vision_data.detected_objects):
        #     float_array = self._obj2float_array(idx, obj)
        #     self.pub_to_context.publish(float_array)

        if self.use_mongodb:
            # update data into DB
            self.msg_store.update_named(self.VISUAL_DATA_NAME, vision_data)

            # get it back with a name
            msg_object = self.msg_store.query_named(
                self.VISUAL_DATA_NAME, perception_msg.Objects._type
            )

    def _obj2float_array(self, idx, obj):
        """
        convert msg format
        from: socialrobot_perception_msgs/Object 
        to  : std_msgs/Float32MultiArray 
        """

        float_array = std_msg.Float32MultiArray()
        # object name
        dim = std_msg.MultiArrayDimension()
        dim.label = obj.name.data
        float_array.layout.dim.append(dim)
        # object center position
        position = obj.bb3d.center.position
        float_array.data += [position.x, position.y, position.z]
        # object center orientation
        quaterion = obj.bb3d.center.orientation
        float_array.data += list(
            euler_from_quaternion([quaterion.x, quaterion.y, quaterion.z, quaterion.w])
        )
        float_array.data.append(idx)
        return float_array

    def _callback_voice(self, data):
        """
        Get the voice recofnition results from topic
        And store the data into the mongoDB
            data : ROS merssage
        """
        # rospy.loginfo("User voice command is accepted.")
        if self.use_mongodb:
            # update data into DB
            self.msg_store.update_named(self.VOICE_DATA_NAME, data)
        return
