#!/usr/bin/env python

'''

 * Copyright (C) 2014 ailab.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License") you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
'''

import os
import os.path
import sys
import signal
import rospy
import rosparam

from sensor_msgs.msg import *
from vision_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from socialrobot_perception_msgs.srv import *
from socialrobot_perception_msgs.msg import *
from rosjava_custom_srv.msg import *

from tf.transformations import euler_from_quaternion, quaternion_from_euler

import time
import math


questioner = rospy.Publisher("/context_manager/query/reception", QueryServiceRequest, queue_size=10)
requestor = rospy.Publisher("/context_manager/main/reception", MainServiceRequest, queue_size=10)



visualObjectPerceptionCount = 0
visualRobotBodyPerceptionCount = 0
visualRobotHandPerceptionCount = 0
visualRobotLeftHandPerceptionCount = 0
visualRobotRightHandPerceptionCount = 0
jointPerceptionCount = 0

visualObjectPerceptionInterval = 0
visualRobotBodyPerceptionInterval = 0
visualRobotHandPerceptionInterval = 0
visualRobotLeftHandPerceptionInterval = 0
visualRobotRightHandPerceptionInterval = 0
jointPerceptionInterval = 0



removeTime = 10
removeInterval = 10

oSubject = []                   
oProperty = []
oObject = []
oGraph = []
oStatus = []
oManager = []


bSubject = []
bProperty = []
bObject = []
bGraph = []
bStatus = []
bManager = []

hSubject = []
hProperty = []
hObject = []
hGraph = []
hStatus = []
hManager = []

jSubject = []
jProperty = []
jObject = []
jGraph = []
jStatus = []
jManager = []

class ContextListener():
    def __init__(self):
        visualObjectPerceptionCount = 0
        visualRobotBodyPerceptionCount = 0
        visualRobotHandPerceptionCount = 0
        visualRobotLeftHandPerceptionCount = 0
        visualRobotRightHandPerceptionCount = 0
        jointPerceptionCount = 0

        visualObjectPerceptionInterval = 0
        visualRobotBodyPerceptionInterval = 0
        visualRobotHandPerceptionInterval = 0
        visualRobotLeftHandPerceptionInterval = 0
        visualRobotRightHandPerceptionInterval = 0
        jointPerceptionInterval = 0

        removeTime = 10
        removeInterval = 10

        oSubject = []
        oProperty = []
        oObject = []
        oGraph = []
        oStatus = []
        oManager = []


        bSubject = []
        bProperty = []
        bObject = []
        bGraph = []
        bStatus = []
        bManager = []

        hSubject = []
        hProperty = []
        hObject = []
        hGraph = []
        hStatus = []
        hManager = []

        jSubject = []
        jProperty = []
        jObject = []
        jGraph = []
        jStatus = []
        jManager = []

        fsdSubject = []
        fsdProperty = []
        fsdObject = []
        fsdStatus = []
        fsdInd = 0
        fsdManager = []

        self.oIdCount = [[0]*2]*50
        oIdInd = 0

        self.bIdCount = [[0]*2]*50
        bIdInd = 0

        self.hIdCount = [[0]*2]*50
        self.hIdInd = 0

        self.jIdCount = [[0]*2]*50
        jIdInd = 0

        obj_name = []
        obj_confidence = []
        affordance_name = []
        affordance_cofidence = []

        queueSize = 1000

        rospy.Subscriber('/objects_infos', Float32MultiArray, self.callback_objects, queue_size=10)
        #rospy.Subscriber('/joint_statess', JointState, self.callback_joints, queue_size=10)
        #rospy.Subscriber('/visual_robot_perceptions', Float32MultiArray, self.callback_robot, queue_size=10)
        rospy.Subscriber('/affordance_info', Person, self.callback_objects, queue_size=10)
        rospy.Subscriber('/context_manager/query/provision_for_pm', QueryServiceResponse, self.callback_answer, queue_size=10)


    def prepare(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))+"/"
        with open(dir_path+'object_list.txt','r') as f:
            i=0
            for line in f.readlines():
                tmp=line.lower().strip()
                if not tmp:continue
                self.obj_name2id[tmp]=i;i+=1
        #for k, v in self.obj_name2id.items():
        #    print(k, v)
        

    def callback_answer(self, data):
        ans = data.result
        rospy.loginfo(String.format("The response is : " + ans))

    def callback_objects(self, data):

        for k in range(0, len(data.data), 11):
            subVisualObjectPerceptionCount=0
            assertString = ""
            retractString = ""
            x = data.data[0+k]
            y = data.data[1+k]
            z = data.data[2+k]
            a = data.data[3+k]
            b = data.data[4+k]
            c = data.data[5+k]
            d = data.data[6+k]
            width = data.data[7+k]
            depth = data.data[8+k]
            height = data.data[9+k]
            IDN = data.data[10+k]
            ID = ""
            type = ""

            # radian to degree
            # a =  math.degrees(a)
            # b =  math.degrees(b)
            # c =  math.degrees(c)

            if IDN == 0:
                    ID = "milk"
                    oIdInd=0
            elif IDN == 1:
                    ID = "juice"
                    oIdInd=1
            elif IDN == 2:
                    ID = "table"
                    oIdInd=2
            elif IDN == 3:
                    ID = "red_gotica"
                    oIdInd=3
            elif IDN ==  4:
                    ID = "bakey"
                    oIdInd=4
            elif IDN ==  5:
                    ID = "pringles_onion"
                    oIdInd=5
            elif IDN ==  6:
                    ID = "diget_box"
                    oIdInd=6
            elif IDN ==  7:
                    ID = "diget"
                    oIdInd=7
            elif IDN ==  8:
                    ID = "gotica"
                    oIdInd=8
            elif IDN ==  9:
                    ID = "vitamin_water"
                    oIdInd=9
            elif IDN ==  10:
                    ID = "diget_small_box"
                    oIdInd=10
            elif IDN ==  11:
                    ID = "chocolate_syrup"
                    oIdInd=11
            elif IDN ==  12:
                    ID = "orange_cup"
                    oIdInd=12
            elif IDN ==  13:
                    ID = "orange_juice"
                    oIdInd=13
            elif IDN ==  14:
                    ID = "mug_red"
                    oIdInd=14
            elif IDN ==  15:
                    ID = "mug_white"
                    oIdInd=15
            elif IDN ==  16:
                    ID = "corn_flight"
                    oIdInd=16
            elif IDN ==  17:
                    ID = "telephone"
                    oIdInd=17
            elif IDN ==  18:
                    ID = "calculator"
                    oIdInd=18
            elif IDN ==  19:
                ID = "black_bowl"
                oIdInd=19
            elif IDN ==  20:
                ID = "handcream"
                oIdInd=20
            elif IDN ==  21:
                ID = "small_milk"
                oIdInd=21
            elif IDN ==  22:
                ID = "red_cup"
                oIdInd=22
            elif IDN ==  23:
                ID = "elephant_mug"
                oIdInd=23
            elif IDN ==  24:
                ID = "kettle"
                oIdInd=24
            elif IDN ==  25:
                ID = "eiffel_tower_mug"
                oIdInd=25
            else:
                oIdInd=49
                continue

                

            #visualObjectPerceptionCount++//=oIdCount[oIdInd][1]++
            subVisualObjectPerceptionCount=self.oIdCount[oIdInd][1]
            self.oIdCount[oIdInd][1]+=1
            visualObjectPerceptionCount=subVisualObjectPerceptionCount
            
            current_time = int(time.time() * 1000) # currentTimeMillis() : 1/1000 s return --> 0m 1s = 100 --> 1s  

            #print(current_time)


            retractString = "arbi:{} knowrob:widthOfObject A objectPerception o".format(ID)
            retractTriple(retractString)
            assertString = "arbi:{} knowrob:widthOfObject literal(type(xsd,'{}')) objectPerception o".format(ID, width)
            assertTriple(assertString)

            print(assertString)

            
            retractString = "arbi:{} knowrob:depthOfObject A objectPerception o".format(ID)
            retractTriple(retractString)
            assertString = "arbi:{} knowrob:depthOfObject literal(type(xsd,'{}')) objectPerception o".format(ID, depth)
            assertTriple(assertString)

            retractString = "arbi:{} knowrob:heightOfObject A objectPerception o".format(ID)
            retractTriple(retractString)
            assertString = "arbi:{} knowrob:heightOfObject literal(type(xsd,'{}')) objectPerception o".format(ID, height)
            assertTriple(assertString)

            assertString = "arbi:visualObjectPerception_{}_{} rdf:type knowrob:'VisualObjectPerception' objectPerception o".format(ID, visualObjectPerceptionCount)

            assertTriple(assertString)
            # startTime
            assertString = "arbi:visualObjectPerception_{}_{} knowrob:startTime arbi:timepoint_{} objectPerception o".format(ID, visualObjectPerceptionCount, current_time)

            assertTriple(assertString)

            
            # objectActedOn
            assertString = "arbi:visualObjectPerception_{}_{} knowrob:objectActedOn arbi:{} objectPerception o".format(ID, visualObjectPerceptionCount, ID)

            assertTriple(assertString)
            # eventOccursAt
            assertString = "arbi:visualObjectPerception_{}_{} knowrob:eventOccursAt arbi:rotationMatrix3D_{}{} objectPerception o".format(ID, visualObjectPerceptionCount, ID, subVisualObjectPerceptionCount)

            assertTriple(assertString)

            
            # rotationMatrix3D
            assertString = "arbi:rotationMatrix3D_{}{} rdf:type knowrob:RotationMatrix3D objectPerception o".format(ID, subVisualObjectPerceptionCount)

            assertTriple(assertString)
            # x,y,z
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m03 literal(type(xsd:double,'{}')) objectPerception o".format(ID, subVisualObjectPerceptionCount, x)
                        
            assertTriple(assertString)
            
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m13 literal(type(xsd:double,'{}')) objectPerception o".format(ID, subVisualObjectPerceptionCount, y)

            assertTriple(assertString)
        
        
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m23 literal(type(xsd:double,'{}')) objectPerception o".format(ID, subVisualObjectPerceptionCount, z)

            assertTriple(assertString)
        
            # a,b,g
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m02 literal(type(xsd:double,'{}')) objectPerception o".format(ID, subVisualObjectPerceptionCount, a)
                        
            assertTriple(assertString)
            
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m12 literal(type(xsd:double,'{}')) objectPerception o".format(ID, subVisualObjectPerceptionCount, b)

            assertTriple(assertString)
            
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m22 literal(type(xsd:double,'{}')) objectPerception o".format(ID, subVisualObjectPerceptionCount, c)
                        
            assertTriple(assertString)
            
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m32 literal(type(xsd:double,'{}')) objectPerception o".format(ID, subVisualObjectPerceptionCount, 0)
                        
            assertTriple(assertString)
            

            #test
            #assertString = ID
            #          + " currentObjectPose a o"
            #updateCurrentTriple(assertString)
            # print data
            '''
            result = "[ContextListener/visual_object_perception] time: {}, Object_name: {}, Object Pose(X: {}, Y: {}, Z: {}),Object Orientation(A: {}, B: {}, G: {}).format(current_time, ID, x, y, z, a, b, c)
            print(result)
            '''

            if self.oIdCount[oIdInd][1]%removeTime==0:
                for i in range(self.oIdCount[oIdInd][0], self.oIdCount[oIdInd][1]-removeInterval, 1):
                    retractTriple("arbi:visualObjectPerception_{}_{} A B C o".format(ID, i))
                    retractTriple("arbi:rotationMatrix3D_{}{} A B C o".format(ID, i))
                self.oIdCount[oIdInd][0]=self.oIdCount[oIdInd][1]-removeInterval
                         
        send_message("ob")
               
    def callback_robot(self, data):
        if len(data.data) == 0:return

        subVisualRobotBodyPerceptionCount=0
        subVisualRobotHandPerceptionCount=0
        assertString = ""
        x = data.data[0]
        y = data.data[1]
        z = data.data[2]
        a = data.data[3]
        b = data.data[4]
        c = data.data[5]
        depth = data.data[6]
        width = data.data[7]
        height = data.data[8]
        id = data.data[9]

        '''
        finger_x = data.data[7];
        finger_y = data.data[8];
        finger_z = data.data[9];
        '''

        ID = ""
        spaceName="space_surrounded_by_hand"
        type = ""

        # radian to degree
        # a =  math.degrees(a);
        # b =  math.degrees(b);
        # c =  math.degrees(c);


        if id ==  10:
                ID = "socialrobot" #"hubo_1";//"robot_body"
                bIdInd=0;
        elif id ==  12:
                ID = "left_hand_1" #"hubo_left_hand_1";//robot_hand";
                self.hIdInd=0
        elif id ==  11:
                ID = "right_hand_1" #"robot_finger_mid";
                self.hIdInd=1
        else:
                bIdInd=49
                self.hIdInd=49
        
        current_time = int(time.time() * 1000)

        global visualRobotBodyPerceptionCount, visualRobotLeftHandPerceptionCount, visualRobotRightHandPerceptionCount


        # assert data
        if ID == "socialrobot":
            visualRobotBodyPerceptionCount+=1
            subVisualRobotBodyPerceptionCount=self.bIdCount[bIdInd][1]
            self.bIdCount[bIdInd][1]+=1


            assertString = "arbi:visualRobotBodyPerception{} rdf:type knowrob:VisualRobotBodyPerception robotPerception b".format(visualRobotBodyPerceptionCount)

            assertTriple(assertString)
            # startTime
            assertString = "arbi:visualRobotBodyPerception{} knowrob:startTime arbi:timepoint_{} robotPerception b".format(visualRobotBodyPerceptionCount, current_time)

            assertTriple(assertString)
            
            # objectActedOn
            assertString = "arbi:visualRobotBodyPerception{} knowrob:objectActedOn arbi:{} robotPerception b".format(visualRobotBodyPerceptionCount, ID)

            assertTriple(assertString)
            # eventOccursAt
            assertString = "arbi:visualRobotBodyPerception{} knowrob:eventOccursAt arbi:rotationMatrix3D_{}{} robotPerception b".format(visualRobotBodyPerceptionCount, ID, subVisualRobotBodyPerceptionCount)

            assertTriple(assertString)
            
            # rotationMatrix3D
            assertString = "arbi:rotationMatrix3D_{}{} rdf:type knowrob:RotationMatrix3D robotPerception b".format(ID, subVisualRobotBodyPerceptionCount)

            assertTriple(assertString)
            # x,y,z
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m03 literal(type(xsd:double,'{}')) robotPerception b".format(ID, subVisualRobotBodyPerceptionCount, x)

            assertTriple(assertString)
            
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m13 literal(type(xsd:double,'{}')) robotPerception b".format(ID, subVisualRobotBodyPerceptionCount, y)

            assertTriple(assertString)
            
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m23 literal(type(xsd:double,'{}')) robotPerception b".format(ID, subVisualRobotBodyPerceptionCount, z)

            assertTriple(assertString)
            
            # a,b,g
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m02 literal(type(xsd:double,'{}')) robotPerception b".format(ID, subVisualRobotBodyPerceptionCount, a)

            assertTriple(assertString)
            
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m12 literal(type(xsd:double,'{}')) robotPerception b".format(ID, subVisualRobotBodyPerceptionCount, b)

            assertTriple(assertString)
            
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m22 literal(type(xsd:double,'{}')) robotPerception b".format(ID, subVisualRobotBodyPerceptionCount, c)

            assertTriple(assertString)
            
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m32 literal(type(xsd:double,'{}')) robotPerception b".format(ID, subVisualRobotBodyPerceptionCount, 0)

            assertTriple(assertString)


            if self.bIdCount[bIdInd][1]%removeTime == 0:
                for i in range(self.bIdCount[bIdInd][0], self.bIdCount[bIdInd][1]-removeInterval, 1):
                    retractTriple("arbi:visualRobotBodyPerception{} A B C b".format(i))
                    retractTriple("arbi:rotationMatrix3D_{}{} A"+" B C b".format(ID, i))
                
                self.bIdCount[bIdInd][0]=self.bIdCount[bIdInd][1]-removeInterval
            

            send_message("bb")

                          
        elif ID == "left_hand_1" or ID == "right_hand_1":                         
            if ID == "left_hand_1":
                visualRobotHandPerceptionCount=visualRobotLeftHandPerceptionCount
                visualRobotLeftHandPerceptionCount+=1
            elif ID == "right_hand_1":
                visualRobotHandPerceptionCount=visualRobotRightHandPerceptionCount
                visualRobotRightHandPerceptionCount+=1

            subVisualRobotHandPerceptionCount=self.hIdCount[self.hIdInd][1]
            self.hIdCount[self.hIdInd][1]+=1

            assertString = "arbi:visualRobotHandPerception_{}_{} rdf:type knowrob:VisualRobotHandPerception robotPerception h".format(ID, visualRobotHandPerceptionCount)

            assertTriple(assertString)
            # startTime
            assertString = "arbi:visualRobotHandPerception_{}_{} knowrob:startTime arbi:timepoint_{} robotPerception h".format(ID, visualRobotHandPerceptionCount, current_time)

            assertTriple(assertString)
            
            # objectActedOn
            assertString = "arbi:visualRobotHandPerception_{}_{} knowrob:objectActedOn arbi:{} robotPerception h".format(ID, visualRobotHandPerceptionCount, ID)

            assertTriple(assertString)
            # eventOccursAt
            assertString = "arbi:visualRobotHandPerception_{}_{} knowrob:eventOccursAt arbi:rotationMatrix3D_{}{} robotPerception h".format(ID, visualRobotHandPerceptionCount, ID, subVisualRobotHandPerceptionCount)

            assertTriple(assertString)
            
            # rotationMatrix3D
            assertString = "arbi:rotationMatrix3D_{}{} rdf:type knowrob:RotationMatrix3D robotPerception h".format(ID, subVisualRobotHandPerceptionCount)

            assertTriple(assertString)

            assertString = "arbi:rotationMatrix3D_{}{} rdf:type knowrob:RotationMatrix3D robotPerception h".format(spaceName, subVisualRobotHandPerceptionCount)

            assertTriple(assertString)
            # inFrontOf-Generally
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:inFrontOf-Generally arbi:rotationMatrix3D_{}{} robotPerception h".format(spaceName, subVisualRobotHandPerceptionCount, ID, subVisualRobotHandPerceptionCount)

            assertTriple(assertString)
            
            # x,y,z
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m03 literal(type(xsd:double,'{}')) robotPerception h".format(ID, subVisualRobotHandPerceptionCount, x)

            assertTriple(assertString)
            
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m13 literal(type(xsd:double,'{}')) robotPerception h".format(ID, subVisualRobotHandPerceptionCount, y)

            assertTriple(assertString)
            
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m23 literal(type(xsd:double,'{}')) robotPerception h".format(ID, subVisualRobotHandPerceptionCount, z)

            assertTriple(assertString)
    
            # a,b,g
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m02 literal(type(xsd:double,'{}')) robotPerception h".format(ID, subVisualRobotHandPerceptionCount, a)

            assertTriple(assertString)
            
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m12 literal(type(xsd:double,'{}')) robotPerception h".format(ID, subVisualRobotHandPerceptionCount, b)

            assertTriple(assertString)
            
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m22 literal(type(xsd:double,'{}')) robotPerception h".format(ID, subVisualRobotHandPerceptionCount, c)

            assertTriple(assertString)
            
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m32 literal(type(xsd:double,'{}')) robotPerception h".format(ID, subVisualRobotHandPerceptionCount, 0)

            assertTriple(assertString)


        elif ID == "robot_finger_mid":

            subVisualRobotHandPerceptionCount=self.hIdCount[self.hIdInd][1]

            # x,y,z
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m03 literal(type(xsd:double,'{}')) robotPerception h".format(spaceName, subVisualRobotHandPerceptionCount, x)

            assertTriple(assertString)
            
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m13 literal(type(xsd:double,'{}')) robotPerception h".format(spaceName, subVisualRobotHandPerceptionCount, y)

            assertTriple(assertString)
            
            assertString = "arbi:rotationMatrix3D_{}{} knowrob:m23 literal(type(xsd:double,'{}')) robotPerception h".format(spaceName, subVisualRobotHandPerceptionCount, z)

            assertTriple(assertString)
                

        if self.hIdCount[self.hIdInd][1]%removeTime==0:
            for i in range(self.hIdCount[self.hIdInd][0], self.hIdCount[self.hIdInd][1]-removeInterval, 1):
                retractTriple("arbi:visualRobotHandPerception_{}_{} A B C h".format(ID, i))
                retractTriple("arbi:rotationMatrix3D_{}{} A B C h".format(ID, i))
                retractTriple("arbi:rotationMatrix3D_{}{} A B C h".format(spaceName, i))
            
            self.hIdCount[self.hIdInd][0]=self.hIdCount[self.hIdInd][1]-removeInterval
        

        
        send_message("hb")
        
        result = "[ContextListener/robot_hand_perception] time: {}, Object_name: {}, Object Pose(X: {}, Y: {}, Z: {}), Object Orientation(A: {}, B: {}, G: {})".format(current_time, ID, x, y, z, a, b, c)
        #print(result)
                     

    def callback_joints(self, data):
        
        current_time = data.header.stamp.secs
        assertString = ""
        name = ""
        position=0
        velocity=0
        effort=0

        # test
        # if((jointPerceptionCount++)%10==0)
        for i in range(0, len(data.name), 1):
            name = data.name[i]
            position = data.position[i]
            position = math.degrees(position)
            if len(data.velocity) > 0:
                velocity = data.velocity[i]
            else:
                velocity = 0
            if len(data.effort) > 0:
                effort = data.effort[i]
            else:
                effort = 0

            # assert data
            # type
            global jointPerceptionCount, jointPerceptionInterval
            jointPerceptionCount+=1

            assertString = "arbi:jointPerception{} rdf:type knowrob:JointPerception graspPerception j".format(jointPerceptionCount)

            assertTriple(assertString)
            # startTime
            assertString = "arbi:jointPerception{} knowrob:startTime arbi:timepoint_{} graspPerception j".format(jointPerceptionCount, current_time)

            assertTriple(assertString)
        
            # objectActedOn
            assertString = "arbi:jointPerception{} knowrob:objectActedOn arbi:{} graspPerception j".format(jointPerceptionCount, name)

            assertTriple(assertString)
            # radius
            assertString = "arbi:jointPerception{} knowrob:radius literal(type(xsd:double,'{}')) graspPerception j".format(jointPerceptionCount, position)

            assertTriple(assertString)
            
            # velocity
            assertString = "arbi:jointPerception{} knowrob:velocity literal(type(xsd:double,'{}')) graspPerception j".format(jointPerceptionCount, velocity)

            assertTriple(assertString)
            
            # effort
            assertString = "arbi:jointPerception{} knowrob:effort literal(type(xsd:double,'{}')) graspPerception j".format(jointPerceptionCount, effort)

            assertTriple(assertString)

            
            
            result_joint = "[ContextListfener/jointPerception] time: {}, Joint Name: {}, Joint radius:({}), Joint Velocity({}), Joint Effort:({})".format(current_time, name, position, velocity, effort)
            # print(result_joint) 
                                              
                    
            tmps = 5
            if jointPerceptionCount%(6*tmps)==0: #data.getName().size()
                    for k in range(jointPerceptionInterval, jointPerceptionCount-tmps*6, 1):
                        retractTriple("arbi:jointPerception{} A B C j".format(k))
                    jointPerceptionInterval=jointPerceptionCount-tmps*6
                
        send_message("jb")
    
    def callback_affordance(self, data):
        print("!!!!!!!!!!!!!!!!!!!!!!!1")
        for i in range(len(data.objName)):
            print("~~~~~~~~~~"+data.objName[i])
        

def assertTriple(triple):
    params = triple.split(" ")

    if params[4] == "o":
        oSubject.append(params[0])
        oProperty.append(params[1])
        oObject.append(params[2])
        oGraph.append(params[3])
        oStatus.append(3)
        oManager.append("ContextManager")
    
    elif params[4] == "b":
        bSubject.append(params[0])
        bProperty.append(params[1])
        bObject.append(params[2])
        bGraph.append(params[3])
        bStatus.append(3)
        bManager.append("ContextManager")
    
    elif params[4] == "h":
        hSubject.append(params[0])
        hProperty.append(params[1])
        hObject.append(params[2])
        hGraph.append(params[3])
        hStatus.append(3)
        hManager.append("ContextManager")
    
    elif params[4] == "j":
        jSubject.append(params[0])
        jProperty.append(params[1])
        jObject.append(params[2])
        jGraph.append(params[3])
        jStatus.append(3)
        jManager.append("ContextManager")
          
     


def retractTriple(triple):
    params = triple.split(" ")

    params[3]=""

    if params[4] == "o":
        oSubject.append(params[0])
        oProperty.append(params[1])
        oObject.append(params[2])
        oGraph.append(params[3])
        oStatus.append(4)
        oManager.append("ContextManager")
    
    elif params[4] == "b":
        bSubject.append(params[0])
        bProperty.append(params[1])
        bObject.append(params[2])
        bGraph.append(params[3])
        bStatus.append(4)
        bManager.append("ContextManager")
    
    elif params[4] == "h":
        hSubject.append(params[0])
        hProperty.append(params[1])
        hObject.append(params[2])
        hGraph.append(params[3])
        hStatus.append(4)
        hManager.append("ContextManager")
    
    elif params[4] == "j":
        jSubject.append(params[0])
        jProperty.append(params[1])
        jObject.append(params[2])
        jGraph.append(params[3])
        jStatus.append(4)
        jManager.append("ContextManager")



def updateTriple(triple):
    retractTriple(triple)
    assertTriple(triple)

def send_message(msg):

    request = MainServiceRequest()

    if msg == "ob":
        set_request(request, oProperty, oSubject, oObject, oGraph, oStatus, oManager)
        clear_list(oSubject, oProperty, oObject, oGraph, oStatus, oManager)


    elif msg == "bb":
        set_request(request, bProperty, bSubject, bObject, bGraph, bStatus, bManager)
        clear_list(bSubject, bProperty, bObject, bGraph, bStatus, bManager)


    elif msg == "hb":
        set_request(request, hProperty, hSubject, hObject, hGraph, hStatus, hManager)
        clear_list(hSubject, hProperty, hObject, hGraph, hStatus, hManager)


    elif msg == "jb":
        set_request(request, jProperty, jSubject, jObject, jGraph, jStatus, jManager)
        clear_list(jSubject, jProperty, jObject, jGraph, jStatus, jManager)
    
    requestor.publish(request)

def set_request(request, property, subject, object, graph, status, manager):

    request.predicate = property
    request.param1 = subject
    request.param2 = object
    request.param4 = graph
    request.status = status
    request.manager = manager
        
def clear_list(*lists):
    for l in lists:
        #del l[:]#l.clear() -- for python version > 3.3
        l = []

def sleep(t):
    # if java - ms
    # if python - s
    time.sleep(t)


##############################
# Main function
##############################
if __name__ == '__main__':
    # ros initialize
    rospy.init_node('context_listener')        

    # context listener
    cl = ContextListener()

    # Start
    rospy.loginfo('[ContextListener] Service Started!')




    try:
        sleep(0.5)
        mRequest = MainServiceRequest()
        req = [] #[]
        req.append("init")
        st = [99]
        mRequest.predicate = req
        mRequest.status = st

        req = []
        req.append("ContextManager")
        mRequest.manager = req
        req = []
        req.append("social_robot")
        mRequest.param1 = req

        requestor.publish(mRequest)
        sleep(1)
    except Exception as e:
            print("Error : ", e)


    loop_freq = 100 # 10hz
    r = rospy.Rate(loop_freq)
    while not rospy.is_shutdown():
        #cl.update()
        r.sleep()
