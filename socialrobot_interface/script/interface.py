#!/usr/bin/env python
import abc
from abc import ABCMeta
from six import with_metaclass
import rospy
import rospkg


class InterfaceBase(with_metaclass(ABCMeta)):
    '''
    InterfaceBase
    '''
    def __init__(self):   
        '''
        Module for Knowledge manager
        '''
        pass
