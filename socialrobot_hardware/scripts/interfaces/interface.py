import abc
from abc import ABCMeta
from six import with_metaclass 

class InterfaceBase(with_metaclass(ABCMeta)):
    ''' a base class of the robot '''
    ACTION_READY = 0
    ACTION_BUSY = 1
    
    def __init__(self, robot_name, **params):
        """
        Arguments:
            robot_name {[string]} -- [robot platform name]
        """

        self._robot_name = robot_name
        self.action_state = InterfaceBase.ACTION_READY
        self.action_start_time = 0
        self.current_time = 0

        self.desired_trajectory = {}

        print('[%s] Interface is started.' % robot_name)

    @abc.abstractmethod
    def get_state(self):
        return True    

    @abc.abstractmethod
    def set_motion(self):
        return True    

    @abc.abstractmethod
    def update(self):
        return True