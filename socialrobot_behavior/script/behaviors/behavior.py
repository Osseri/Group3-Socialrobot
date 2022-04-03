import abc
from abc import ABCMeta
from six import with_metaclass


class BehaviorBase(with_metaclass(ABCMeta)):
    ''' a base class of the behavior class '''
    # _name = ''

    # 0:none, 1:left, 2:right, 3:both
    # _target_arm = 0
    # _target_gripper = 0
    # _target_base = 0

    # 0:init, 1:ready for behavior, 2:runnging, 3:done, -1:error
    # 0->1->2->3
    INIT_STATE = 0
    READY_STATE = 1
    RUNNING_STATE = 2
    DONE_STATE = 3
    ERROR_STATE = -1
    _state = DONE_STATE
    _state_transition = {}

    def __init__(self, name, **params):
        self._name = name
        self._target_arm = params.get('target_arm', 0)
        self._target_gripper = params.get('target_gripper', 0)
        self._target_base = params.get('target_base', 0)
        self._hardware_if = params.get('hardware_interface', 'vrep')
        self.behavior_data = {}
        self.motion_trajectory = {}
        self.input_args = []
        self.output_args = []
        self.controller = []
        self.hardware_group = []

        self._state = self.DONE_STATE
        self._state_transition = {}

        print('[Behavior] name:%s' % name)

        self._state_transition[self.INIT_STATE] = (self.check_requirement, {
            True: self.READY_STATE,
            False: self.ERROR_STATE
        })
        self._state_transition[self.READY_STATE] = (self.prepare_behavior, {
            True: self.RUNNING_STATE,
            False: self.ERROR_STATE
        })
        self._state_transition[self.RUNNING_STATE] = (self.run_behavior, {
            1: self.DONE_STATE,
            0: self.RUNNING_STATE,
            -1: self.ERROR_STATE
        })
        self._state_transition[self.DONE_STATE] = (self.finish_behavior, {
            True: self.DONE_STATE,
            False: self.ERROR_STATE
        })
        self._state_transition[self.ERROR_STATE] = (lambda: False, {True: self.ERROR_STATE, False: self.ERROR_STATE})

    @abc.abstractmethod
    def check_requirement(self):
        ''' target body and controller '''
        print('check_requirement')
        return True

    @abc.abstractmethod
    def prepare_behavior(self):
        ''' set-up controller '''
        print('prepare_behavior')
        return True

    @abc.abstractmethod
    def run_behavior(self):
        ''' run behavior '''
        print('run_behavior')
        return True

    @abc.abstractmethod
    def finish_behavior(self):
        ''' finish behavior '''
        print('finish_behavior')
        return True

    @abc.abstractmethod
    def get_motion(self):
        ''' get motion '''
        print('get_motion')
        return True

    def check_requirement(self):
        ''' check requirement '''      
        return self.input_args

    def reset_motion_ref(self, **params):
        self.behavior_data['joint'] = params.get('joint')
        self.behavior_data['path'] = params.get('path')
        self.behavior_data['duration'] = params.get('duration', 2.0)

        self._state = self.INIT_STATE

    def loop_until_done(self):
        '''
        control loop(-1:error, 0:running, 1:done)
        callback from Behavior Manager
        '''
        callback_fn, transition = self._state_transition.get(self._state, self.ERROR_STATE)
        result = callback_fn()
        next_state = transition.get(result, -1)

        if next_state == -1:
            self._state = self.ERROR_STATE
        else:
            self._state = next_state

        return self._state
