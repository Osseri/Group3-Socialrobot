#!/usr/bin/env python
from PyQt5 import QtCore, QtGui, QtWidgets, Qt
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtCore import QThread
from PyQt5.QtCore import QWaitCondition
from PyQt5.QtCore import QMutex
from PyQt5.QtGui import QColor, QPalette, QBrush

import sys
import cPickle as pickle
import rospy
import rospkg

from std_msgs.msg import String
from std_srvs.srv import *
from socialrobot_task_planner.srv import *
from socialrobot_task_msgs.srv import *
from socialrobot_task_msgs.msg import *
from socialrobot_actionlib.srv import *
from socialrobot_actionlib.msg import *
import socialrobot_interface.srv
from smach_msgs.msg import SmachContainerStatus

primitive_plan = ''

class Thread(QThread):

    change_value = 0
    def __init__(self, tree_widget):
        QThread.__init__(self)
        self.cond = QWaitCondition()
        self.mutex = QMutex()
        rospy.init_node('smach_status_listener')
        self.status_srv = rospy.ServiceProxy("/socialrobot_state/status", Trigger)
        self.idx = 0
        self.action = ''
        self.planView = tree_widget

    def __del__(self):
        self.wait()

    def refresh_item(self, item):
        return

    def update_view(self):
        self.planView.clear()
        global primitive_plan        
        for idx, action in enumerate(primitive_plan):
            act_list = []
            act_list.append(action.name)
            parent = self.add_plan_tree(idx, action.name, '  '.join(action.parameters),'','','')
        return

    def add_plan_tree(self, action_idx, action_name, action_arg, dispatch_time='', duration='', status=''):
        item = QtWidgets.QTreeWidgetItem(self.planView)
        item.setText(0, str(action_idx))
        item.setText(1, action_name)
        item.setText(2, action_arg)
        item.setText(3, dispatch_time)
        item.setText(4, duration)
        item.setText(5, status)
        return item

    def run(self):
        while True:
            self.mutex.lock()
            rospy.wait_for_service("/socialrobot_state/status")
            req = TriggerRequest()
            res = self.status_srv(req)
            
            idx = int(res.message.split(',')[0])
            action = res.message.split(',')[1]            

            if self.planView.topLevelItemCount()>0:

                self.update_view()

                if idx > -1:   
                    for i in range(0,idx,1):
                        item = self.planView.topLevelItem(i)
                        item.setBackground(1, QColor('#7fc97f'))
                        item.setForeground(1, QBrush(QColor('#ffffff')))
                        item.setBackground(2, QColor('#7fc97f'))
                        item.setForeground(2, QBrush(QColor('#ffffff')))             
                    item = self.planView.topLevelItem(idx)
                    item.setBackground(1, QColor('#8094ff'))
                    item.setForeground(1, QBrush(QColor('#ffffff')))
                    item.setBackground(2, QColor('#8094ff'))
                    item.setForeground(2, QBrush(QColor('#ffffff')))    

            self.msleep(1000)  
            self.mutex.unlock()


class Ui_SocialrobotActionViewer(object):

    def setupUi(self, SocialrobotActionViewer):
        SocialrobotActionViewer.setObjectName("SocialrobotActionViewer")
        SocialrobotActionViewer.resize(819, 629)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(SocialrobotActionViewer.sizePolicy().hasHeightForWidth())
        SocialrobotActionViewer.setSizePolicy(sizePolicy)
        self.gridLayout = QtWidgets.QGridLayout(SocialrobotActionViewer)
        self.gridLayout.setObjectName("gridLayout")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.planButton = QtWidgets.QPushButton(SocialrobotActionViewer)
        self.planButton.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.planButton.setObjectName("planButton")
        self.horizontalLayout_2.addWidget(self.planButton)
        self.loadButton = QtWidgets.QPushButton(SocialrobotActionViewer)
        self.loadButton.setObjectName("loadButton")
        self.horizontalLayout_2.addWidget(self.loadButton)
        self.clearButton = QtWidgets.QPushButton(SocialrobotActionViewer)
        self.clearButton.setObjectName("clearButton")
        self.horizontalLayout_2.addWidget(self.clearButton)
        self.label = QtWidgets.QLabel(SocialrobotActionViewer)
        self.label.setObjectName("label")
        self.horizontalLayout_2.addWidget(self.label)
        self.statusLabel = QtWidgets.QLineEdit(SocialrobotActionViewer)
        self.statusLabel.setEnabled(True)
        self.statusLabel.setCursor(QtGui.QCursor(QtCore.Qt.BlankCursor))
        self.statusLabel.setAutoFillBackground(False)
        self.statusLabel.setFrame(True)
        self.statusLabel.setReadOnly(True)
        self.statusLabel.setObjectName("statusLabel")
        self.horizontalLayout_2.addWidget(self.statusLabel)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem)
        self.gridLayout.addLayout(self.horizontalLayout_2, 0, 0, 1, 1)
        self.splitter = QtWidgets.QSplitter(SocialrobotActionViewer)
        self.splitter.setCursor(QtGui.QCursor(QtCore.Qt.ArrowCursor))
        self.splitter.setOrientation(QtCore.Qt.Vertical)
        self.splitter.setChildrenCollapsible(False)
        self.splitter.setObjectName("splitter")
        self.planView = QtWidgets.QTreeWidget(self.splitter)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.planView.sizePolicy().hasHeightForWidth())
        self.planView.setSizePolicy(sizePolicy)
        self.planView.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.planView.setObjectName("planView")
        self.layoutWidget = QtWidgets.QWidget(self.splitter)
        self.layoutWidget.setObjectName("layoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.layoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label_2 = QtWidgets.QLabel(self.layoutWidget)
        self.label_2.setIndent(4)
        self.label_2.setObjectName("label_2")
        self.verticalLayout.addWidget(self.label_2)
        self.goalView = QtWidgets.QListWidget(self.layoutWidget)
        self.goalView.setSelectionMode(QtWidgets.QAbstractItemView.MultiSelection)
        self.goalView.setObjectName("goalView")
        self.verticalLayout.addWidget(self.goalView)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.removeGoalButton = QtWidgets.QPushButton(self.layoutWidget)
        self.removeGoalButton.setObjectName("removeGoalButton")
        self.horizontalLayout_3.addWidget(self.removeGoalButton)
        self.addGoalButton = QtWidgets.QPushButton(self.layoutWidget)
        self.addGoalButton.setEnabled(True)
        self.addGoalButton.setObjectName("addGoalButton")
        self.horizontalLayout_3.addWidget(self.addGoalButton)
        self.goalNameComboBox = QtWidgets.QComboBox(self.layoutWidget)
        self.goalNameComboBox.setAutoFillBackground(False)
        self.goalNameComboBox.setFrame(True)
        self.goalNameComboBox.setObjectName("goalNameComboBox")
        self.horizontalLayout_3.addWidget(self.goalNameComboBox)
        self.goalComboBox = QtWidgets.QComboBox(self.layoutWidget)
        self.goalComboBox.setMaximumSize(QtCore.QSize(300, 16777215))
        self.goalComboBox.setInsertPolicy(QtWidgets.QComboBox.InsertAlphabetically)
        self.goalComboBox.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToContents)
        self.goalComboBox.setObjectName("goalComboBox")
        self.horizontalLayout_3.addWidget(self.goalComboBox)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem1)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.layoutWidget1 = QtWidgets.QWidget(self.splitter)
        self.layoutWidget1.setObjectName("layoutWidget1")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.layoutWidget1)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label_3 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_3.setIndent(4)
        self.label_3.setObjectName("label_3")
        self.verticalLayout_2.addWidget(self.label_3)
        self.modelView = QtWidgets.QListWidget(self.layoutWidget1)
        self.modelView.setEnabled(True)
        self.modelView.setSelectionMode(QtWidgets.QAbstractItemView.MultiSelection)
        self.modelView.setObjectName("modelView")  
        self.verticalLayout_2.addWidget(self.modelView)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.removeFactButton = QtWidgets.QPushButton(self.layoutWidget1)
        self.removeFactButton.setObjectName("removeFactButton")
        self.horizontalLayout.addWidget(self.removeFactButton)
        self.addFactButton = QtWidgets.QPushButton(self.layoutWidget1)
        self.addFactButton.setObjectName("addFactButton")
        self.horizontalLayout.addWidget(self.addFactButton)
        self.factNameComboBox = QtWidgets.QComboBox(self.layoutWidget1)
        self.factNameComboBox.setObjectName("factNameComboBox")
        self.horizontalLayout.addWidget(self.factNameComboBox)
        self.factComboBox = QtWidgets.QComboBox(self.layoutWidget1)
        self.factComboBox.setObjectName("factComboBox")
        self.horizontalLayout.addWidget(self.factComboBox)
        spacerItem2 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem2)
        self.verticalLayout_2.addLayout(self.horizontalLayout)
        self.verticalLayoutWidget = QtWidgets.QWidget(self.splitter)
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.label_4 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_4.setObjectName("label_4")
        self.verticalLayout_3.addWidget(self.label_4)
        self.instanceView = QtWidgets.QTreeWidget(self.verticalLayoutWidget)
        self.instanceView.setAcceptDrops(False)
        self.instanceView.setAutoFillBackground(False)
        self.instanceView.setHeaderHidden(False)
        self.instanceView.setObjectName("instanceView")
        self.instanceView.header().setVisible(True)
        self.instanceView.setColumnCount(2)
        self.instanceView.setHeaderLabels(["Name", "Param", "Group", "Planner"])
        self.instanceView.setColumnWidth(0, 150)
        self.instanceView.setColumnWidth(1, 400)
        self.verticalLayout_3.addWidget(self.instanceView)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.addInstanceButton = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.addInstanceButton.setObjectName("addInstanceButton")
        self.horizontalLayout_4.addWidget(self.addInstanceButton)
        self.label_5 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_5.setObjectName("label_5")
        self.horizontalLayout_4.addWidget(self.label_5)
        self.typeComboBox = QtWidgets.QComboBox(self.verticalLayoutWidget)
        self.typeComboBox.setObjectName("typeComboBox")
        self.horizontalLayout_4.addWidget(self.typeComboBox)
        self.label_6 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_6.setObjectName("label_6")
        self.horizontalLayout_4.addWidget(self.label_6)
        self.instanceNameEdit = QtWidgets.QLineEdit(self.verticalLayoutWidget)
        self.instanceNameEdit.setObjectName("instanceNameEdit")
        self.horizontalLayout_4.addWidget(self.instanceNameEdit)
        spacerItem3 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem3)
        self.verticalLayout_3.addLayout(self.horizontalLayout_4)
        self.gridLayout.addWidget(self.splitter, 1, 0, 1, 1)

        self.retranslateUi(SocialrobotActionViewer)
        QtCore.QMetaObject.connectSlotsByName(SocialrobotActionViewer)

         # set view colors
        palette = self.planView.palette()
        palette.setBrush(QPalette.Base, QBrush())
        self.planView.setPalette(palette)

        palette = self.goalView.palette()
        palette.setBrush(QPalette.Base, QBrush())
        self.goalView.setPalette(palette)

        self.instanceView.setAlternatingRowColors(True)
        self.instanceView.setStyleSheet("alternate-background-color: #ffe394;background-color: #ffedbb;")

        self.modelView.setAlternatingRowColors(True)
        self.modelView.setStyleSheet("alternate-background-color: #c7d0ff;background-color: #d0e1ff;")

        # topic / service
        self.th = Thread(self.planView)
        self.th.start()

        # button event
        self.loadButton.clicked.connect(self.execute_load)
        self.planButton.clicked.connect(self.execute_plan)
        self.clearButton.clicked.connect(self.execute_clear)

        # parameters
        self.action_list = []

    def execute_clear(self):
        self.planView.clear()
        self.modelView.clear()
        self.goalView.clear()
        self.instanceView.clear()
        return

    def execute_plan(self):
        self.planView.clear()

        self.statusLabel.setText("CONNECTING")
        rospy.wait_for_service('/task_plan/get_action_sequence')
        response = GetActionSeqResponse()

        items = []
        for index in xrange(self.modelView.count()):
            items.append(self.goalView.item(index))

        if items:
            try:
                requestActions = rospy.ServiceProxy('/socialrobot_interface/task/get_plan', socialrobot_interface.srv.Task) 
                requestPrimitives = rospy.ServiceProxy('/socialrobot_interface/actionlib/decode_plan', socialrobot_interface.srv.Actionlib)
                request = ""

                # get compound action
                req = socialrobot_interface.srv.TaskRequest()
                req.command = "GET"
                req.target = "PLAN"

                plan = []
                res = requestActions(req)
                for act in res.action_sequence:
                    act_list = act.split(" ")
                    action = Action()
                    action.name = act_list[0]
                    for param in act_list[1:]:
                        action.parameters.append(param)
                    plan.append(action)
                
                # get primitive action
                req = socialrobot_interface.srv.ActionlibRequest()
                req.inputs = "GET"
                req.action = plan 
                res = requestPrimitives(req)   

                global primitive_plan
                primitive_plan = res.action

                for idx, action in enumerate(res.action):
                    act_list = []
                    act_list.append(action.name)
                    parent = self.add_plan_tree(idx, action.name, '  '.join(action.parameters),'','','')

            except Exception as e:        
                self.statusLabel.setText("ERROR")        
            self.statusLabel.setText("SUCCESS")
        else:
            self.statusLabel.setText("ERROR")   


    def execute_load(self):
        # load fact and goal
        self.execute_clear()
        self.add_fact_goal()

        # # service list
        # srv_action_list = rospy.ServiceProxy('/actionlib/get_action_list', GetActionList)
        # srv_action_info = rospy.ServiceProxy('/actionlib/get_action_info', GetActionInfo)

        # # get all actions
        # get_action_req = GetActionListRequest()
        # get_action_req.action_type = GetActionListRequest().ALL
        # get_action_res = srv_action_list(get_action_req)
        # action_list = get_action_res.actions
        
        # # set robot hardware list
        # self.initDomain(['Arm', 'Gripper'])

        # # get available actions
        # get_action_req.action_type = GetActionListRequest().AVAILABLE_ACTIONS
        # get_action_res = srv_action_list(get_action_req)
        # action_list = get_action_res.actions

        # for act in action_list:
        #     self.action_list.append(act.data)
        #     req = GetActionInfoRequest()
        #     req.action_name = act.data
        #     action_info = srv_action_info(req)
        #     parent = self.add_tree_root(act.data, '  '.join(action_info.parameters), ' '.join(action_info.group), ' '.join(action_info.planner))
        #     #for child in d['objects']:
        #     #    self.add_tree_child(parent, *child)
        
        # #
        # self.statusLabel.setText("DONE")

    def initDomain(self, hardware_list):

        # service list
        get_domain_srv = rospy.ServiceProxy('/actionlib/get_domain', GetDomain)

        # set the robot hardware domain
        get_domain_req = GetDomainRequest()
        get_domain_req.group_list = hardware_list

        # deserialize the data
        domain_info = get_domain_srv(get_domain_req)
        domain_info.types = pickle.loads(domain_info.types)
        domain_info.actions = pickle.loads(domain_info.actions)
        domain_info.predicates = pickle.loads(domain_info.predicates)

        return domain_info

    def add_plan_tree(self, action_idx, action_name, action_arg, dispatch_time='', duration='', status=''):
        item = QtWidgets.QTreeWidgetItem(self.planView)
        item.setText(0, str(action_idx))
        item.setText(1, action_name)
        item.setText(2, action_arg)
        item.setText(3, dispatch_time)
        item.setText(4, duration)
        item.setText(5, status)
        return item

    def add_plan_child(self, parent, action_idx, action_name, action_arg, dispatch_time='', duration='', status=''):
        item = QtWidgets.QTreeWidgetItem()
        item.setText(0, str(action_idx))
        item.setText(1, action_name)
        item.setText(2, action_arg)
        item.setText(3, dispatch_time)
        item.setText(4, duration)
        item.setText(5, status)
        parent.addChild(item)
        return item

    def add_tree_root(self, name, param, group, planner):
        item = QtWidgets.QTreeWidgetItem(self.instanceView)
        item.setText(0, name)
        item.setText(1, param)
        item.setText(2, group)
        item.setText(3, planner)
        return item

    def add_tree_child(self, parent, name, param, group, planner):
        item = QtWidgets.QTreeWidgetItem()
        item.setText(0, name)
        item.setText(1, param)
        item.setText(2, group)
        item.setText(3, planner)
        parent.addChild(item)
        return item

    def retranslateUi(self, SocialrobotActionViewer):
        _translate = QtCore.QCoreApplication.translate
        SocialrobotActionViewer.setWindowTitle(_translate("SocialrobotActionViewer", "Action Viewer"))
        self.planButton.setText(_translate("SocialrobotActionViewer", "Plan"))
        self.loadButton.setText(_translate("SocialrobotActionViewer", "Load"))
        self.clearButton.setText(_translate("SocialrobotActionViewer", "Clear"))
        self.label.setText(_translate("SocialrobotActionViewer", "System Status"))
        self.planView.headerItem().setText(0, _translate("SocialrobotActionViewer", "action_id"))
        self.planView.headerItem().setText(1, _translate("SocialrobotActionViewer", "action_name"))
        self.planView.headerItem().setText(2, _translate("SocialrobotActionViewer", "action_arg"))
        self.planView.headerItem().setText(3, _translate("SocialrobotActionViewer", "dispatch_time"))
        self.planView.headerItem().setText(4, _translate("SocialrobotActionViewer", "duration"))
        self.planView.headerItem().setText(5, _translate("SocialrobotActionViewer", "status"))
        self.planView.setColumnWidth(0, 10)
        self.planView.setColumnWidth(1, 150)
        self.planView.setColumnWidth(2, 350)
        self.planView.setColumnWidth(3, 100)
        self.planView.setColumnWidth(4, 100)
        self.planView.setColumnWidth(5, 50)
        self.label_2.setText(_translate("SocialrobotActionViewer", "Goal State"))
        self.goalView.setSortingEnabled(True)
        self.removeGoalButton.setText(_translate("SocialrobotActionViewer", "Remove Selected Goals"))
        self.addGoalButton.setText(_translate("SocialrobotActionViewer", "Add Goal"))
        self.label_3.setText(_translate("SocialrobotActionViewer", "Initial State"))
        self.removeFactButton.setText(_translate("SocialrobotActionViewer", "Remove Selected Facts"))
        self.addFactButton.setText(_translate("SocialrobotActionViewer", "Add Fact"))
        self.label_4.setText(_translate("SocialrobotActionViewer", "Action list"))
        self.addInstanceButton.setText(_translate("SocialrobotActionViewer", "Add Instance"))
        self.label_5.setText(_translate("SocialrobotActionViewer", "Type:"))
        self.label_6.setText(_translate("SocialrobotActionViewer", "Name:"))

    def add_fact_goal(self):            
        fact_widget = self.modelView
        goal_widget = self.goalView

        # get problems from knowledge interface
        problem_srv = rospy.ServiceProxy('/socialrobot_interface/knowledge/get_problem', socialrobot_interface.srv.Knowledge)
        self.statusLabel.setText("WAITING") 
        rospy.wait_for_service('/socialrobot_interface/knowledge/get_problem')

        get_problem_req = socialrobot_interface.srv.KnowledgeRequest()
        get_problem_req.command = ""
        
        problem_info = problem_srv(get_problem_req).problem

        # facts
        for f in problem_info.facts:
            self.add_predicate(fact_widget, f.name, f.args, f.is_negative)

        # goals
        for g in problem_info.goals:
            self.add_predicate(goal_widget, g.name, g.args, g.is_negative)

    def add_predicate(self, list_widget, name, arg, is_negative):
        if is_negative:
            pred = '(not ('+ name + ' ' + ' '.join(arg)+'))'
        else:
            pred = '('+ name + ' ' + ' '.join(arg)+')'
        item = QtWidgets.QListWidgetItem()
        item.setText(pred)
        list_widget.addItem(item)


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    SocialrobotActionViewer = QtWidgets.QWidget()
    ui = Ui_SocialrobotActionViewer()
    ui.setupUi(SocialrobotActionViewer)
    SocialrobotActionViewer.show()
    sys.exit(app.exec_())

