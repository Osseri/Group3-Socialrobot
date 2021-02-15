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
from socialrobot_actionlib.srv import *
from socialrobot_actionlib.msg import *
import socialrobot_interface.srv
from socialrobot_interface.msg import *

primitive_plan = ''

class Thread(QThread):

    change_value = 0
    def __init__(self, tree_widgets):
        QThread.__init__(self)
        self.cond = QWaitCondition()
        self.mutex = QMutex()

        rospy.init_node('task_state_viewer')
        rospy.Subscriber('/socialrobot/system/feedback', SystemActionFeedback, self._feedback_callback)
        
        self.idx = 0
        self.action = ''
        self.plan_tree, self.goal_tree, self.fact_tree, self.action_tree = tree_widgets
        self.action_states = None

    def __del__(self):
        self.wait()

    def _feedback_callback(self, data):

        self.action_states = data.feedback

    def update_view(self):
        self.planView.clear()        
        # for idx, action in enumerate(primitive_plan):
        #     act_list = []
        #     act_list.append(action.name)
        #     parent = self.add_plan_tree(idx, action.name, '  '.join(action.parameters),'','','')

        return

    def add_plan_tree(self, action_idx, action_name, action_arg, dispatch_time='', duration='', status=''):
        item = QtWidgets.QTreeWidgetItem(self.plan_tree)
        item.setText(0, str(action_idx))
        item.setText(1, action_name)
        item.setText(2, action_arg)
        item.setText(3, dispatch_time)
        item.setText(4, duration)
        item.setText(5, status)
        return item

    def add_fact_tree(self, predicate):
        facts = self.action_states.facts
        for fact in facts:
            if fact.is_negative:
                pred = '(not ('+ fact.name + ' ' + ' '.join(fact.args)+'))'
            else:
                pred = '('+ fact.name + ' ' + ' '.join(fact.arg)+')'
            item = QtWidgets.QListWidgetItem()
            item.setText(pred)
            self.fact_tree.addItem(item)


    def add_action_tree(self, name, param, group, planner):
        item = QtWidgets.QListWidgetItem(self.action_tree)
        item.setText(0, name)    
        item.setText(1, param)       
        item.setText(2, group)         
        item.setText(3, planner)     
        return item

    def run(self):
        while True:
            self.mutex.lock()
            # clear
            self.fact_tree.clear()

            # update tree widgets
            if self.action_states:
                self.fact_tree('pred')
     
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
        self.factView = QtWidgets.QListWidget(self.layoutWidget1)
        self.factView.setEnabled(True)
        self.factView.setSelectionMode(QtWidgets.QAbstractItemView.MultiSelection)
        self.factView.setObjectName("modelView")  
        self.verticalLayout_2.addWidget(self.factView)
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
        self.actionView = QtWidgets.QTreeWidget(self.verticalLayoutWidget)
        self.actionView.setAcceptDrops(False)
        self.actionView.setAutoFillBackground(False)
        self.actionView.setHeaderHidden(False)
        self.actionView.setObjectName("instanceView")
        self.actionView.header().setVisible(True)
        self.actionView.setColumnCount(2)
        self.actionView.setHeaderLabels(["Name", "Param", "Group", "Planner"])
        self.actionView.setColumnWidth(0, 150)
        self.actionView.setColumnWidth(1, 400)
        self.verticalLayout_3.addWidget(self.actionView)
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

        self.factView.setAlternatingRowColors(True)
        self.factView.setStyleSheet("alternate-background-color: #c7d0ff;background-color: #d0e1ff;")

        self.actionView.setAlternatingRowColors(True)
        self.actionView.setStyleSheet("alternate-background-color: #ffe394;background-color: #ffedbb;")

        # topic / service
        tree_widgets = [self.planView, self.goalView, self.factView, self.actionView]
        self.th = Thread(tree_widgets)
        self.th.start()

        # button event
        self.loadButton.clicked.connect(self.execute_load)
        self.planButton.clicked.connect(self.execute_plan)
        self.clearButton.clicked.connect(self.execute_clear)

    def execute_clear(self):
        self.planView.clear()
        self.factView.clear()
        self.goalView.clear()
        self.actionView.clear()
        return

    def execute_plan(self):
        self.planView.clear()

        self.statusLabel.setText("CONNECTING") 
        self.statusLabel.setText("SUCCESS")
        self.statusLabel.setText("ERROR")   

    def execute_load(self):
        self.execute_clear()
        print(self.th.action_states)

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
        item = QtWidgets.QTreeWidgetItem(self.actionView)
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
        fact_widget = self.factView
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

