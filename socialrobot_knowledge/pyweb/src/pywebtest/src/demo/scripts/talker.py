#!/usr/bin/env python 

import rospy
import random
import threading
import webbrowser
from std_msgs.msg import String

import os


from selenium import webdriver
import time
import urllib
import urllib2

refreshrate=2
port = 9090# + random.randint(0, 999)
url = "file://"
url += os.path.dirname(os.path.realpath(__file__))#os.getcwd()
url += "../../../../../webtest/gui/gui.html"#.format(port)

driver = webdriver.Firefox()
driver.get(url)
driver.set_page_load_timeout(5)
#while True:
#    time.sleep(refreshrate)
#    driver.refresh()

