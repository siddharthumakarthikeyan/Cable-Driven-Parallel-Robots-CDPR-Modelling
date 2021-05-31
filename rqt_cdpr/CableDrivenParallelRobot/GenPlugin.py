#!/usr/bin/env python
#-*- coding:utf-8 -*-
import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from PyQt5.QtWidgets import QWidget
from PyQt5.QtWidgets import QMainWindow
from python_qt_binding import *
from rqt_cdpr.saveparam import *
from rqt_cdpr.loadparam import * 
import numpy as np 

class GenPlugin(Plugin):

    def __init__(self, context):
        super(GenPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('GenPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QMainWindow()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('cdpr3'), 'src', 'cabledrivenparallerobot.ui')
	#ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'cabledrivenparallerobot.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('GenPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
	QtCore.QObject.connect(self._widget.pushButton_2, QtCore.SIGNAL("clicked()"), self.pSaveParameter)
	QtCore.QObject.connect(self._widget.pushButton_3, QtCore.SIGNAL("clicked()"), self.pLoadParameter)
 		
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
    	object = self._widget
    def pSaveParameter(object):
		SaveParameter(object)

    def pLoadParameter(object):
		LoadParameter(object)
