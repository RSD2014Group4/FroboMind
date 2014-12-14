#!/usr/bin/env python

import os
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QUrl
from python_qt_binding.QtGui import QCompleter, QIcon, QWidget, QTableWidgetItem
from python_qt_binding.QtWebKit import QWebPage, QWebView

class ManWidget(QWidget):
    def __init__(self):
        super(ManWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_man'), 'resource', 'man_widget.ui')
        loadUi(ui_file, self)
        self.setObjectName('ManWidget')

        self.pushButtonAuto.clicked.connect(self._handle_auto_clicked)
        self.pushButtonMan.clicked.connect(self._handle_man_clicked)
	self.manualMode = 'true'
	self.pushButtonMan.setStyleSheet("background-color: green")


        self.pushButtonTpRS.clicked.connect(self._handle_tp_R_S_clicked)
	
	self.pushButtonDeadman.clicked.connect(self._handle_deadman_clicked)
	self.pushButtonDeadman.setStyleSheet("color: black; background-color: red")
	self.deadmanActive = 'false'

    def _handle_deadman_clicked(self):
	if self.deadmanActive == 'false':
		self.deadmanActive = 'true'
	        self.pushButtonDeadman.setStyleSheet("color: black; background-color: green")
		self.pushButtonDeadman.setText('Deadman active')
	else:
		self.deadmanActive = 'false'
	        self.pushButtonDeadman.setStyleSheet("color: black; background-color: red")
		self.pushButtonDeadman.setText('Deadman inactive')
	
    def _handle_auto_clicked(self):
        self.pushButtonMan.setStyleSheet("background-color: light grey")
	self.pushButtonAuto.setStyleSheet("background-color: green")
	self.manualMode = 'false'

    def _handle_man_clicked(self):
        self.pushButtonAuto.setStyleSheet("background-color: light grey")
	self.pushButtonMan.setStyleSheet("background-color: green")
	self.manualMode = 'true'

    def _handle_tp_R_S_clicked(self):
        pass

    def save_settings(self, settings):
        pass

    def restore_settings(self, settings):
        pass

