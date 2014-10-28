import os
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QUrl
from python_qt_binding.QtGui import QCompleter, QIcon, QWidget
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
        self.pushButtonMoveRobot.clicked.connect(self._handle_move_robot_clicked)
        self.pushButtonTpDown.clicked.connect(self._handle_tp_down_clicked)
        self.pushButtonTpUp.clicked.connect(self._handle_tp_up_clicked)

    def _handle_auto_clicked(self):
        pass

    def _handle_man_clicked(self):
        pass

    def _handle_move_robot_clicked(self):
        pass

    def _handle_tp_down_clicked(self):
        pass

    def _handle_tp_up_clicked(self):
        pass

    def save_settings(self, settings):
        pass

    def restore_settings(self, settings):
        pass

