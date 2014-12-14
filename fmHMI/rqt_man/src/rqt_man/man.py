

from qt_gui.plugin import Plugin
from man_widget import ManWidget


class Man(Plugin):
    """
    Plugin to interface with webtools via ros_gui
    """
    def __init__(self, context):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        super(Man, self).__init__(context)
        self.setObjectName('Man')
	#self.context = context

	self._man = ManWidget()
        if context.serial_number() > 1:
	   self._man.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
	   # Add widget to the user interface
	context.add_widget(self._man)

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
