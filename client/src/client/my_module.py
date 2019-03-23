import os

from geometry_msgs.msg import Twist

import rospy
import rospkg
import sys

#setattr(sys, 'SELECT_QT_BINDING', 'pyside')

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtWidgets import QWidget

# --------------------------------------------------
#
# Heavily referenced from rosviz - rqt_robot_steering
# For CWRUbotix
#
# Manual Control Subteam
# ---------------------------------------------------

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

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

        # ROS Publisher
        self._publisher = None

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.

        ### I don't know what the hell I'm doing

        #self._widget.button.clicked.connect(self.hello)
        print("Widget info", self._widget)

        print("More widget info", self._widget.speed_label.text())
        
        # Hook things up
        self._widget.vertical_add_button.pressed.connect(self.increase_linear_speed_pressed)
        self._widget.vertical_subtract_button.pressed.connect(self.decrease_linear_speed_pressed)

        # ROS Connection Fields
        self._widget.topic_line_edit.textChanged.connect(self._on_topic_changed)

        ###

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # ROS Twist Stuff

        # timer to consecutively send twist messages
        self._update_parameter_timer = QTimer(self)
        self._update_parameter_timer.timeout.connect(
            self._on_parameter_changed)
        self._update_parameter_timer.start(100)
        self.zero_cmd_sent = False


    # ROS Connection things
    @Slot(str)
    def _on_topic_changed(self, topic):
        topic = str(topic)
        self._unregister_publisher()
        print("trying topic: ", topic)
        if topic == '':
            return
        try:
            self._publisher = rospy.Publisher(topic, Twist, queue_size = 10)
        except TypeError:
            self._publisher = rospy.Publisher(topic, Twist)

    ##### Unregister ROS publisher
    def _unregister_publisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None


    #### Speed and Angle change Functions
    def speed_linear_changed(self):
        self._widget.speed_label.setText(self._widget.verticalSpeedSlider.text())
        # Publish the updated speed
        self._on_parameter_changed()
        print("Changed linear speed")

    def increase_linear_speed_pressed(self):
        self._widget.speed_label.setText(
            str(int(self._widget.speed_label.text()) + 1))
        print("Increasing linear speed")

    def decrease_linear_speed_pressed(self):
        self._widget.speed_label.setText(
            str(int(self._widget.speed_label.text()) - 1))
        print("Decreasing linear speed")

    #def azimuth_changed(self):
    #    self._widget.azimuth_label.setText(self._widget.)

    #### Sending messages
    def _on_parameter_changed(self):
        self._send_twist(
            int(self._widget.speed_label.text()),
            int(self._widget.azimuth_label.text())
        )

    def _send_twist(self, x_linear, z_angular):
        if self._publisher is None:
            return
        twist = Twist()
        twist.linear.x = x_linear
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = z_angular

        # Only send the zero command once so other devices can take control
        if x_linear == 0 and z_angular == 0:
            if not self.zero_cmd_sent:
                self.zero_cmd_sent = True
                self._publisher.publish(twist)
        else:
            self.zero_cmd_sent = False
            print(twist)
            self._publisher.publish(twist)

### Specific messages to test labels
    def hello(self):
        print("Hello world!")

###


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self._update_parameter_timer.stop()
        self._unregister_publisher()

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
