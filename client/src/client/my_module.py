import os

from geometry_msgs.msg import Twist
from std_msgs.msg import String

import rospy
import rospkg
import sys
import rosservice

#setattr(sys, 'SELECT_QT_BINDING', 'pyside')

from client.srv import motorCommand
from client.msg import sensorValue

import robotInterface

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtWidgets import QWidget

node_name = 'robotInterface'
motorCommandTopic = 'motorCommand'
sensorValueTopic = 'sensorValue'

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

        #motor_command = motorCommand
        #sensor_value = client.msg.sensorValue
        #print(motor_command)
        #print(sensor_value)

        # Service Proxy and Subscriber
        self._service_proxy = None
        self._subscriber = None

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('client'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        
        # Hook Qt UI Elements up
        """
        self._widget.vertical_add_button.pressed.connect(self.increase_linear_speed_pressed)
        self._widget.vertical_subtract_button.pressed.connect(self.decrease_linear_speed_pressed)
        """
        self._widget.motor0_spinbox.valueChanged.connect(self.motor0_spinbox_changed)
        self._widget.motor1_spinbox.valueChanged.connect(self.motor1_spinbox_changed)
        self._widget.motor2_spinbox.valueChanged.connect(self.motor2_spinbox_changed)
        self._widget.motor3_spinbox.valueChanged.connect(self.motor3_spinbox_changed)

        self._widget.emergency_stop_button.pressed.connect(self.estop_pressed)

        # ROS Connection Fields
        """
        self._widget.topic_line_edit.textChanged.connect(self._on_topic_changed)
        """
        self._widget.connect_button.pressed.connect(self._connect_to_topics)

        self._widget.service_name_textbox.setText(motorCommandTopic)
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
    """
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
    """

    """
    Unregister ROS publisher
    """
    def _unregister_publisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None

        if self._service_proxy is not None:
            # TODO:Doesn't actually shutdown/unregister??
            #self._service_proxy.shutdown('Shutting down service proxy...')
            self._service_proxy = None


    #### Speed and Angle change Functions
    """
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
    """

    """
    Individual Motor Change Functions
    """

    def motor0_spinbox_changed(self):
        val = int(self._widget.motor0_spinbox.value())
        print("Spinbox Motor 0 val:", val)
        resp = self._send_motor_command(0, val)
        print("Motor 0 did send successfully: ",  resp)

    def motor1_spinbox_changed(self):
        val = int(self._widget.motor1_spinbox.value())
        print("Spinbox Motor 1 val:", val)

    def motor2_spinbox_changed(self):
        val = int(self._widget.motor2_spinbox.value())
        print("Spinbox Motor 2 val:", val)

    def motor3_spinbox_changed(self):
        val = int(self._widget.motor3_spinbox.value())
        print("Spinbox Motor 3 val:", val)

    """
    Grouped Motor Control Functions
    """
    def estop_pressed(self):
        # TODO: Implement this as a loop over some list of all known motors
        # Check against the values reported in sensorValue
        print("Attempting to zero all motors...")
        self._send_motor_command(0, 0)
        self._send_motor_command(1, 0)
        self._send_motor_command(2, 0)
        self._send_motor_command(3, 0)

    """
     Sending messages
    """
    def _on_parameter_changed(self):
        #self._send_twist(
        #    int(self._widget.speed_label.text()),
        #    int(self._widget.azimuth_label.text())
        #)
        speed = int(self._widget.speed_label.text())
        azimuth = int(self._widget.azimuth_label.text())
        #print(result)

    def _connect_to_topics(self):
        self._unregister_publisher()
        print("trying topic: " + motorCommandTopic)
        alltopics = rosservice.get_service_list()
        print("Services Reported As Published")
        for t in alltopics :
            print(t)

        try:
            rospy.wait_for_service(motorCommandTopic, timeout=3)
        except (rospy.ServiceException, rospy.ROSException), e:
            print("ERROR: Timed out while waiting for service: %s" % motorCommandTopic)
            self._set_status_text("SERVICE TIMEOUT ERROR")
            return
        try:
            self._service_proxy = rospy.ServiceProxy(motorCommandTopic, motorCommand, persistent=True)
            self._set_status_text("SERVICES CONNECTED")
        except TypeError:
            # TODO: Uhhh, what do we do if this breaks
            print("Error while connecting to topic: %s" % motorCommandTopic)
            self._set_status_text("TOPIC CONNECTION ERROR")
            #self._service_proxy = rospy.ServiceProxy()

    """
    Sends a single commanded value (0-100) to a specified motor id
    """
    def _send_motor_command(self, motor_id, val):
        if self._service_proxy is None:
            return
        try:
            resp = self._service_proxy(motor_id, val)
            return resp.success
        except rospy.ServiceException as exc:
            print("motor command service didn't process request: " + str(exc))
            return False

    def _set_status_text(self, text):
        self._widget.status_label.setText(text)

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
"""
class ros_robot_interface():

    def __init__(self, motor_command, sensor_value):
        node_name = 'robotInterface'
        motorCommandTopic = 'motorCommand'
        sensorValueTopic = 'sensorValue'

        motorCommand = motor_command
        sensorValue = sensor_value

        #rospy.init_node(node_name,disable_signals=True)

        rospy.wait_for_service(motorCommandTopic)
        motorCommandPub = rospy.ServiceProxy(motorCommandTopic, motorCommand, persistent=True)

        rospy.Subscriber(sensorValueTopic,sensorValue,sensorValueCallback)

"""