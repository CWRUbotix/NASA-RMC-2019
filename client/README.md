# Development for Manual Control Client

## Running it

* Refer to the root README for instructions.

## Goals

* Provide a means to control the robot, through its onboard computer, providing messages to HCI (Hardware Control Interface)

* Provide information about robot sensor values to a given user

## Architecture Overview

* The libraries:

  * The Client relies on PyQt for front-end layout and functionlity, with rospy for ROS specific features
  * It should be noted that PyQt is accessed through the 'python_qt_binding' namespace, which is intended to allow for development agnostic of PySide (the official bindings) vs PyQt (the open-source alternative). See the ROS tutorial page for more information as well as how to get started.

* `src/client/robotinterface.py`
  * A **very very important** component of the client that handles the specifics for how we decided to communicate with ROS. One may notice that 'rosservice' is imported here and there -- by keeping most of the ROS-specific things here, the impact of moving away from one communication model was minimized to the GUI itself.

* `src/client/my_module.py`
  
  * This is the bulk of the code, and by that, I mean where literally 95% of the functionality you see when running rqt_gui > plugins > "the plugin name" is implemented.

  * `def __init__(self, context):`
    * This is where hooking up all the signals, class variables, and other things happen. If you want to make a button do something, it has to go through something like `self._widget.BUTTON_NAME_HERE`
    * Portions of the code remain from following the tutorial(s) on creating a plugin for `rqt_gui`. It is not advised to modify them without understanding what they do.

  * **_The functions that come after_**
    * Also quite important are all the functions that come after `__init__`. Most of their naming is self-explanatory, though perhaps not coded exceptionally well. Following are brief explainations of the most important ones:
    * `keyPressEvent` captures key presses made while under focus of the main 'form', buttons directly associated with it that regain focus, or some of the frames.
      * It is *very* important to note that as currently implemented, a user **MUST** regain focus, after using the majority of the spinboxes, by clicking in one of the frames/buttons. The WASD/E controls will otherwise feed into the spinboxes
      * The large series of `if/elif` switch between self-describing ENUMs or something. Find these by looking up the `PyQt` docmentation, if your autocomplete ~~sucks~~ is inadequate like on VSCode.
    * `send_spinbox_value` is a simple function that can include debugging messaging when sending a value using robotinterface. As developed, the client is not consistent with using it
    * `w, a, s, d _pressed` are referenced in the init to provide the message sending a driver expects
    * `estop_pressed`: It should be noted that this 'e-stop' as implemented is not a true 'emergency stop' in the electronic/hardware sense. 
      * This mechanism relies on the robot's computer running and working (not shutting down/restarting a la Alabama 2019) to command relevant motors to 0 speed (as determined beforehand by the init's list)
      * Furthermore, some motors such as those for translation and belt attitude control position. These send the last known positions as recieved by robotinterface
      * Timers for translation are stopped
        * TODO: the timer for attitude shift should also be stopped
    * `motor 0..7 spinbox_changed`: Sends the value when the spinbox changes at all. Note, it is probably more convenient to only send a given position when the user is done editing, but this feature was not added in time.
      * Example: User wants to edit a value from 0 to 45. Their entry may look like this: 0 > 04 > 45 or 0 > 40 > 4 > 45
    * `timer start/update functions`: As implemented, most of the timers are designed to fire off a specified function for some specified interval, per 'tick'. Timer set-up is done in the start function, setting the QTimer reference specified in `__init__` to a timer function specified later
    * `try_update_sensor`: Takes the form of a function called by a timer. The rate of updating is simply whatever the HCI guy/robotInterface guy decided to do.

* `src/robotInterface.py`
  * If you find any other robotInterfaces sitting around, you should probably delete them and push after checking whether they were used or not. This one is *probably* the real one.
  * I'm not gonna babble too much, the functions do as they say, assuming you have your `ROS_MASTER_URI` and `ROS_URI` set, as well as the robot __connected to the same network and on__.
  * It's advised to read the *rospy/ROS* docs before messing with this too much. It works well with Glennifer last I checked.


Author | Info | Date
--- | --- | ---
Steven Xie | Initial information filled out | 11 May 2019
