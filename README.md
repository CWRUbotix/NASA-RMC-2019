# NASA-RMC-2019

## Manual Control Client

### Considerations

* If you're a mech person who wants to be able to control the robot, closely follow the prerequisites.

### Prerequisites

* A computer running `Ubuntu 16.04 LTS`
* `ROS Kinetic` properly installed
  * If you don't know what either of the previous items are, please consult Google or a knowledgeable software person
* For ease of development, one can add the following lines to the .bashrc
* `source /opt/ros/kinetic/setup.bash`
* `source ~/catkin_ws/devel/setup.bash`
* and for ease of use, something like: 
  * `export ROS_PACKAGE_PATH=/home/YOUR USERNAME HERE/catkin_ws/src:$ROS_PACKAGE_PATH`
 
### How to run the client

* First, make sure that you've pulled the latest stable branch, typically `master` into your catkin_ws or wherever ROS can run/find things
* run `catkin_make` from the `/catkin_ws/` level
  * important note: if you see errors such as something about `serial` missing:
  * at the catkin_ws/src level, clone `http://wjwwood.github.com/serial/` and try again
* You can then do two things if catkin_make ran successfully

* __To test/develop on the client__:
  * Open a new terminal, spin up a new ros master, entering `roscore`
  * enter: `rosrun rqt_gui rqt_gui` on your original terminal
  * from the `plugins` taskbar button thing at the top, run the client plugin
  * If you want, open up a new terminal and run `hci` to observe motor commands possibly scrolling across

* __To connect to the robot__:
  * Make sure the ROBOT is connected to the network you'd like to use
  * On the ROBOT computer, use `rosrun hci hci` to start the hardware control interface
  * Connect the CLIENT computer to the same wireless network
    * On the CLIENT computer:
    * Find the URI the ROBOT computer is using, export this like:
      * export ROS_MASTER_URI=http://192.168.0.100:11311
    * Similarly, export the ROS_IP that your computer has, located under 'connection info'
      * `export ROS_IP=192.168.0.105`
* Finally, try to run `rosrun rqt_gui rqt_gui` and start the plugin.
* Ensuring the robot is e-stopped, if debugging is enabled, you should observe changing values on the robot when WASD inputs are sent. Otherwise, the robot should move.


Author | Info | Date
--- | --- | ---
Steven Xie | I learned some markdown today. | 11 May 2019
