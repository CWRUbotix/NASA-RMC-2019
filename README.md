# NASA-RMC-2019

## Manual Control Client
### Prerequisites
- A computer running Ubuntu 16.04 LTS, with ROS properly installed
- **MAKE SURE YOUR .BASHRC SOURCES:**
 * source /opt/ros/kinetic/setup.bash
 * source ~/catkin_ws/devel/setup.bash
 * and for ease of use, something like: 
    * export ROS_PACKAGE_PATH=/home/YOUR USERNAME HERE/catkin_ws/src:$ROS_PACKAGE_PATH
 
### How to run the client
- First, make sure that you've pulled the latest stable branch into your catkin_ws or wherever ROS can run/find things
- run catkin_make from the /catkin_ws/ level
 - important note: if you see errors such as something about 'serial' missing:
 - at the catkin_ws/src level, clone http://wjwwood.github.com/serial/ and try again
- You can then do two things if catkin_make ran successfully
- To test/develop on the client
    * Open a new terminal, spin up a new ros master
    * enter: rosrun rqt_gui rqt_gui on your original terminal
    * from the 'plugins' taskbar at the top, run the client plugin
    
- To connect to the robot:
    * Make sure the ROBOT is connected to the network you'd like to use
    * On the ROBOT computer, use 'rosrun hci hci' to start the hardware control interface
    * Connect the CLIENT computer to the same wireless network
    * On the CLIENT computer:
    * Find the URI the ROBOT computer is using, export this like:
        * export ROS_MASTER_URI=http://192.168.0.100:11311
    * Similarly, export the ROS_IP that your computer has, located under 'connection info'
        * export ROS_IP=192.168.0.105
- Finally, try to 'rosrun rqt_gui rqt_gui' and start the plugin.
- Ensuring the robot is e-stopped, if debugging is enabled, you should observe changing values on the robot when WASD inputs are sent.


