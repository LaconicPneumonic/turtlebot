#How to Setup & Tame A Turtlebot

Wrriten By: Anthony Rolland | @laconicpneumonic | rolland@mit.edu

_The following instructions assume you are installing on a computer with Ubuntu Trusty_

##Installation On Turtlebot Computer
Follow the instructions [__HERE__] (http://wiki.ros.org/indigo/Installation/Ubuntu) to install ROS Indigo

In addition, you will need to install the following debs for TurtleBot (please update this if you find any errors):
```bash
> sudo apt-get install ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions 
ros-indigo-turtlebot-simulator ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon ros-indigo-rocon-qt-library 
ros-indigo-ar-track-alvar-msgs
```

##Post-Installation

###Create the appropriate environment variables for the Kobuki Base
```bash
> . /opt/ros/indigo/setup.bash 
> rosrun kobuki_ftdi create_udev_rules
```


##Installation On Remote Computer
Install Ubuntu Trusty
Install ROS Indigo [__HERE__] (http://wiki.ros.org/indigo/Installation/Ubuntu)
###Kinect
Download the proper driver with this command
```bash
> sudo apt-get install ros-indigo-freenect-launch
```

Create the "/camera" topic with the following
```bash
> roslaunch freenect_launch freenect.launch
```

##Network Configuration
Use __ifconfig__ to determine the IP Adresses of both the TurtleBot and Remote computers

Confirm connectivity by pinging from Turtlebot to Remote and vice versa

```bash
> sudo apt-get install openssh-server
> ssh turtle@<TURTLEBOTP_IP>
```
Replace turtle in the command above with the username you created when installing the turtlebot software, and replace <TURTLEBOT_IP> with the hostname or IP address of Turtlebot.


You should export the variables inside your Turtlebot work space setup script.

```bash
> echo export ROS_MASTER_URI=http://localhost:11311 >> ~/.bashrc
> echo export ROS_HOSTNAME=IP_OF_TURTLEBOT >> ~/.bashrc
```

You should export the variables inside your Remote workspace setup script. Note that the meaning of the ROS_MASTER_URI changes here - the master is in the turtlebot!
```bash
> echo export ROS_MASTER_URI=http://IP_OF_TURTLEBOT:11311 >> ~/.bashrc
> echo export ROS_HOSTNAME=IP_OF_PC >> ~/.bashrc
```

###Verify from Remote PC to TurtleBot
Open a new command line terminal on remote pc and run:

```bash
> rostopic list
```
If you don't see list of topics check the value of ROS_MASTER_URI.

On remote pc run:

```bash
> rostopic echo /diagnostics
```
If you don't get a warning that topic has not been published, then verify that ROS_HOSTNAME is set correctly on the TurtleBot laptop.

###Verify from TurtleBot to Remote PC
Finally, check that TurtleBot laptop can get data from ROS node running on remote pc.

On remote pc run:

```bash
> rostopic pub -r10 /hello std_msgs/String "hello"
```
On TurtleBot run

```bash
> rostopic echo /hello
```
The message "hello" begin printed about 10 times a second. If not, check the ROS_HOSTNAME setting on the remote pc.
