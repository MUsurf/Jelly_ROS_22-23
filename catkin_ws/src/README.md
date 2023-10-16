# Jelly_ROS_22-23
Code to run Jelly for competition
Establish Jetson connection
Start ROS
Run roscore
enter another terminal session
You can see what nodes that are being run at the moment by running:
rosnode list
You can similarly see the topic that are being published to, run:
rostopic list
Additionally you can see what is being published to a topic by running:
rostopic echo /topic_name
You can graph the data by running
rqt_graph



Sensor Hardware info:
Ms5837 depth sensor

General Running ROS Commands
First establish a Jetson connection
Next, get ROS running by typing  into the terminal
	To run any ROS command, another terminal session has to be opened
You can now run a topic by typing and replace the 
	package with the node package name and the executable with the node
	executable (see below for the names for each node)
Once a topic (or topics) are running, you can see which nodes are running two ways:


Once you see that a node is successfully running, then you can see what the node 
	displaying . The node name will be exactly what is listed
	when  OR  is ran (including the /rov/(etc.))
After making changes to a src files (Especially C++ file) use the 
command


ms5837 Notes
Command to run the depth sensor 
Information about the node name can be found in /ms5837/src/README.md

Imu_bno055 Notes
Command to run pkg: roslaunch imu_bno055 imu.launch (RECOMMENDED)
OR rosrun imu_bno055 bno055_i2c_node*

How to change the Bus number:
Go to ros-imu-bno055/launch/imu.launch
On the line where it says: 
<param name="device" type="string" value="/dev/i2c-8"/>
Change ‘value’ accordingly. On the Jetson Xavier this number will likely be /dev/i2c-1 or /dev/i2c-8

How to change the address:
Go to ros-imu-bno055/launch/imu.launch
On the line where it says:
<param name="address" type="int" value="40"/> <!-- 0x28 == 40 is the default for BNO055 -->
Change ‘value’ accordingly. The adafruit bno055 imu should always have a hex address of 0x28. Since roslaunch supports int types instead of hex, you will need to input the decimal equivalent of the hex address. In this case, the decimal version of 0x28 is 40.

*
Note about rosrun command:
If you do rosrun and not roslaunch it will not touch the imu.launch file and will not have access to any parameters defined there, such as the device parameter. In line 69 of the bno055_i2c_node.cpp file:
nh_priv->param("device", param_device, (std::string)"/dev/i2c-8");
This line will attempt to get the parameter from the imu.launch file. Since the roslaunch command was not used, it will instead take the last parameter of this function as a default. In this case, that default is “/dev/i2c-8”. You can change this default to get it to run, however, this change will not be taken until you stop the node and roscore and rebuild the whole package with catkin_make. For this reason, I HIGHLY recommend you use the roslaunch command with the imu.launch file to run this node. This is how jelly_stab.launch will launch this node as well.
