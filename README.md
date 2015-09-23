# vis_pose_test

This ROS package can be used to test the localization of a UAS using a vision system which is capable of tracking markers or targets and doing offboard postion control in a fashion similar to what is shown in this video:

[https://vimeo.com/140210396](https://vimeo.com/140210396)

This package runs on an offboard computer running ROS and interfaces to a Pixhawk running the PX4 flight stack. All communications between the offboard computer and the Pixhawk are via mavros on the offboard computer.

If this is your first exposure to the Pixhawk or the PX4 flight stack we recommend you start with the extensive online documentation found here:

[http://www.pixhawk.org/choice](http://www.pixhawk.org/choice)
[https://pixhawk.org/dev/start](https://pixhawk.org/dev/start)

This package runs in ROS and therefore familiarity with ROS is helpful. ROS as it relates to the PX4 flight stack can be found here:

[https://pixhawk.org/dev/ros/start](https://pixhawk.org/dev/ros/start)


## Requirements

The major components used in testing this package consist of the following:

1. UAS utilizing a Pixhawk flight controller running the PX4 flight stack.
2. RC controller for flying the UAS, preferably 8 channels.
2. Companion computer that can be interfaced to the Pixhawk and capable of running ROS. It is greatly desireable if the companion computer also has a WiFi network connection for remote access through consoles.
3. Camera mounted on the UAS utilizing ROS based vison SW running on the companion computer.
4. QGroundControl installed on some computer

## Principles of Operation
This packages makes the following assumptions:
- The camera used is attached to the front of the UAS in the same orientation as the heading of the UAS.
- The target poses produced by the vison sysem are assumed to consist of three coordinates (x,y,z) which are "Front", "Left", "Up" (FLU) wrt the horizontal plane projecting out from the camera. This is consistent with a camera mounted on a 2D gimbal. If the camera is not mounted on a gimabal then the vision system should compensate for the camera attitude to create a pose in the horizontal plane.
- There is no predetermined level of accruacy in the x,y,z coordinates, but no doubt the more accurate they are the better. The aruco vision system used to test vis_pose_test was accurate to within a few centimeters.
- The heading of the camera is assumed to be fixed wrt the vehicle. If it is not fixed then the vison system should compensate for changes in the heading of the camera so that the poses it publishes are FLU wrt the UAS.
- The orientation of the target or marker is not used in vis_pose_test, but if it is available it could be exploited to make the localization more accurate with ehancements to vis_pose_test.

The vis_pose_test package consists of the following ROS nodes.

### rc_joystick
Simple node that takes the raw inputs from the Pixhawk's RC controller and publishes them on the "joy" topic.

### mavros
This node is used to proxy mavlink messages between the Pixhawk and the ROS apps. For more information see [http://wiki.ros.org/mavros](http://wiki.ros.org/mavros). The version provided with this package is an extention of version 0.14.2. It has an additional plugin that supports the new attitude control topic "setpoint_pt_attitude". This topic is simply an exetension of the existing topic "setpoint_attitude/attitude" with the throttle value included.


### vis_pose_test 

The core of this package. Used to perform localization and simple motion control based on target poses received from some vision system. This node listens for target poses on the topic "target_pose" which are sent in a so called "Front", "Left", "Up" (FLU) frame with resepct to the UAS camera frame as described above. It takes these FLU coordinates and transforms them into ENU coordinates (fixed inertial frame) with the target at the origin. In order to do these transformations it uses the vehicle's current pose which it gets by listening on the topic "local_position/local". It publishes the vehicle ENU poses to the topic "vision_pose/pose" which are sent to the FCU. This localization is performed in the file locator.cpp.

In addition to the localization described above this node also performs simple offboard control depending upon the position of the offboard control switch. It does this by listening to the topic "joy". The joy topic contains the various joy stick and switch positions of the same RC controller that is used to control the UAS. Just like the Pixhawk supports MANUAL, ALTCTL, and POSCTL this node also supports those same flight modes as implemented in the controller.cpp file. MANUAL mode takes the joystick positions and translates that into attitude control commands that it publishes on the "setpoint_pt_attitude" topic. The ALTCTL and POSCTL takes joystick positions and translates those into position setpoints that it sends on the "setpoint_position/local" topic.

### Vision Node(s)
Note that the vision node used to generate the target poses is not included in this package. Vis_pose_test was tested using aruco. In principle any vision system could be used as long as it publishes target poses on the "target_pose" topic using the FLU frame of reference as described above.
Whatever vision system is used should generate target or marker poses in the following fashion.
- Poses should be posted to the topic /target_pose. See the file locator.cpp for more info.
- /target_pose has the following data structure: geometry_msgs::PoseArray. This means that a list of poses are sent to vis_pose_test. Note that vis_pose_test only looks at the first one in the list so you should not use more than one target or marker as that may cause "hyperspacing" of the UAS location if the ordering of the targets in the lsit are not maintained.
- Each entry in the PoseArray is a geometry_msgs::Pose. Currently vis_pose_test only uses the x,y,z components of the Pose.
- The vison system should continue updating target_pose even if there are no targets and it sends an empty list. This is espcailly true if it loses the target. If the list is empty then vis_pose_test considers this an error and will take fail safe measures to drop the UAS out of offboard mode to insure that does not fly into the ceiling or crash because it has lost its localization.


## Setup and Installation
In general this system uses so called offboard control as described here: [https://pixhawk.org/dev/offboard_control](https://pixhawk.org/dev/offboard_control).

Special attention should be paid to those sections referring to the use of ROS and mavros.

The initial development and testing of vis_pose_test was done using a Jetson TK1 running Ubuntu 14.04.3. The Jetson was interfaced to the Pixhawk via a FTDI USB serial cable that was plugged into the Telemetry 2 port of the Pixhawk. More specific setup instructions are given below.

In addition vis_pose_test has also been run on other platforms including Ubuntu VM's running on a desktop computer. Such configurations are useful for doing development and testing before porting to the target platform. While not described in this document it is possible to interface these desktop VM's to the Pixhawk just like the compnaion computer. While not tested it should also be possible to run vis_pose_test using SITL and a simulation environment.

### Install QGroundControl
QGroundControl (QGC) is an operator interface to the UAS that allows you perform a wide range of operations including loading and configuring the firmware of the Pixhawk. Install it according to the instructions here:
[http://qgroundcontrol.org/](http://qgroundcontrol.org/)

### Pixhawk

#### Install Firmware
1. Clone the GitHub repository:
git clone https://github.com/AutoModality/Firmware.git -b am_stable_v1.0.1_build
This repository contains firmware for the Pixhawk. This version of PX4 SW is a branch of version 1.0.1 PX4 (stable build) and includes some additional parameters and telemetry streams that can aid in the debugging. At some point these features may be merged into a future release of the PX4 fight stack at which time this repository will no longer be necessary. It is not necessary to build this firmware since pre-built firmwafe versions are already included in the repository, but all the source code is available if you want to make modifications.

3. Load Firmware/Build/px4fmu-v2_default.build/firmware.px4 into the Pixhawk using QGroundControl (QGC). In order for the new parameters to appear in QGC you must load the firmware using QGC.
2. Use QGC to configure and set all the appropriate parameters in the Pixhawk including vehicle type etc. The file vis_pose_test/sample/sample.param file can be loaded into QGC and used as a starting point, but you almost certainly need to make modifications to suit your UAS. The sample.param file provided was used to fly indoors using an X8 frame. There will be some additional instrcutions on relevant parameters to tune the localization and position control below.

3. Use QGC to calibrate the sensors and radios. 

2. Configure the system startup files (e.g. /etc/extras.txt) as described here to suit your situation:
[https://pixhawk.org/dev/system_startup](https://pixhawk.org/dev/system_startup)
The system configured for the development of vis_pose_test did not require any edits to /etc/extras.txt


#### Configure Pixhawk Parameters
A nunmber of parameters should be set using QGC. As a starting point you can use the sample.param file described above. Use caution when loading this parameter file since if you have already made changes to your parameters as part of the above procedures then you will overwirte those changes by loading sample.param. The following discusses considerations for each of the major parameter sections that can be edited using QGC.

2. **System**
The System parameters configure the interface between the Pixhawk and the companion computer. The following parameters were used for testing vis_pose_test. You may need to use different values depending upon how you interfraced your compnaion computer to the Pixhawk
	- SYS_AUTOSTART	12001
	- SYS_COMPANION 921600
	- SYS_RESTART_TYPE 0

4. **Radio Switches**
Set up the RC radio switches to switch into various flight modes. There are the normal onboard Pixhawk flights modes (MANUAL, ALTCTL, POSCTL) and there are offboard flight modes (MANUAL, ALTCTL, POSCTL) that mirror the same onboard functionality except using offboard setpoint commands. Vis_pose_test was tested using a Spektrum DX8 with the following switch configuration. Consult the sample.param file.
	- A three position switch to switch between MANUAL, ALTCTL, and POSCTL. The radio used in the testing of vis_pose_test used the FLAPS switch on the DX8 or this purpose.
	- A two position switch used to switch between offboard and ONBOARD-CONTROL. The radio used in the testing of vis_pose_test used the GEAR switch on the DX8 for this purpose. The vis_pose_test node assumes this switch so if you diecide to use a different switch you wil need to change control.cpp appropriately to map a different switch.
	- A three position switch used to switch between OFFBOARD-MANUAL, OFFBOARD-ALTCTL, and OFFBOARD-POSCTL. The SW used in testing this package used the AUX 2 switch on the DX8 for this purpose. The vis_pose_test node assumes this switch so if you diecide o use a different switch you wil need to change control.cpp appropriately to map a differetn switch.

1. **Position Estimator INAV**
Vis_pose_test was tested using the INAV position estimator since that estimator currently supports vision based location inputs. The testing was done indoors with GPS disabled to insure the localization was to a large extent based on the vision based pose information. These parameters should be tuned to best suit your vision system. The following INAV parameters were modified from their default values.
	- INAV_W_ACC_BIAS 0.01
	- INAV_W_XY_GPS_P 0
	- INAV_W_XY_GPS_V 0
	- INAV_W_XY_VIS_P 1
	- INAV_W_Z_BARO 0.01
	- INAV_W_Z_GPS_P 0
	- INAV_W_Z_VIS_P 1
	- INAV_W_Z_VIS_V 0 (new parameter)

2. **Multicopter Positon Control**
These parameters control the postion control of the UAS and will dictate how agressively it will respond to positon setpoints. Their values are dependent upon a number of factors including vehicle dynamics and vision system performance. The parameters in smaple.param are intentionally set to give very slow overdamped responsesot setpoints. Once you have your system flying the way you like you can increase the various PID loop gains to change behavior. The following parameters were changed form their default values.
	- MPC_MANTHR_MIN 0
	- MPC_THR_MAX 1
	- MPC_THR_MIN 0
	- MPC_XY_P 0.1
	- MPC_Z_P 0.5
	- MPC_Z_VEL_P 0.15

2. **Multicopter Attitude Control**
These parameters are completely dependent upon your frame type and UAS performance and should be tuned accordingly. Do not expect the parameters in the sample.param file to work for your paltform and in fact if you have loaded the sample.param file you should as a first step change any of the parameters that are different from their defajult values back to their defaults.

1. **Attitude Estimator EKF**
A new parameter (ATT_VIS_EN) was added to this attitude estimator to diable the vision inputs use in the estimator.

### Companion Computer

This section describes setting up the companion computer.

#### Interface to Pixhawk
The companion computer used to test vis_pose_test was interface to the Pixwak using an FTDI USB serial cable connected to the TELEM 2 part of the Pixhawk as described here [https://pixhawk.org/dev/companion_link](https://pixhawk.org/dev/companion_link).

#### Set up Software
These instructons assume you are running Ubuntu on the companion computer. If not then adjust the directions accordingly.
1. Install Ubuntu
2. Install ROS Jade

(adapted from http://wiki.ros.org/jade/Installation/Ubuntu)

    $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    $ sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
    $ sudo apt-get update
    $ sudo apt-get install ros-jade-desktop
    $ sudo rosdep init
    $ rosdep update
    $ echo "source /opt/ros/jade/setup.bash" >> ~/.bashrc
    $ source ~/.bashrc
    $ sudo apt-get install python-rosinstall

1. Install mavros and mavlink

Note that the version used to test vis_pose_test uses an extension to version 0.14.2 of mavros. In addition the version of mavlink that was tested was Mavlink 2015.8.8. The extension to mavros is simply a new plugin that adds a new topic for attitude control and should be compatible with future version of mavros. The official ROS repositories take time to get upddated with newer releases and you will find that different platfroms get updated at different rates. Therefore to insure that you are getting the correct version of both mavros and mavlink that are compatible with the version of Firmware loaded into the Pixhawk we recommend that you build them from sources according to the directions below.

    $ mkdir -p ~/mav_catkin_ws/src
    $ cd ~/mav_catkin_ws/src
    $ wstool init
    $ git clone https://github.com/AutoModality/mavros -b am_master
    $ rosinstall_generator mavlink | tee /tmp/rosinstall.yaml
    $ wstool merge /tmp/rosinstall.yaml
    $ wstool up -j4
    $ cd ..
    $ catkin build
    $ sudo usermod -aG dialout (username used to log into Ubuntu)

2. Configure vis_pose_test workspace

Note that since it is unlikely that changes will be required to mavros and because it uses a different tool chain to build than the officialy released ROS tool chain we are using the concpet of overlaid packages to seperate the vis_pose_test catkin environment from the mavros environment. This approach is adapted from http://wiki.ros.org/catkin/Tutorials/workspace_overlaying.

    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/src
    $ catkin_init_workspace
    $ cd ..
    $ source ~/mav_catkin_ws/devel/setup.sh
    $ catkin_make
    $ echo "source ~/catkin_ws/devel/setup.sh" >> ~/.bashrc
    
3. Install vis_pose_test

In a new shell:

    $ cd ~/catkin_ws/src
    $ git clone https://github.com/AutoModality/vis_pose_test
    $ cd ..
    $ catkin_make

#### Using Eclipse

For those that like to use Eclipse for their development please see the following instructions.

**++Install Java++**

	$ sudo apt-get install openjdk-7-jre

**++Install Eclipse++**

Download Eclipse IDE for C/C++ Developers, Linux, 64 bit from https://eclipse.org/downloads/.

Install (assuming tarball is in ~/Downloads):

	$ cd ~
	$ tar xzvf Downloads/eclipse-cpp-mars-R-linux-gtk-x86_64.tar.gz
    
**++Configure Eclipse ROS Integration (adapted from http://wiki.ros.org/IDEs)++**

Create the Eclipse project file

	$ cd ~/catkin_ws
	$ catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles"
	$ awk -f $(rospack find mk)/eclipse.awk build/.project > build/.project_with_env && mv build/.project_with_env build/.project
	$ cd build
	$ cmake ../src -DCMAKE_BUILD_TYPE=Debug

Start Eclipse (the source command can be skipped if you already have it in your ~/.bashrc)

	$ source ~/catkin_ws/devel/setup.sh
	$ cd ~/eclipse
	$ ./eclipse

Enter "/home/ubuntu/catkin_ws" for the "Workspace" and click "OK"

Select "File->Import...->General->Existing Projects into Workspace” and click "Next"

- Select "Select root directory" and enter "/home/ubuntu/catkin_ws"
- Click "Finish"
- This creates a single project called “build"


Right click on "build" and select "properties"

- Select “c++ Include Paths and Symbols”
	- Click on the “Add External Include Path...” button
		- Enter "/usr/include /usr/include/c++/4.8" and click "OK"

	- Repeat for:
		- /usr/include/x86_64-linux-gnu
		- /usr/include/x86_64-linux-gnu/4.8/include
		- /usr/include.x86_64-linux-gpu/c++/4.8

	- Click "OK"

**++Editing Files++**

Edit files by expanding the "build->[Source directory]" in the "Project Explorer" and navigating to the file.

**++Building++**

Select "Project->Build All" or press Ctrl-B.

**++Cleaning++**

Right click on the "build" project and select "Clean Project".


## Users Manual


### Launching all the Applications
The launch files for the various ROS nodes that comprise this application can be found in "vis_pose_test/launch". In there you will also find script files for launching the various nodes as described below. You may have to make the following edits to the launch files:
- Edit mavros.launch to make sure that the fcu_url matches how you interfaced the companion computer to the Pixhawk. The supplied launch file is for a USB FTDI connectected to the TELEM 2 port.
- Edit the mavros.launch file and change the gcu_url to the IP address of he machine where you are running QGC

To run everything follow these steps.


1. Turn on the UAS and make sure it starts up properly. 
2. Power up your RC controller. Make sure the offboard switch is off (i.e. in onboard mode) on the RC controller and that both the onboard and offboard fight mode switches are set to the MANUAL position. 
2. Start QGC and connect to UDP. Note that QGC will not connect to the UAS until you run mavros.
3. At this point in time you should be able to fly the UAS without the compnaion computer.
4. Power up the companion computer.
5. Open an ssh console to the companion computer and run mavros:

	$ cd catkin_ws/src/vis_pose_test/launch
    $./lmv
    
    At this point QGC should connect to the Pixhawk and you should hear the ennuciation from QGC that it is connected. If mavros launches correctly and you get the QGC enunciation then a lot of things are working correctly. Don't proceed until this works. Note that the mavros.lauch file launches both the mavros node and the rc_controller node. If for some reason mavros fails to launch check the following:
	-     The FTDI cable is plugged into the companion computer.
	-     Check that /dev/ttyUSB0 exists and you have permissions to access it.
	-     Check to see if the RC controller is on. Sometimes the rc_joystick node will node start if it is off.
5. Set up a "target" somewhere in view of the camera on the UAS.
6. Open an ssh console to the companion computer and run your vision node(s):
It is a good idea to have some sort or console output that indicates that everythgin is running proeprly and you are in fact picking up targets or markers.
5. Open an ssh console to the companion computer and run vis_pose_test:

	$ cd catkin_ws/src/vis_pose_test/launch
    $./lvp
    
    At this point vis_pose_test shgould indicate that it is running and assuming it is getting target poses from the vision software it should be forwarding vision based locations to the Pixhawk. If for some reason you are getting watchdog timeout messages every few seconds this indicates that vis_pose_test is not getting any messages from the rc controller. Make sure it is on and connected to the Pixhawk.
    If you desire more output to see what is happening in vis_pose_test use ROS to turn on the ROS_DEBUG messages and 

### Validating Operation
Before attempting any flights you should do extensive bench testing to insure that the localization and control are working properly. The steps below will help insure things are operating properly

#### Validating Vision Based Localization
The first step to validating that the locaization is working properly is to use the QGC Analyze view to plot the following:
- LOCAL_POSIITION_NED.x
- LOCAL_POSIITION_NED.y
- LOCAL_POSIITION_NED.z
- VISION_POSITION_ESTIMATE.x
- VISION_POSITION_ESTIMATE.y
- VISION_POSITION_ESTIMATE.z

The VISION_POSITION_ESTIMATE represents the UAS poses wrt to the target that  vis_pose_test is sending to the Pixhawk and the LOCAL_POSIITION_NED is where the Pixhawk thinks it is based upon a number of inputs including the vision estimate. If everything is working properly the LOCAL_POSIITION_NED and the VISION_POSITION_ESTIMATE will correspond closely. If for some reason you can not find the VISION_POSITION_ESTIMATE in the list of telemetry streams displayed by QGC then this indicates that the Pixhawk is not receiving any VISION_POSITION_ESTIMATE updates from the companion computer. Open an ssh console on the companion comuter and use the rostopic tool to trace the message flows through the system. Make sure the proper messages are flowing from the vision system to vis_pose_test and likewise that vis_pose_test is publishing vison estimates to mavros.

Assuming you are getting plots in QGC that indicate that vision estimates are coming in and that the Pixhawk is using them to estimate it's position then try flying the UAS by hand and/or moving the target and observe how the locations being plotted in QGC change. Remember that the Pixhawk is in NED so in order to work out if the right cordinates are changing as exepcted when moving the UAS or target you must know which way the UAS is oriented. Also bear in mind that the poses from vis_pose_test are with respect to the target meaning that the target is at the origin. This can be a little couterintuitive. For example if the UAS is oriented north with the target directly in front of it then the x coordinate being plotted in QGC should be negative, i.e. the UAS is south of its local origin. Furthermore if the target moves closer to the UAS then the x coordinate will "increase", i.e. corresponding to moving in a northerly direction. Also assuming all motion is in a north south direction then x coordinate should change and y should remain relatively unchanged. Assuming the same configuration the y cooridinate should be around 0 and if you move the UAS left (target right) then the y coodinate should decrease becasue the UAS it is moving in a westerly direction from the origin. Try this same sort of motion through all the different quadrants to make sure they are changing as expected.

The next step is to validate the magnitude of the changes as the UAS moves. One way to do this is to get a long board that you can set the UAS (or target) on and mark out measurements along the board. Orient the board along one of the compass axis and moved the UAS (or target) along the board in very well defined increments and see if the corresponding axis changes as expected in QGC by that amount. When orienting the board make sure that you orient it as per what the UAS thinks is North/South/East/West by using QGC and not what some third party compass might say. It is not as important to orient things along true north as it is to orient then in a fashion that is consistent with the UAS. Do this same test along a number of different orientations North/South or East/West orientations.

As a final test put the UAS on one end of the board and the target on the other so that they are facing each other. Fix the end of the board with the target at some point (e.g. a chair that swivels) and rotate the end with the UAS around the fixed target in a circle. If you log the LOCAL_POSIITION_NED data either in QGC or in the Pixhawk logs on the SD card then you should be able to plot x against y and see a perfect circle. There are utilites for exporting Pixhawk log data to Matlab that makes this process relatively straightforward. If you repest the test with the UAS at the center and the target being rotated at the other end of the board you should get the same result.



#### Validating Offboard Control
The control loop in controller.cpp is driven by messages received on the joy topic from the rc_joystick node. Every time a message is received on the joy topic then a setpoint message is sent to the Pixhawk. If you want to change the frequency of setpoint messages then you must change the frequency of joy messages. This also means that if joy messages are not being received by  vis_pose_test then no setpoint message will be sent to the Pixhawk. This may happen if the rc controller is  not powered on or if the rc_joystick ROS node is not operating properly.
There is a watchdog timer in vis_pose_test that prints a warning to the console if setpoint messages are not been sent by vis_pose_test in a timely fashion. If you do not see watchdog timeout messages being printed on the console then setpoint messages are being published by vis_pose_test.
The Pixhawk will not go into or stay in OFFBOARD mode unless it receives setpioint messages on a regular basis. To confirm that the Pixhawk is in fact receiving the setpoint message you need only move the offboard switch on the rc controller to the OFFBOARD position. If the Pixhawk can successfully go into OFFBOARD mode then the QGC will indicate this both with an enunciation and the filght status changing to OFFBOARD.
Before arming the UAS and trying to fly, perform the following steps to validate that offboard control is working as expected.
1. Set both the onboard and the offboard flight mode switch to MANUAL. Confirm that you are not getting any watchdog timeout messages on the vis_pose_test console.
2. Move the offboard switch to the OFFBOARD position. The QGC should inidcate that the Pixhawk is in OFFBOARD mode. If it does then it is successfully receiving attitude setpoints. If the Pixhawk is not successfully receiving attitude setpoints it will not go into OFFBOARD mode and the QGC will enunciate "REJECT OFFBOARD" which it will repeat every few seconds until you move the offboard switch back to the ONBOARD position.
2. With the offboard switch still in the OFFBOARD position, briefly move the offboard flight mode switch to POSCTL and then back to the MANUAL position. Look at the QGC Analyze screen and locate the POSITION_TARGET_LOCAL_NED telemetry stream. Plot the x,y,z coordinates. This is the stream that shows position setpoint values. It will not apear until setpoints are received so if you can not find the POSITION_TARGET_LOCAL_NED this probably indicates that the Pixhawk did not receive any position setpoints.
3. In the analyze screen also plot LOCAL_POSITION_NED and VISION_POSITION_ESTIMATE x,y,z coordinates. Note that the POSITION_TARGET_LOCAL_NED stream does not get updated unless the Pixhawk is in OFFBOARD mode AND the offboard flight mode is either ALTCTL or POSCTL.
4. Move the offboard flight mode switch from MANUAL to ALTCTL. Move the throttle stick on the rc controller and watch the altitude setpoint move up and down on the QGC Analyze plots. Note that when you normally fly the UAS in MANUAL mode the throttle is in the down position, but when you switch to ALTCTL or POSCTL the throttle in the down position moves the altitude setpoint to a lower altitude. Note that this corresponds to POSITION_TARGET_LOCAL_NED.z going "up" (NED frame) in the Analyze screen of QGC. Take this opportunity to practice switching from MANUAL to ALTCTL or POSCTL mode while at the same time moving the throttle to it's neutral zero position. You will need this skill when it comes time to actually fly.
5. Move the offboard flight mode switch to POSCTL. Move the roll and pitch stick on the rc controller and watch the positions setpoints for the x and y coordinates move. Note that their values and how much they change are dependent upon the heading of the UAS. The pitch lever moves the position setpoint to be in front or in back of the vehicle depending upon whether you are pitching forward or backward. Therefore if the UAS is oriented in a North/South orientation then the POSITION_TARGET_LOCAL_NED.x coordiante will change when you move the pitch, BUT if the UAS is oriented in an EAST/WEST direction then the same pitch movements will change the POSITION_TARGET_LOCAL_NED.y coordinate. Also note that every time the offboard flight mode switch moves from MANUAL to either ALTCTL or POSCTL then the position setpoint gets reset to the current position of the vehicle (LOCAL_POSITION_NED) This is consistent with what you would see if flying using the same ONBOARD flight control modes.

### Flying
The vis_pose_test node supports the following OFFBOARD flight modes which in general work the same way as the corresponding ONBOARD filight modes.
- MANUAL - Manual control. Moving the joysticks on the controller causes the UAS to move accordingly. The vis_pose_test node sends attitude setpoints to the Pixhawk.
- ALTCTL - Altitude control. The vis_pose_test node sends position setpoints to the Pixhawk. Since it is altitude control then only the z coordinate of the Pixhawk is changed and not the x,y coordinates.
- POSCTL - Position control. The vis_pose_test node send position setpoints to the Pixhawk. All three coordinates x,y,z may be modified based upon inputs form the joy stick.

Moving the offboard flight mode switch to different positions selects between these different modes. When vis_pose_test switches to a different flight mode then it will print on the console which flight mode it is switching to.

As noted they operate the same as the ONBOARD flight modes so if you are familiar with those then you should be able to operate the UAS the same way in offboard mode. One difference is that because the UAS localization is done using the target or marker moving the target or marker while the UAS is in flight should casue the UAS to also move to maintain the same setpoint with respect to the target.

To test fly do the following steps.
1. Put the UAS offboard switch into the ONBOARD position. Put both the onboard and offboard flight mode switches into MANUAL.
1. Arm the UAS. Fly it manually briefly to make sure it is working.
2. With the UAS still armed and on the ground move the offboard switch to the OFFBOARD position and confirm that the Pixhawk goes into OFFBOARD mode. Note that we recommend that initially you never put the UAS into OFFBOARD mode unless the onboard and offboard flight mode switches are both in the MANUAL position.
3. If the UAS is successfully in OFFBOARD mode and the offboard flight mode switch is in MANUAL try to fly the UAS manually. This should fly in a simialr fashion as the ONBOARD flight MANUAL mode. Note that in this mode there is no dependency on the vision system localization. Land the UAS.
4. The next mode to test is the offboard ALTCTL fight mode. This mode does depend upon vision based localization so extra care shold be taken. The following are recomended precautions:
	- Consider tethering the UAS if you are afraid of it losing control.
	- Practice qucikly flipping offboard switch from OFFBOARD mode back into ONBOARD mode and taking back manual control of the UAS. Bear in mind that becasue of the way ALTCTL and POSCTL control works the throttle is most likely going to be in the middle position which may coresspond to a sudden change in thrust when the UAS drops back into onboard MANAUL mode.
	- Be prepared for the UAS to automatically drop out of OFFBOARD mode if something goes wrong. See the section on failsafes below for more info.
1. If you feel comfortable that you have taken the necessary precautions with the UAS then:
	- UAS on ground with both onboard and offboard flight modes set to MANUAL and the offboard switch in the ONBOARD position. arm the UAS.
	- move the offboard switch into the OFFBOARD position to switch the UAS into OFFBOARD mode
	- move the offboard flight mode switch to the ALTCTL position. Note that when you do this the throttle will be int the down position which immediately starts moving the altitude setpoint down (up in QGC). Use the skill you acquired in your testing above to move the throttle to the zero position in a timely fashion so that it does not change too much. Having the altitude setpoint move significantly in this fashion is not a danger to the UAS, but it can be disconcerting to your plots on the Analyze screen. If the altitude setpoint gets too far out of wack then simply move the offboard flight mode switch to MANUAL and then back to ALTCTL. This will reset the altitude setpoint to the UAS current location.
	- Watch the plot in QGC to move the altitude setpoint to a desired location and observe the motion of the UAS. In theory setting the altitude setpoint to 0 should bring the UAS to a level at the same height as the target. Depending upon the constants you set in the Multicopter Positon Control parameters the UAS may be more or less responsive to changes in the setpoint. Avoid the temptation to keep changing the setpoint in the desired direction until you get a better feel for the dynamics of the UAS.
	- Assuming you get liftoff and somewhat stable flight then try changing the altitude setpoint and observe the UAS behaviour. 
	- If you do not get stable flight then try adjusting the postion control paramedters. You may also have to look at your vision system performance including such factors as frames per second and latencies between image capture to whent he locations are sent to the Pixhawk. If your UAS is osciallating around some setpoint this is a classsic sign of latencies in the control loop which might be caused by latencies in the vision based localization.
1. Once you achieve stable altitude control repeat the above steps but move the offboard flight mode into POSCTL. Now you should be able to control the UAS in not only the z direction, but also the x,y.

**++FAIL SAFES++**
There are fail safes built into the system such that if vis_pose_test detects the loss of marker or target acquisition by the vision system then it stops sending setpoints to the Pixhawk. Vis_pose_test detects the loss of target acquisition when it receives an empty list from the vision system. There are currently no timers in vis_pose_test that detect when the vision_pose/pose topic has not been updated in a timely fashion although these can be easily added if desired.
When vis_pose_test does go into failsafe mode the setpoints will cease to be sent to the Pixhawk and the watchdog timeouts will be dispalyed on the screen. This will cause the Pixhawk to drop out of OFFBOARD mode and you should get the enunciation "REJECT OFFBOARD". Note that when the Pixhawk drops out of OFFBOARD mode it will try to drop into whatever flight mode indicated by the onboard flight mode switch so it is probably desireable to have it in the MANUAL position.
Once the vis_pose_test goes into failsafe mode you can not resume normal operation until you set the offboard switch to ONBOARD AND you set the offbaord flight mode switch to MANUAL.







