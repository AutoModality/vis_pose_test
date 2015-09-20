# vis_pose_test

This ROS package can be used to test the tracking of targets using vision systems in conjunction with the PX4 flight stack. This package assumes you are running on an offboard computer and communicating the the Pixhawk via mavros.

## Requirements

1. Pixhawk.
2. Companion computer that can be interfaced to the Pixhawk and capable of running ROS.
3. Camera with approriate ROS based vison SW running on the companion computer that can produce target poses and publish those poses on the appropriate ROS topic as described below.


## Setup and Installation

### Pixhawk
Clone the repository XXX which contains firmware for the Pixhawk.

This version of PX4 SW is a branch of version 1.0.1 PX4 stable build and includes some additional parameters and telemetry streams that can aid in the debugging of your system receiving vision poses. At some point these features may be merged into a future release of the PX4 fight stack at which time this repository will no longer be necessary.

It is not necessary to build this firmware since pre-built firmwafe versions are already included in the repository, but all the source code is available if you want to make modifications.

Load Firmware/Build/px4fmu-v2_default.build/firmware.px4 into the Pixhawk using QGround control.

### Companion Computer

Interface the companion computer to the Pixhawk using the following instructions:



- - -

1. Interface a companion computer to the PixHawk using the following instructions:
XXX
Note that it is recommended that you 

## Users Manual


