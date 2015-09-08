/*
 * Controller.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: ubuntu
 */

#include "controller.h"

Controller::Controller(ros::NodeHandle* n)
{
    initROS(n);
    setTargetSetpointEqCurrent = true;
}

Controller::~Controller()
{
    // TODO Auto-generated destructor stub
}

void Controller::initROS(ros::NodeHandle *n)
{
    rosNode = n;

    // Subscribe to joystick topic
    joySub = rosNode->subscribe<sensor_msgs::Joy>("joy", 10, &Controller::joyCallback, this);

    // Advertise the attitude setpoint topic
    bbAttitudeSetpointPub = rosNode->advertise<brain_box_msgs::BBPose>("/vstate/pose/setpoint", 1000);
//        attitudeSetpointPub = rosNode->advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 1000);

    // Advertise the position setpoint topic
    positionSetpointPub = rosNode->advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1000);

    // Subscribe to service for changing the FCU mode
    setModeClient = rosNode->serviceClient<mavros::SetMode>("/mavros/set_mode");

    // Set up the control loop timer
    //        loopTimer = rosNode->createTimer(ros::Duration(1.0), &Controller::controlLoopCallback, this);
    //	loopTimer.stop();

    // Set up the watchdog timer
    watchDogTimer = rosNode->createTimer(ros::Duration(0.2), &Controller::watchDogCallback, this);
    watchDogTimer.start();

    ROS_INFO("^^^ ROS ON CONTROLLER INITIALIZED ^^^");

    return;
}

void Controller::setFCUFlightModeManual()
{
    // force the FCU to manual mode
    mavros::SetMode mode;
    mode.request.base_mode = 0;
    mode.request.custom_mode = "MANUAL";
    if (setModeClient.call(mode))
    {
        ROS_INFO("Setting FCU to manual mode");
    }
    else
    {
        ROS_ERROR("Failed to set FCU to manual mode");
    }
    return;
}

void Controller::locationError()
{
    // As a failsafe set the FCU flight mode to manual
    setFCUFlightModeManual();
}


void Controller::sendAttitudeCommand()
{
    // Port to this when we eliminate BB
    //	geometry_msgs::PoseStamped vpc;
    //	vpc.pose = vehicleState->vehicle_command_ENU.getPose();
    //	vpc.header.stamp = ros::Time::now();
    //	attitudeSetpointPub.publish(vpc);
    //	setpointCommandCnt = 0;

    brain_box_msgs::BBPose bbPose;
    bbPose.pose = vehicle_command_ENU.getPose();
    bbPose.throttle.data = vehicle_command_ENU.attitude.throttle;
    bbPose.latency.smart_pilot_stamp1 = ros::Time::now();
    bbAttitudeSetpointPub.publish(bbPose);

    lastLoopTime = ros::Time::now();

    return;
}

void Controller::sendAttitudeCommand(double roll, double pitch, double yaw, double throttle)
{
    ROS_INFO(">SEND ATT SETPOINT[R:%5.3f P:%5.3f Y:%5.3f T:%5.3f]",
             roll, pitch, yaw, throttle);

    // Norte that roll and pitch are reversed to account for difference between NED and ENU
    vehicle_command_ENU.setAttitude(pitch, roll, yaw, throttle);
    sendAttitudeCommand();

    return;
}

void Controller::vehiclePositionUpdated()
{
    if (vehicleCommandEqCurrent && locator != NULL)
    {
         // Initialize the vehicle command equal to the current position
        vehicle_command_ENU.setPose(locator->vehicle_current_ENU.getPose());

        vehicleCommandEqCurrent = false;
    }
}

void Controller::targetPositionUpdated()
{
    // Do the normal control loop
    controlLoop();
}

void Controller::sendPositionCommand()
{
	if (locator == NULL || !locator->isTargetPoseInitialized())
	{
		return;
	}
//	ROS_DEBUG("SEND POS CMD");
	geometry_msgs::PoseStamped vpc;
	vpc.pose = vehicle_command_ENU.getPose();
	vpc.header.stamp = ros::Time::now();
	positionSetpointPub.publish(vpc);
        ROS_INFO(">SEND POS SETPOINT[X:%5.3f Y:%5.3f Z:%5.3f]",
                 vpc.pose.position.x,
                 vpc.pose.position.y,
                 vpc.pose.position.z);

//	setpointCommandCnt = 0;

//##	brain_box_msgs::BBPose bbPose;
//	bbPose.header = targetPoseCurrent.header;
//	bbPose.pose = vehiclePoseCommand.pose;
//	bbPose.throttle.data = vehicleAttitudeCommand.throttle;
//	bbPose.latency = targetPoselatency;
//	bbPose.latency.smart_pilot_stamp0 = in_time;
//	bbPose.latency.smart_pilot_stamp1 = ros::Time::now();
//	bbAttitudeSetpointPub.publish(bbPose);

//	setpointCommandCnt = 0;

	lastLoopTime = ros::Time::now();

	return;
}

void Controller::setPositionCommand(double x, double y, double z)
{
//	ROS_DEBUG("++++SET POS[X:%5.3f Y:%5.3f Z:%5.3f]", x, y, z);

	VehicleAttitude va = vehicle_setpoint_ENU.attitude;
	va.yaw = -va.yaw;
	vehicle_command_ENU.setAttitude(va);
	vehicle_command_ENU.setPosition(x, y, z);

	return;
}

void Controller::sendPositionCommand(double x, double y, double z)
{
	setPositionCommand(x, y, z);

//	VehicleAttitude va = vehicleState->vehicle_setpoint_ENU.attitude;
//	va.yaw = -va.yaw;
//	vehicleState->vehicle_command_ENU.setAttitude(va);
//	vehicleState->vehicle_command_ENU.setPosition(x, y, z);
	sendPositionCommand();

	return;
}

//void Controller::startControlLoop(double period) {
//	loopTimer.setPeriod(ros::Duration(period));
//	loopTimer.start();
//	ROS_INFO("TIMER STARTED");
//
//	return;
//}
//
//void Controller::doControlMode() {
//	printf("++++ ");
//	switch (controlMode)
//	{
//	case ControlMode::LANDED:
//		printf("LANDED\n");
//		break;
//	case ControlMode::LANDING_WAYPOINT:
//		printf("LANDING_WAYPOINT\n");
//		landVehicleWaypoint();
//		break;
//	case ControlMode::LANDING_DESCENDING:
//		printf("LANDING_DESCENDING\n");
//		landVehicleDescending();
//		break;
//	case ControlMode::LOITER:
//		printf("LOITER\n");
//		break;
//	case ControlMode::TAKING_OFF:
//		printf("TAKING OFF\n");
//		launchVehicle();
//		return;
//		break;
//	case ControlMode::TRACKING_TARGET_TRANSITION:
//		printf("TRANSITION\n");
//		changingTarget();
//		break;
//	case ControlMode::TRACKING_TARGET:
//		printf("TRACKING\n");
//		break;
//	}
//	geometry_msgs::Pose sp = vehicleState->vehicle_command_ENU.getPose();
//	sendPositionCommand(sp.position.x, sp.position.y, sp.position.z);
//}

//void Controller::controlLoopCallback(const ros::TimerEvent& te) {
//    controlLoop();
//}

void Controller::controlLoop()
{
    if (!controlEnabled)
    {
        return;
    }


    if (!initLoop)
    {
        initLoop = true;
    }
    else
    {
        // Currently send the position setpoint
        geometry_msgs::Pose sp = vehicle_command_ENU.getPose();
        sendPositionCommand(sp.position.x, sp.position.y, sp.position.z);
    }

    return;
}

void Controller::watchDogLoop()
{
    ros::Duration dt =  ros::Time::now() - lastLoopTime;
    if (dt.toSec() > WATCHDOG_TIMEOUT)
    {
        ROS_WARN("***** WARNING WATCHDOG TIMEOUT *****");
        if (locator != NULL)
        {
            locator->updateFCULocation();
        }

        // Simply resend the last position command
        sendPositionCommand();
    }
}

void Controller::watchDogCallback(const ros::TimerEvent& te)
{
    watchDogLoop();
}

bool Controller::isVehiclePoseInitialized()
{
    return locator == NULL ? false : locator->isVehiclePoseInitialized();
}

void Controller::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    if (locator == NULL)
    {
        return;
    }

    // Determine the flight mode based upon the joystick buttons
    FlightMode fm = getJoyFlightMode(msg);
    switch (fm)
    {
        case FlightMode::MANUAL:
        {
            //**              printf("JOY ---- MANUAL\n");
            if (flight_mode != FlightMode::MANUAL)
            {
                ROS_INFO("ENTERING FILIGHT MODE MANUAL");
            }
            joyCallback(msg, fm);
            flight_mode = FlightMode::MANUAL;
            // Since we are in manual mode disable the automatic control loop
            controlEnabled = false;

            return;
        }

        case FlightMode::POSCTL:
        case FlightMode::ALTCTL:
        {
            if (flight_mode != fm)
            {
                ROS_INFO("ENTERING FLIGHT MODE: %s\n", fm == FlightMode::POSCTL ? "POSCTL" : "ALTCTL");
            }
            if (flight_mode == FlightMode::MANUAL)
            {
                controlEnabled = true;
                // initialize the setpoint to the current position so that the joystick
                // inputs create setpoints relative to that.
                vehicle_setpoint_ENU.setPose(locator->vehicle_current_ENU.getPose());
                ROS_INFO("Vehicle command updated x:%5.3f y:%5.3f z:%5.3f",
                         vehicle_command_ENU.getPose().position.x,
                         vehicle_command_ENU.getPose().position.y,
                         vehicle_command_ENU.getPose().position.z);
            }
            flight_mode = fm;
            // Process the joystick inputs considering the flight mode
            joyCallback(msg, fm);
        }
    }

}

void Controller::joyCallback(const sensor_msgs::Joy::ConstPtr& msg, FlightMode fm)
{
//      ROS_INFO("RAW JOY: AXES[%4.2f %4.2f %4.2f %4.2f %4.2f %4.2f] BUTTONS[%d %d %d %d]",
//                      (double)msg->axes[0],
//                      (double)msg->axes[1],
//                      (double)msg->axes[2],
//                      (double)msg->axes[3],
//                      (double)msg->axes[4],
//                      (double)msg->axes[5],
//                      msg->buttons[0],
//                      msg->buttons[1],
//                      msg->buttons[2],
//                      msg->buttons[3]);
    ROS_DEBUG("RPYT[%4.2f %4.2f %4.2f %4.2f]",
                (double)msg->axes[joyAxes.roll],
                        (double)msg->axes[joyAxes.pitch],
                        (double)msg->axes[joyAxes.yaw],
                        (double)msg->axes[joyAxes.throttle]);

    if (isVehiclePoseInitialized())
    {
        VehicleAttitude currentAttitude = locator->vehicle_current_ENU.getAttitude();

        ROS_DEBUG("<ATT[r:%5.3f p:%5.3f y:%5.3f t:%5.3f]",
                  currentAttitude.roll,
                  currentAttitude.pitch,
                  currentAttitude.yaw,
                  currentAttitude.throttle);
        switch (fm)
        {
            case FlightMode::MANUAL:
                updateAttitudeSetpoint(msg, currentAttitude);
                break;
            case FlightMode::ALTCTL:
                updatePositionSetpoint(msg, false);
                break;
            case FlightMode::POSCTL:
                updatePositionSetpoint(msg, true);
                break;
        }
    }
}


void Controller::updateAttitudeSetpoint(const sensor_msgs::Joy::ConstPtr& msg, VehicleAttitude currentAttitude)
{
    // Set the attitude rotation commands based upon the joystick input
    double roll_sp, pitch_sp, yaw_sp, throttle_sp;

    // Calculate pitch
    pitch_sp = (msg->axes[joyAxes.pitch] + joyOffset.pitch) * attitudeScale.pitch;

    // Calculate roll
    roll_sp = (msg->axes[joyAxes.roll] + joyOffset.roll) * attitudeScale.roll;

    // Calculate yaw
    yaw_sp = ((msg->axes[joyAxes.yaw] + joyOffset.yaw) * attitudeScale.yaw) + currentAttitude.yaw;

    // Calculate the throttle
    throttle_sp = (msg->axes[joyAxes.throttle] + joyOffset.throttle) * attitudeScale.throttle;
    if (throttle_sp < 0)
    {
        throttle_sp = 0;
    }

    sendAttitudeCommand(roll_sp, pitch_sp, yaw_sp, throttle_sp);
}

void Controller::updatePositionSetpoint(const sensor_msgs::Joy::ConstPtr& msg, bool adjust_xy)
{
    double x_off, y_off, z_off, joy_sp;

    // Calculate the z offset
    joy_sp = msg->axes[joyAxes.throttle];
    if (joy_sp > -th_deadband && joy_sp < th_deadband)
    {
        joy_sp = 0;
    }

    z_off = joy_sp * maxPos.throttle;

    if (adjust_xy)
    {
        // Calculate the y offset
        joy_sp = msg->axes[joyAxes.pitch];
        if (joy_sp > -xy_deadband && joy_sp < xy_deadband)
        {
            joy_sp = 0;
        }
        y_off = joy_sp * maxPos.pitch;

        // Calculate the x offset
        joy_sp = msg->axes[joyAxes.roll];
        if (joy_sp > -xy_deadband && joy_sp < xy_deadband)
        {
            joy_sp = 0;
        }
        x_off = joy_sp * -maxPos.roll;
    }
    else
    {
        y_off = 0;
        x_off = 0;
    }
    geometry_msgs::Point p_off = locator->convertRFUtoENUoff(x_off, y_off, z_off);
    geometry_msgs::Point p = vehicle_setpoint_ENU.position;
    p.x += p_off.x;
    p.y += p_off.y;
    p.z += p_off.z;
    vehicle_setpoint_ENU.setPosition(p);
    sendPositionCommand(p.x, p.y, p.z);
}

