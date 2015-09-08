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
    ROS_INFO(">SEND ATT[R:%5.3f P:%5.3f Y:%5.3f T:%5.3f]",
             roll, pitch, yaw, throttle);

    vehicle_command_ENU.setAttitude(roll, pitch, yaw, throttle);
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
//
//void Controller::relativeSetpointCallback(
//		const brain_box_msgs::BBVxKinematics::ConstPtr& msg) {
//}
//
//void Controller::absoluteSetpointCallback(
//		const brain_box_msgs::BBVxKinematics::ConstPtr& msg) {
//}
//
////typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
////_header_type header;
////
////typedef uint8_t _command_type;
////_command_type command;
////
////typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _target_id_type;
////_target_id_type target_id;
////
////typedef uint8_t _frame_of_reference_type;
////_frame_of_reference_type frame_of_reference;
////
////typedef  ::brain_box_msgs::BBVxKinematics_<ContainerAllocator>  _kinematics_type;
////_kinematics_type kinematics;
////
////
//// enum { COMMAND_START = 0u };
////  enum { COMMAND_STOP = 1u };
////  enum { COMMAND_TRACK_TARGET = 2u };
////  enum { FOR_TARGET = 0u };
////
//void Controller::controlCommandCallback(
//		const brain_box_msgs::BBVxCommand::ConstPtr& msg) {
//
////	brain_box_msgs::BBVxCommand msg;
////	msg.header.stamp = ros::Time::now();
////	msg.command = brain_box_msgs::BBVxCommand::COMMAND_TRACK_TARGET;
////	msg.target_id = "1";
////	msg.frame_of_reference = brain_box_msgs::BBVxCommand::FOR_TARGET;
////	msg.kinematics.position.x = 0.0;
////	msg.kinematics.position.y = y;
////	msg.kinematics.position.z = 0.0;
////	cmnd_pub_.publish(msg);
//
//	if (msg->command == brain_box_msgs::BBVxCommand::COMMAND_TRACK_TARGET)
//	{
//		/*
//		 * Track target at some position
//		 */
//		trackTarget(msg->target_id, msg->kinematics);
//	}
//	else if (msg->command == brain_box_msgs::BBVxCommand::COMMAND_START)
//	{
//		if (controlMode == ControlMode::LANDED)
//		{
//			startLaunch();
//		}
//		else
//		{
//			ROS_WARN("CANT LAUNCH - NOT LANDED");
//		}
//	}
//	else if (msg->command == brain_box_msgs::BBVxCommand::COMMAND_STOP)
//	{
//		if (controlMode == ControlMode::LOITER ||
//				controlMode == ControlMode::TRACKING_TARGET)
//		{
//			startLanding();
//		}
//		else
//		{
//			ROS_WARN("CANT LAND - NOT LOITERING OR TRACKING");
//		}
//	}
//}

///*
// * Array to map the control mode of the smart pilot to the status mode
// */
////enum class ControlMode {LANDED=0, TAKING_OFF, LANDING_WAYPOINT, LANDING_DESCENDING, LOITER, TRACKING_TARGET, TRACKING_TARGET_TRANSITION};
//
////enum { STATE_TAKING_OFF = 1u };
////enum { STATE_LOITER = 2u };
////enum { STATE_TRACKING = 3u };
////enum { STATE_LANDING = 4u };
//unsigned int controlMode_2_BBSPSTatus[] = {
//		brain_box_msgs::BBSPStatus::STATE_LANDED,
//		brain_box_msgs::BBSPStatus::STATE_TAKING_OFF,
//		brain_box_msgs::BBSPStatus::STATE_LANDING,
//		brain_box_msgs::BBSPStatus::STATE_LANDING,
//		brain_box_msgs::BBSPStatus::STATE_LOITER,
//		brain_box_msgs::BBSPStatus::STATE_TRACKING,
//		brain_box_msgs::BBSPStatus::STATE_TRACKING};
//
//void Controller::sendStatus() {
//	sp_status_.header.stamp = ros::Time::now();
//	sp_status_.status.summary = brain_box_msgs::BBStatus::SUMMARY_GREEN;
//	sp_status_.state = brain_box_msgs::BBSPStatus::STATE_LANDED;
//	sp_status_.target_id = track_id;
////	sp_status_.target_id = vehicleState->target_list_RFU.getFocus() != NULL ?
////			vehicleState->target_list_RFU.getFocus()->object_id : "XX";
////	std::cout << "=====SEND STATUS id:" << sp_status_.target_id << ", state:" << sp_status_.state << "\n";
//	statusPub.publish(sp_status_);
//}
//
//void Controller::trackTarget(
//		brain_box_msgs::BBVxCommand::_target_id_type target_id,
//		brain_box_msgs::BBVxKinematics kinematics) {
//
//	geometry_msgs::Point p = convertRFUtoENUoff(kinematics.position.x,
//			kinematics.position.y, kinematics.position.z);
//
//	p.x = -p.x;
//	p.y = -p.y;
//	slide_position_to = p;
//	if (vehicleState->target_list_RFU.isFocus(target_id))
//	{
//		vehicleState->vehicle_setpoint_ENU.setPosition(p);
//		setPositionCommand(p.x, p.y, p.z);
//		controlMode = ControlMode::TRACKING_TARGET;
//		track_id = target_id;
//	}
//	else
//	{
//		Kinematics* k;
//		if ((k = vehicleState->target_list_RFU.findNode(target_id)) == NULL)
//		{
//			// can't currently see it so dont track it
//			return;
//		}
//		geometry_msgs::Point org = vehicleState->target_list_RFU.getFocus()->position;
//		double del_x = k->position.x - org.x;
//		double del_y = k->position.y - org.y;
//		double del_z = k->position.z - org.z;
//		p = convertRFUtoENUoff(kinematics.position.x+del_x,
//				kinematics.position.y+del_y, kinematics.position.z+del_z);
//		p.x = -p.x;
//		p.y = -p.y;
//
//		// invalidate the location so it will be reinitialized upon next update
//		tracking_init = false;
//		slide_start = ros::Time().now();
////		locator->setTargetPoseInitialized(false);
//		vehicleState->vehicle_setpoint_ENU.setPosition(p);
//		setPositionCommand(p.x, p.y, p.z);
//		controlMode = ControlMode::TRACKING_TARGET;
//		track_id = target_id;
////		controlMode = ControlMode::TRACKING_TARGET_TRANSITION;
//	}
//}
//
//void Controller::startLaunch() {
//	if (vehicleState->target_list_RFU.getFocus() != NULL)
//	{
//		controlMode = ControlMode::TAKING_OFF;
//		launch_focus = vehicleState->target_list_RFU.getFocus()->object_id;
//		launch_position = vehicleState->vehicle_current_ENU.position;
//		launch_start = ros::Time::now();
//	}
//}
//
//void Controller::launchVehicle() {
//	double launch_interval = ros::Time::now().toSec() - launch_start.toSec();
//	double alt = ((launch_duration_sec - launch_interval) / launch_duration_sec) * launch_position.z;
//	if (alt > 0)
//	{
//		alt = 0;
//	}
//printf("^^^^^ %f   %f^^^^^\n", alt, launch_position.z);
//	getVehicleState()->vehicle_setpoint_ENU.setPosition(launch_position.x, launch_position.y, alt);
//
////	setPositionCommand(launch_position.x, launch_position.y, alt);
//	if (launch_interval >= launch_duration_sec)
//	{
//		controlMode = ControlMode::LOITER;
//	}
////	double throttle = (launch_interval / launch_duration_sec) * launch_throttle;
////	if (throttle > launch_throttle)
////	{
////		throttle = launch_throttle;
////	}
////
////	sendAttitudeCommand(0.0, 0.0,
////			getVehicleState()->vehicle_current_ENU.getAttitude().yaw, throttle);
////	if (launch_interval >= launch_duration_sec)
////	{
////		loiter_position = getVehicleState()->vehicle_current_ENU.position;
////		setPositionCommand(launch_position.x, launch_position.y, launch_position.z);
////		controlMode = ControlMode::LOITER;
////	}
//}
//
//void Controller::startLanding() {
//	controlMode = ControlMode::LANDING_WAYPOINT;
//	getVehicleState()->vehicle_setpoint_ENU.setPosition(loiter_position.x, loiter_position.y, loiter_position.z);
//}
//
//void Controller::landVehicleWaypoint() {
//	double dist = pointDistance(loiter_position, getVehicleState()->vehicle_current_ENU.position);
//
//	if (dist < WAYPOINT_DIST)
//	{
//		controlMode = ControlMode::LANDING_DESCENDING;
////		getVehicleState()->vehicle_setpoint_ENU.setPosition(loiter_position.x, loiter_position.y, loiter_position.z);
//	}
//}
//
//void Controller::landVehicleDescending() {
//	double launch_interval = ros::Time::now().toSec() - launch_start.toSec();
//	double alt = ((launch_interval / launch_duration_sec) * launch_position.z) + 0.25;
//printf("^^^landing %f   %f\n", alt, launch_position.z);
//	getVehicleState()->vehicle_setpoint_ENU.setPosition(launch_position.x, launch_position.y, alt);
//
//	if (launch_interval >= launch_duration_sec)
//	{
//		controlMode = ControlMode::LANDED;
//	}
//}
//
//void Controller::changingTarget() {
//	if (!locator->isTargetPoseInitialized())
//	{
//		return;
//	}
//
//	if (!tracking_init)
//	{
//		slide_position_from = getVehicleState()->vehicle_current_ENU.position;
//		tracking_init = true;
//	}
//	double change_interval = ros::Time::now().toSec() - slide_start.toSec();
//	double factor = change_interval / slide_duration_sec;
//	double del_x = (slide_position_to.x - slide_position_from.x) * factor;
//	double del_y = (slide_position_to.y - slide_position_from.y) * factor;
//	double del_z = (slide_position_to.z - slide_position_from.z) * factor;
//
//	if (change_interval >= slide_duration_sec)
//	{
//		controlMode = ControlMode::TRACKING_TARGET;
//		getVehicleState()->vehicle_setpoint_ENU.setPosition(slide_position_to.x, slide_position_to.y, slide_position_to.z);
//		printf("XXXXXXXXX DONE %5.3f   %5.3f   %5.3f\n", slide_position_to.x,
//				slide_position_to.y, slide_position_to.z);
//	}
//	else
//	{
//		getVehicleState()->vehicle_setpoint_ENU.setPosition(slide_position_from.x+del_x,
//				slide_position_from.y+del_y, slide_position_from.z+del_z);
//		printf("----------SLIDING %5.3f   %5.3f   %5.3f\n", slide_position_from.x+del_x,
//				slide_position_from.y+del_y, slide_position_from.z+del_z);
//	}
//}
//
/*
** Deal with position setpoints
*/
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
//	ROS_DEBUG(">>>SEND POS[X:%5.3f Y:%5.3f Z:%5.3f]", x, y, z);
	printf(">>>SEND POS[X:%5.3f Y:%5.3f Z:%5.3f]\n", x, y, z);

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
            //              printf("JOY ---- MANUAL\n");
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

    sendAttitudeCommand(pitch_sp, roll_sp, yaw_sp, throttle_sp);
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

