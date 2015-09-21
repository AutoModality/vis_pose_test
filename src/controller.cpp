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
}

Controller::~Controller()
{
    // TODO Auto-generated destructor stub
}

void Controller::initROS(ros::NodeHandle *n)
{
    ros_node_ = n;

    // Subscribe to joystick topic
    joy_sub_ = ros_node_->subscribe<sensor_msgs::Joy>("joy", 10, &Controller::joyCallback, this);

    // Advertise the attitude setpoint topic
    pt_attitude_setpoint_pub_ = ros_node_->advertise<mavros_msgs::PoseThrottle>("/mavros/setpoint_attitude/pt_attitude", 1000);

    // Advertise the position setpoint topic
    position_setpoint_pub_ = ros_node_->advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1000);

    // Subscribe to service for changing the FCU mode
    // Currently not used
    set_mode_client_ = ros_node_->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // Set up the watchdog timer
    watchdog_timer_ = ros_node_->createTimer(ros::Duration(0.2), &Controller::watchDogCallback, this);
    watchdog_timer_.start();

    ROS_INFO("^^^ ROS ON CONTROLLER INITIALIZED ^^^");

    return;
}

void Controller::locationError()
{
	// An error has occurred so stop sending setpoints to the FCU and let it do its normal
	// failsafe to drop out of offboard mode
	off_board_enabled_ = false;
}


void Controller::sendAttitudeCommand()
{
	/*
	 * Don't send setpoint unless offboard enabled.
	 */
	if (!off_board_enabled_)
	{
		return;
	}
	mavros_msgs::PoseThrottle pt;
    pt.pose = vehicle_command_ENU_.getPose();
    pt.throttle.data = vehicle_command_ENU_.attitude.throttle;
    pt_attitude_setpoint_pub_.publish(pt);

    last_update_time_ = ros::Time::now();

    return;
}

void Controller::sendAttitudeCommand(double roll, double pitch, double yaw, double throttle)
{
    ROS_DEBUG(">SEND ATT SETPOINT[R:%5.3f P:%5.3f Y:%5.3f T:%5.3f]",
             roll, pitch, yaw, throttle);

    vehicle_command_ENU_.setAttitude(roll, pitch, yaw, throttle);
    sendAttitudeCommand();

    return;
}

void Controller::vehiclePositionUpdated()
{
    if (initialize_setpoint_ && locator_ != NULL)
    {
        // Initialize the vehicle's current setpoints equal to the current position.
        vehicle_command_ENU_.setPose(locator_->vehicle_current_ENU_.getPose());
        vehicle_setpoint_ENU_.setPose(locator_->vehicle_current_ENU_.getPose());

        initialize_setpoint_ = false;
    }
}

void Controller::targetPositionUpdated()
{
	// Do nothing place holder for those that want to do some sort of control on each target frame update
	return;
}

void Controller::sendPositionCommand()
{
	if (locator_ == NULL || !locator_->isTargetPoseInitialized() || !off_board_enabled_)
	{
		return;
	}
	geometry_msgs::PoseStamped vpc;
	vpc.pose = vehicle_command_ENU_.getPose();
	vpc.header.stamp = ros::Time::now();
	position_setpoint_pub_.publish(vpc);
	ROS_DEBUG(">SEND POS SETPOINT[X:%5.3f Y:%5.3f Z:%5.3f]",
			vpc.pose.position.x,
			vpc.pose.position.y,
			vpc.pose.position.z);

	last_update_time_ = ros::Time::now();

	return;
}

void Controller::setPositionCommand(double x, double y, double z)
{
	VehicleAttitude va = vehicle_setpoint_ENU_.attitude;
	vehicle_command_ENU_.setAttitude(va);
	vehicle_command_ENU_.setPosition(x, y, z);

	return;
}

void Controller::sendPositionCommand(double x, double y, double z)
{
	setPositionCommand(x, y, z);

	sendPositionCommand();

	return;
}

void Controller::watchDogLoop()
{
    ros::Duration dt =  ros::Time::now() - last_update_time_;
    if (dt.toSec() > WATCHDOG_TIMEOUT)
    {
        ROS_WARN("***** WATCHDOG TIMEOUT NOT SENDING SETPOINTS *****");

        last_update_time_ = ros::Time::now();
    }
}

void Controller::watchDogCallback(const ros::TimerEvent& te)
{
    watchDogLoop();
}

bool Controller::isVehiclePoseInitialized()
{
    return locator_ == NULL ? false : locator_->isVehiclePoseInitialized();
}

void Controller::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    if (locator_ == NULL)
    {
        return;
    }

    // Determine the flight mode based upon the joystick buttons
    FlightMode fm = getJoyFlightMode(msg);
    switch (fm)
    {
        case FlightMode::MANUAL:
        {
            /*
             * Fail safe mechanism to only enable offboard mode when the flight mode is manual AND
             * The offboard button on the RC controller is not enabled.
             * This will ensure that if off board mode is ever disabled then certain steps must be taken
             * by the operator to put it back into offboard mode.
             * Note that a better way to do this would be to query the FCU for its flight mode instead
             * instead of relying on the state of the offboard switch on the joystick.
             */
            if (!msg->buttons[OFF_BOARD_BUTTON])
            {
            	off_board_enabled_ = true;
            }

            if (flight_mode_ != FlightMode::MANUAL)
            {
                ROS_INFO("ENTERING FLIGHT MODE MANUAL");
            }
            joyCallback(msg, fm);
            flight_mode_ = FlightMode::MANUAL;
            // Since we are in manual mode disable the automatic control loop
            control_enabled_ = false;

            return;
        }

        case FlightMode::POSCTL:
        case FlightMode::ALTCTL:
        {
            if (flight_mode_ != fm)
            {
                ROS_INFO("ENTERING FLIGHT MODE: %s\n", fm == FlightMode::POSCTL ? "POSCTL" : "ALTCTL");
            }
            if (flight_mode_ == FlightMode::MANUAL)
            {
                control_enabled_ = true;
                // initialize the setpoint to the current vehicle position so that the joystick
                // inputs create position setpoints relative to that.
                vehicle_setpoint_ENU_.setPose(locator_->vehicle_current_ENU_.getPose());
            }
            flight_mode_ = fm;
            // Process the joystick inputs considering the flight mode
            joyCallback(msg, fm);
        }
    }

}

void Controller::joyCallback(const sensor_msgs::Joy::ConstPtr& msg, FlightMode fm)
{
	ROS_DEBUG("RAW JOY: AXES[%4.2f %4.2f %4.2f %4.2f %4.2f %4.2f] BUTTONS[%d %d %d %d]",
			(double)msg->axes[0],
			(double)msg->axes[1],
			(double)msg->axes[2],
			(double)msg->axes[3],
			(double)msg->axes[4],
			(double)msg->axes[5],
			msg->buttons[0],
			msg->buttons[1],
			msg->buttons[2],
			msg->buttons[3]);
    ROS_DEBUG("RPYT[%4.2f %4.2f %4.2f %4.2f]",
                (double)msg->axes[joy_axis_.roll],
                        (double)msg->axes[joy_axis_.pitch],
                        (double)msg->axes[joy_axis_.yaw],
                        (double)msg->axes[joy_axis_.throttle]);

    if (isVehiclePoseInitialized())
    {
        VehicleAttitude currentAttitude = locator_->vehicle_current_ENU_.getAttitude();

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
    pitch_sp = (msg->axes[joy_axis_.pitch] + joy_offset_.pitch) * attitude_scale_.pitch;

    // Calculate roll
    roll_sp = (msg->axes[joy_axis_.roll] + joy_offset_.roll) * attitude_scale_.roll;

    // Calculate yaw
    yaw_sp = ((msg->axes[joy_axis_.yaw] + joy_offset_.yaw) * attitude_scale_.yaw) + currentAttitude.yaw;

    // Calculate the throttle
    throttle_sp = (msg->axes[joy_axis_.throttle] + joy_offset_.throttle) * attitude_scale_.throttle;
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
    joy_sp = msg->axes[joy_axis_.throttle];
    if (joy_sp > -th_deadband && joy_sp < th_deadband)
    {
        joy_sp = 0;
    }

    z_off = joy_sp * max_pos_value_.throttle;

    if (adjust_xy)
    {
        // Calculate the y offset
        joy_sp = msg->axes[joy_axis_.roll];
        if (joy_sp > -xy_deadband && joy_sp < xy_deadband)
        {
            joy_sp = 0;
        }
        y_off = joy_sp * max_pos_value_.roll;

        // Calculate the x offset
        joy_sp = msg->axes[joy_axis_.pitch];
        if (joy_sp > -xy_deadband && joy_sp < xy_deadband)
        {
            joy_sp = 0;
        }
        x_off = joy_sp * max_pos_value_.pitch;
    }
    else
    {
        y_off = 0;
        x_off = 0;
    }
    geometry_msgs::Point p_off = locator_->convertFLUtoENUoff(x_off, y_off, z_off);
    geometry_msgs::Point p = vehicle_setpoint_ENU_.position;
    p.x += p_off.x;
    p.y += p_off.y;
    p.z += p_off.z;
    vehicle_setpoint_ENU_.setPosition(p);
    sendPositionCommand(p.x, p.y, p.z);
}

