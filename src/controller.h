/*
 * Controller.h
 *
 *  Created on: Mar 28, 2015
 *      Author: ubuntu
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <string.h>
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/Joy.h"
#include "control_toolbox/pid.h"

#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/PoseThrottle.h"

#include "std_msgs/Float64.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

#include "locator.h"

class Locator;

/**
 * Offboard flight modes
 */
enum FlightMode:int {MANUAL=0, ALTCTL, POSCTL, IDLE};

/**
 * Index of button in the joy message that determines the flight mode
 */
#define B_FLIGHT_MODE   2

/**
 * Index of button in the joy message that determines if the FCU is in offboard mode
 */
#define OFF_BOARD_BUTTON 0

/**
 * Watchdog timeout period
 */
#define WATCHDOG_TIMEOUT 3

/**
 * Joy stick axes mapping
 */
struct JoyAxes
{
  int roll;
  int pitch;
  int yaw;
  int throttle;
};

/**
 * Scale of the various joy stick axes
 */
struct AxesScale
{
  double roll;
  double pitch;
  double yaw;
  double throttle;
};

/*
** Need to adjust these to match your joystick or RC controller
*/ 
// The axis corresponding to the joystick
const JoyAxes exAxes = {0, 1, 2, 3};
// Offset of each axis inout
const AxesScale exOffset = {0.0,0.0,0.0,1.0};
// ascale of each axis input
const AxesScale exScale = {-0.6, 0.6, 1.0, 0.5};
// maximum value for each axis input
const AxesScale exMax = {0.1, 0.1, 0.1, 0.01};


/**
 * Controller class used to control the motion of the vehicle
 * It uses offboard mode and takes inputs from the joystick to
 * support the following flight modes:
 * MANUAL - takes inputs from the joystick and send attitude setpoints to the FCU
 * ALTCTL - takes inputs from the joystick and send position setpoints to the FCU
 *      note that ALTCTL only modifies the altitidue and fixes the x, y axes
 * POSCTL - takes inputs from the joystick and sends position setpoints to the FCU for all axes
 *
 * Note that the when you switch from MANUAL to ALTCTL or POSCTL then the current local
 * location of the vehicle becomes the setpoint
 */
class Controller {
public:
    Controller(ros::NodeHandle *n);
    virtual ~Controller();

    /**
     * Send attitude setpoint to the FCU
     */
    void sendAttitudeCommand(double roll, double pitch, double yaw, double throttle);

    /**
     * Send position setpoint to the FCU
     */
    void sendPositionCommand(double x, double y, double z);

    /**
     * Set the position setpoint command without sending it to the FCU
     */
    void setPositionCommand(double x, double y, double z);

    /**
     * Set the FCU flight mode to manual.
     * This is a fail safe mechanism and is called when a vision error is detected.
     */
    void setFCUFlightModeManual();

    /**
     * Deal with Vision location error.
     * Called by the Locator when an error is detected
     */
    void locationError();

    /**
     * Called by the Locator whenever the vehicle pose is updated
     */
    void vehiclePositionUpdated();

    /**
     * Called by the Locator whenever the target pose is updated
     */
    void targetPositionUpdated();

    /**
     * Initialize the locator associated with this controller
     */
    void setLocator(Locator* locator) {
        this->locator_ = locator;
    }

    bool isVehiclePoseInitialized();

protected:
private:
    /*
     * Calculates the error between two poses
     */
    geometry_msgs::Pose calcPoseError(geometry_msgs::Pose sp, geometry_msgs::Pose pv);

    /*
     * This is the main control loop that is called to update setpoints.
     * Currently this is called whenever the target pose is updated or the joy stick
     * inputs are processed.
     */
    void controlLoop();

    /**
     * Callback for the watchdog timer
     */
    virtual void watchDogCallback(const ros::TimerEvent& te);

    /**
     * Called whenever the watchdog timer goes off by the watchdog callback method
     */
    virtual void watchDogLoop();

    /**
     * ROS node handle
     */
    ros::NodeHandle *ros_node_;

    /**
     * Vehicle pose command (atttude or position setpoint) state last sent to FCU
     */
    Kinematics vehicle_command_ENU_;

    /**
     * Desired vehicle pose
     */
    Kinematics vehicle_setpoint_ENU_;

    /**
     * Locator associated with this controller
     */
    Locator* locator_ {NULL};


    ros::Time last_update_time_;

    /**
     * Used to enable/disable the control loop
     */
    bool control_enabled_ = true;

    /**
     * Used to enable/disable sending setpoints to the FCU
     */
    bool off_board_enabled_ = false;

    //        ros::Time in_time; // time when processing started

    /**
     * Timer used to control the watchdog
     */
    ros::Timer watchdog_timer_;

    /**
     * Initialize ROS handles
     */
    void initROS(ros::NodeHandle *n);

    /**
     * Send an attitude setpoint to the FCU
     */
    void sendAttitudeCommand();

    /**
     * Sends a position setpoint to the FCU
     */
    void sendPositionCommand();

    /**
     * If the following is true then initialize the vehicle setpoints the next time a vehicle location is received
     */
    bool initialize_setpoint_ {true};

    /*===============
     * ROS handles
     *===============*/

    /**
     * ROS joystick topic handle
     */
    ros::Subscriber joy_sub_;

    /**
     * ROS handle to publish attitude setpoints
     */
    ros::Publisher pt_attitude_setpoint_pub_;
    //	ros::Publisher attitudeSetpointPub;

    /**
     * ROS handle to publish position setpoints
     */
    ros::Publisher position_setpoint_pub_;

    /**
     * ROS handle to set the FCU mode
     */
    ros::ServiceClient set_mode_client_;

    /*====================================================================
     * Methods and attributes for handing the joystick input and control
     *====================================================================*/

    /**
     * Callback for updates to the joystick topic
     */
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

    /**
     * Joystick callback for processing joystick inputs, including the flight mode
     */
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg, FlightMode fm);

    /**
     * Update the attitude setpoint
     * Takes inputs from the joystick and determines the appropriate attitude setpoint
     * to send to the FCU.
     * This is called duing MANUAL flight mode.
     */
    void updateAttitudeSetpoint(const sensor_msgs::Joy::ConstPtr& msg, VehicleAttitude currentAttitude);

    /**
     * Update the position setpoint.
     * Takes inputs from the joystick and calculates new posisition setpoints
     * and send the new position setpoints to the FCU.
     * This is called furing ALTCTL and POSCTL flight modes.
     */
    void updatePositionSetpoint(const sensor_msgs::Joy::ConstPtr& msg, bool adjust_xy);

    /*
     * Return the fligt mode based upon joystick buttons
     */
    FlightMode getJoyFlightMode(const sensor_msgs::Joy::ConstPtr& msg) {
        return msg->buttons[B_FLIGHT_MODE] < 4 ? (FlightMode)msg->buttons[B_FLIGHT_MODE] : FlightMode::IDLE;};

    /**
     * Current offboard flight mode
     */
    FlightMode flight_mode_ = FlightMode::IDLE;

    // Joystick mappings and various parameters
    JoyAxes joy_axis_ = exAxes;
    AxesScale joy_offset_ = exOffset;
    AxesScale attitude_scale_ = exScale;
    AxesScale max_pos_value_ = exMax;
    double th_deadband {0.25};
    double xy_deadband {0.05};
};

#endif /* CONTROLLER_H_ */
