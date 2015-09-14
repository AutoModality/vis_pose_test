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

#include "mavros/SetMode.h"
#include "mavros/PoseThrottle.h"

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
 * Timeout before sending another setpoint command
 */
#define WATCHDOG_TIMEOUT 2.5

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

const JoyAxes gpAxes = {4, 3, 0, 1};
const AxesScale gpOffset = {0.0,0.0,0.0,0.0};
const AxesScale gpScale = {-0.6, -0.6, 1.0, 0.6};

const JoyAxes exAxes = {0, 1, 2, 3};
const AxesScale exOffset = {0.0,0.0,0.0,1.0};
const AxesScale exScale = {-0.6, -0.6, 1.0, 0.5};
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

    /*
     * Called by the Locator whenever the target pose is updated
     */
    void targetPositionUpdated();

    /**
     * Initialize the locator associated with this controller
     */
    void setLocator(Locator* locator) {
        this->locator = locator;
    }

    bool isVehiclePoseInitialized();

protected:
private:
    /*
     * Calculates the error between two poses
     */
    geometry_msgs::Pose calcPoseError(geometry_msgs::Pose sp, geometry_msgs::Pose pv);

    //        /**
    //         * Callback for control loop timer
    //         */
    //        virtual void controlLoopCallback(const ros::TimerEvent& te);

    /*
     * This is the main control loop that is called to update setpoints.
     * Currently this is called whenever the target pose is updated or the joy stick
     * inputs are processed.
     * Also may be called by the watchdog timer if no updates have occured recently.
     */
    void controlLoop();

    /**
     * Callback for the watchdog timer
     * This insures that locations and setpoints keep being sent to the FCU
     * so that it does not drop out of offboard mode.
     */
    virtual void watchDogCallback(const ros::TimerEvent& te);

    /**
     * Called whenever the watchdog timer goes off by the watchdog callback method
     */
    virtual void watchDogLoop();

    /**
     * ROS node handle
     */
    ros::NodeHandle *rosNode;

    /**
     * Vehicle pose command (atttude or position setpoint) state last sent to FCU
     */
    Kinematics vehicle_command_ENU;

    /**
     * Desired vehicle pose
     */
    Kinematics vehicle_setpoint_ENU;

    /**
     * Locator associated with this controller
     */
    Locator* locator {NULL};


    ros::Time lastLoopTime;

    /**
     * Used to initialize the control loop
     */
    bool initLoop = false;

    /**
     * Used to enable/disable the control loop
     */
    bool controlEnabled = true;


    //        ros::Time in_time; // time when processing started

    /**
     * Timer used to control the watchdog
     */
    ros::Timer watchDogTimer;

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
     * If the following is true then set the target setpoint to be equal to the next target Pose received.
     */
    bool setTargetSetpointEqCurrent {false};

    /**
     * If the following is true then set the vehicle command to be equal to the next vehicleLocation received,
     * This provides a means to initialize the setpoints as the flight mode is changed
     * fro MANUAL to ALTCTL or POSCTL
     */
    bool vehicleCommandEqCurrent {true};

    /**
     * Used to track when the last setpoint command was sent to the UAV
     */
//    int setpointCommandCnt = 0;

    /*
     * ROS handles
     */
    /**
     * ROS joystick topic handle
     */
    ros::Subscriber joySub;

    /**
     * ROS handle to publish attitude setpoints
     */
    ros::Publisher ptAttitudeSetpointPub;
    //	ros::Publisher attitudeSetpointPub;

    /**
     * ROS handle to publish position setpoints
     */
    ros::Publisher positionSetpointPub;

    /**
     * ROS handle to set the FCU mode
     */
    ros::ServiceClient setModeClient;

    //	ros::Timer loopTimer;

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
     * Current flight mode
     */
    FlightMode flight_mode = FlightMode::IDLE;

    // Joystick mappings and various parameters
    JoyAxes joyAxes = exAxes;
    AxesScale joyOffset = exOffset;
    AxesScale attitudeScale = exScale;
    AxesScale maxPos = exMax;
    double th_deadband {0.25};
    double xy_deadband {0.05};
};

#endif /* CONTROLLER_H_ */
