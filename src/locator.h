/*
 * Locator.h
 *
 */

#ifndef LOCATOR_H_
#define LOCATOR_H_

#include <math.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "std_msgs/Float64.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

#include "brain_box_msgs/BBPose.h"
#include "brain_box_msgs/BBPoseArray.h"

#include "kinematics.h"
#include "controller.h"

class Controller;

// Period for the watchdog timer
#define WATCH_DOG_PERIOD 0.2

/**
 * Class for maintaining the pose of the target and the vehicle.
 *
 * Listens on the ROS topic /mavros/local_position/local for the vehicles pose
 * Listens on the ROS topic /sensor/camera/target/actual for the target pose
 * Publishes the vehicle position on ROS topic /mavros/vision_pose/pose
 */
class Locator {
public:
    Locator(ros::NodeHandle *n, Controller* c);
    virtual ~Locator();

    bool isVehiclePoseInitialized() const {
        return vehiclePoseInitialized_;
    }

    bool isTargetPoseInitialized() const {
        return targetPoseInitialized_;
    }

    /**
     * updateFCULocation.
     * Update the FCU with the current vision based vehicle location
     */
    void updateFCULocation();

    /**
     * Return the target coordinates wrt to the target origin.
     * Essentially this provides a means of creating an offset to the target
     * coordinates by setting target_origin_ENU to non zero.
     */
    geometry_msgs::Pose getTargetCurrentENUWRTOrigin();

    /**
     * Converts offsets from FLU body frame coordinates into ENU inertial frame coordinates.
     * Possibly a more efficient way to do this with ROS frames.
     */
    geometry_msgs::Point convertFLUtoENUoff(double front, double left, double up);

    /**
     * target pose in ENU frame
     */
    Kinematics target_current_ENU_;

    /**
     * target pose in Front, Left, Up (FLU) frame
     */
    Kinematics target_current_FLU_;

    /**
     * target origin in ENU
     */
    Kinematics target_origin_ENU_;

    /**
     * vehicle pose in ENU
     */
    Kinematics vehicle_current_ENU_;

//    /**
//     * Return the yaw of the target. This yaw is wrt the camera frame
//     * The target of the yaw may or may not be support depending upon the
//     * vision system.
//     * In the case of the vision system used in conjunction with this test the yaw was actually returned
//     * as the roll.
//     */
//    double getTargetYaw() {
//    	return target_current_FLU.attitude.roll;
//    }

protected:
private:
    /**
     * Initialize ROS interface
     */
    void initROS(ros::NodeHandle *n);

    /**
     * Callback for vehicle pose from FCU (local)
     */
    virtual void localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    /**
     * Callback for target pose from vision system
     */
    virtual void targetPoseCallback(const geometry_msgs::PoseArray::ConstPtr& msg);

    /**
     * Handle for the main ROS node
     */

    ros::NodeHandle* ros_node_;
    /**
     * Handle for the pose received from the FCU
     */

    ros::Subscriber local_pos_sub_;
    /**
     * ROS handle for the target pose received from the vision system
     */

    ros::Subscriber target_pos_sub_;
    /**
     * ROS handle for publishing the vision derived pose to the FCU
     */

    ros::Publisher local_pos_pub_;

    /**
     * pointer to the controller used for controlling motion of vehicle
     */
    Controller* controller_ {NULL};

    /**
     * This function takes the FLU target pose which is relative to the vehicle and transforms it into
     * the intertial frame with the vehicle at the origin.
     * Possibly a more efficient way to do this with ROS frames
     */
    void convertTargetFLUtoENU();

    /**
     * time when the location was last updated
     */
    ros::Time location_update_time_;

    /**
     * Used to track if the vehicle pose has been initialized
     */
    bool vehiclePoseInitialized_ {false};

    /**
     * Used to track if the target pose has been initialized
     */
    bool targetPoseInitialized_ {false};

    /**
     * Means for synchronizing the YAW of the target and the vehicle.
     * Used as part of the filtering.
     * Only effective of the vision system provide a target yaw.
     */
    void syncVehicleandTargetPose();

//    double vehicleYawOrigin = 0.0;
//    double targetYawOrigin = 0.0;
//    double deltaVehicleYaw = 0.0;
//    double deltaTargetYaw = 0.0;

    /**
     * Number of consecutive vision frames without a target.
     * Initialized to one so that any logic that depends on this being zero won't
     * execute until it is reset by the first valid frame.
     */
    int position_error_cnt_ = 1;

    /**
     * Cache of the vision pose message
     * Primarily used to cache the header of the incoming target pose
     */
    geometry_msgs::PoseStamped vision_pose_;

    /**
     * Used to cache previous vision poses to perform some simple filtering
     */
    geometry_msgs::Pose filtered_pose_;

    /**
     * Used to apply simple low pass filter to target locations.
     * Filters each axis separately. Also does filtering of the yawing which is
     * dependent upon knowing the attitude of the target. If you have a 3D gimbal
     * that fixes the heading then this is probably not necessary.
     */
    geometry_msgs::Pose& filterPosition(geometry_msgs::Pose pose);

    // Filter constants for each axis of simple filter
    const double X_ALPHA = 0.15;
    const double X_BETA = 1.0 - X_ALPHA;
    const double Y_ALPHA = 0.15;
    const double Y_BETA = 1.0 - Y_ALPHA;
    const double Z_ALPHA = 0.15;
    const double Z_BETA = 1.0 - Z_ALPHA;
};

#endif /* LOCATOR_H_ */
