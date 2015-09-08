/*
 * Kinematics.h
 *
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#include <math.h>

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

/*
 ** RPY Representation of vehicle state
 */
struct VehicleAttitude
{
    double roll;
    double pitch;
    double yaw;
    double throttle;
};

class Kinematics {
public:
    Kinematics();
    Kinematics(const geometry_msgs::Pose& p);
    Kinematics(const geometry_msgs::Pose& p, const std::string id);

    virtual ~Kinematics();

    // Identifier for the object represented by these kinematics
    std::string object_id {"foo"};

    // Current location
    geometry_msgs::Point position;
    // Global origin offset with respect to local position origin
    geometry_msgs::Point origin;
    // Current orientation
    geometry_msgs::Quaternion orientation;
    // Current attitude in RPY
    VehicleAttitude attitude;

    // Linear velocity
    geometry_msgs::Vector3 velLinear;
    // Angular velocity
    geometry_msgs::Vector3 velAngular;

    /*
     * General setters. Will maintain consistency between the pose with quaternions
     * and the vehcile attitude which is RPY. Whenever you set one the other gets calculated
     */
    void setPose(const geometry_msgs::PoseStamped& p) {setPose(p.pose);}
    void setPose(const geometry_msgs::Pose& p);
    geometry_msgs::Pose& getPose();

    void setPosition(const geometry_msgs::Point& p);
    void setPosition(double x, double y, double z);

    void setOrigin(const geometry_msgs::Point& p);
    void setOrigin(double x, double y, double z);

    const geometry_msgs::Point& getGlobalPosition();

    void setAttitude(const VehicleAttitude& va);
    void setAttitude(double roll, double pitch, double yaw, double throttle);
    void setAttitude(const geometry_msgs::Quaternion q);

    void setVelocity(const geometry_msgs::Vector3& linear, const geometry_msgs::Vector3& angular);
    void setAngularVelocity(const geometry_msgs::Vector3& vel);
    void setAngularVelocity(double x, double y, double z);
    void setLinearVelocity(const geometry_msgs::Vector3& vel);
    void setLinearVelocity(double x, double y, double z);

    void publishTransform();
    void test();
    void print();

    const VehicleAttitude& getAttitude() const {
        return attitude;
    }

    const geometry_msgs::Quaternion& getOrientation() const {
        return orientation;
    }

    const geometry_msgs::Point& getPosition() const {
        return position;
    }

    const geometry_msgs::Vector3& getAngularVelocity() const {
        return velAngular;
    }

    const geometry_msgs::Vector3& getLinearVelocity() const {
        return velLinear;
    }

    bool isPoseInitialized() const {
        return poseInitialized;
    }

    void setPoseInitialized(bool poseInitialized = false) {
        this->poseInitialized = poseInitialized;
    }

    const ros::Time& getUpdateTime() const {
        return update_time;
    }

    void setUpdateTime(const ros::Time& updateTime) {
        update_time = updateTime;
    }

    double pointDistance(geometry_msgs::Point p1, geometry_msgs::Point p2);

    VehicleAttitude quaternionMsg2Att(geometry_msgs::Quaternion q);

private:
    geometry_msgs::Pose _p;

    geometry_msgs::Point _global_position;

    ros::Time update_time;

    void _init();

    bool poseInitialized = false;
};

#endif /* KINEMATICS_H_ */
