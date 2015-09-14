/*
 * pose_proxy.cpp
 *
 */

#include "locator.h"

Locator::Locator(ros::NodeHandle* n, Controller* c)
{
    controller = c;
    if (controller != NULL)
    {
        controller->setLocator(this);
    }
    initROS(n);
}

Locator::~Locator()
{
    // TODO Auto-generated destructor stub
}

void Locator::initROS(ros::NodeHandle *n)
{
    rosNode = n;

    /*
     * Subscribe to various mavros topics
     */
    // vehicle pose from the FCU
    localPosSub = rosNode->subscribe("/mavros/local_position/local", 1000, &Locator::localPositionCallback, this);
    // target pose from the vision system
    targetPosSub = rosNode->subscribe("/target_pose", 1000, &Locator::targetPoseCallback, this);

    // Advertise the position topic to be published to the FCU
    localPosPub = rosNode->advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1000);

    ROS_INFO("^^^ ROS ON POSE PROXY INITIALIZED ^^^");

    return;
}

geometry_msgs::Pose Locator::getTargetCurrentENUWRTOrigin()
{
    geometry_msgs::Pose p = target_current_ENU.getPose();
    p.position.x -= target_origin_ENU.position.x;
    p.position.y -= target_origin_ENU.position.y;
    p.position.z -= target_origin_ENU.position.z;

    return p;
}

void Locator::localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    vehicle_current_ENU.setPose(msg->pose);
    //	printf("VEHICLE POSE CALLBACK yaw = %f\n", vehicle_current_ENU.attitude.yaw);
    if (!vehiclePoseInitialized)
    {
        if (targetPoseInitialized)
        {
            syncYawOrigins();
        }
    }
    vehiclePoseInitialized = true;
    ROS_DEBUG("<<<RECV LATT: r:%f p:%f y:%f",
              vehicle_current_ENU.attitude.roll,
              vehicle_current_ENU.attitude.pitch,
              vehicle_current_ENU.attitude.yaw);
    ROS_DEBUG("<<<RECV POS: x:%f y:%f z:%f",
              vehicle_current_ENU.position.x,
              vehicle_current_ENU.position.y,
              vehicle_current_ENU.position.z);
    // Let the controller know that the target position has been updated
    if (controller != NULL)
    {
        controller->vehiclePositionUpdated();
    }
}

void Locator::targetPoseCallback(const geometry_msgs::PoseArray::ConstPtr& msgArray)
{
    ROS_INFO_STREAM("targetPoseCallback");
	// Check to see if we have any targets
    if (msgArray->poses.size() > 0)
    {
        // We have a target to process it
    	geometry_msgs::Pose msg = msgArray->poses.at(0);
        // copy the message header since we need it when we forward it on to mavros
        visionPose.header = msgArray->header;

        if (!targetPoseInitialized)
        {
            // Target pose not initialized so initialize it
            target_current_RFU.setPose(msg);
            targetPoseInitialized = true;
            if (vehiclePoseInitialized)
            {
                syncYawOrigins();
            }
            else
            {
                targetYawOrigin = target_current_RFU.attitude.roll;
            }
        }
        else
        {
            // Update the target state with the filtered position
            geometry_msgs::Pose fPose = filterPosition(msg);
            target_current_RFU.setPose(fPose);
        }

        if (vehiclePoseInitialized)
        {
            // The vision system produces targets in the RFU frame so convert it to ENU
            convertTargetRFUtoENU();
            // Publish target location to the FCU
            updateFCULocation();
        }
        // Reset the consecutive frame error count
        positionErrorCnt = 0;
    }
    else
    {
        // No targets detected so potential error

        // reset the pose intitialization flag
        targetPoseInitialized = false;
        if (controller != NULL)
        {
            // If this is the first error frame detected then notify the controller
            // Note that you can change this constant it you want to notify the
            // controller after 'X' number of consecutive frame errors or perhaps
            // base the conditional on some time duration
            if (positionErrorCnt == 1)
            {
                controller->locationError();
            }
            positionErrorCnt++;
        }
    }
}

#define PI_2 1.5707963
void Locator::convertTargetRFUtoENU()
{
    //variables associated with the camera view
    double Xc, Yc, Dc, Ac;
    // Variables associated with the current vehicle state (i.e. yaw)
    double Ay;
    // Calculated Variables associated with the calculated target position in ENU
    double Xt, Yt, At;

    Xc = target_current_RFU.position.x;
    Yc = target_current_RFU.position.y;
    Dc = sqrt((Xc*Xc) + (Yc*Yc));
    Ay = vehicle_current_ENU.attitude.yaw + PI_2;

    if (Dc > 0)
    {
        Ac = asin(-Xc/Dc);
        At = Ay + Ac;
        Xt = Dc * cos(At);
        Yt = Dc * sin(At);
    }
    else
    {
        Xt = 0;
        Yt = 0;
    }

    // update the target ENU state
    target_current_ENU.position.x = Xt;
    target_current_ENU.position.y = Yt;
    target_current_ENU.position.z = target_current_RFU.position.z;
    target_current_ENU.setAttitude(target_current_RFU.getAttitude());
    //	printf("Xc:%f, Yc:%f, Dc:%f, Xt:%f, Yt:%f, YAW:%f, Ay:%f, Ac:%f, At:%f\n", Xc, Yc, Dc, Xt, Yt, vehicle_current_ENU.attitude.yaw, Ay, Ac, At);
    //			targetCurrentLocal.position.x,
    //			targetCurrentLocal.position.y,
    //			targetCurrentLocal.position.z,
    //			vehicleCurrent.attitude.yaw);
    //	printf("target ENU position = x,y,z %f, %f, %f\n",
    //			targetCurrentENU.position.x,
    //			targetCurrentENU.position.y,
    //			targetCurrentENU.position.z);
}

geometry_msgs::Point Locator::convertRFUtoENUoff(double right, double front,
                double up) {
        geometry_msgs::Point p;

        double u_off = up;
        double n_off = front;
        double e_off = -right;

        double total_xy = sqrt((e_off*e_off)+(n_off*n_off));
        double ang_off;
        if (total_xy != 0)
        {
                ang_off = atan2(e_off, n_off);
                double ang = ang_off + vehicle_current_ENU.attitude.yaw;
                n_off = total_xy * cos(ang);
                e_off = -total_xy * sin(ang);
        }

        p.x = e_off;
        p.y = n_off;
        p.z = u_off;
        return p;
}

geometry_msgs::Pose& Locator::filterPosition(geometry_msgs::Pose pose)
{
    filteredPose = pose;
    // Only apply the filter if the error count is 0 meaning we have at
    // least two consecutive frames without errors
    if (positionErrorCnt == 0)
    {
        deltaVehicleYaw = vehicle_current_ENU.attitude.roll - vehicleYawOrigin;
        deltaTargetYaw = target_current_RFU.attitude.roll - targetYawOrigin;
        double deltaYaw = deltaVehicleYaw + deltaTargetYaw;
        double x = pose.position.x + pose.position.y * sin(deltaYaw);
        filteredPose.position.x = (X_ALPHA * pose.position.x) +
                (X_BETA * filteredPose.position.x);
        filteredPose.position.y = (Y_ALPHA * pose.position.y) +
                (Y_BETA * filteredPose.position.y);
        filteredPose.position.z = (Z_ALPHA * pose.position.z) +
                (Z_BETA * filteredPose.position.z);
    }

    return filteredPose;
}

void Locator::updateFCULocation()
{
    // Get the current target position wrt to the vehicle and some predefined origin
    visionPose.pose = getTargetCurrentENUWRTOrigin();

    // Translate to the target frame
    visionPose.pose.position.x = -visionPose.pose.position.x;
    visionPose.pose.position.y = -visionPose.pose.position.y;
    visionPose.pose.position.z = -visionPose.pose.position.z;

    ROS_DEBUG("Update Vision Pose[%5.3f %5.3f %5.3f]",
              visionPose.pose.position.x,
              visionPose.pose.position.y,
              visionPose.pose.position.z);
    visionPose.header.stamp = ros::Time::now();
    location_update_time = visionPose.header.stamp;
    printf("Update Vision Pose %5.3f  %5.3f  %5.3f\n", visionPose.pose.position.x, visionPose.pose.position.y, visionPose.pose.position.z);

    // Send the position to the FCU
    localPosPub.publish(visionPose);

    return;
}

void Locator::syncYawOrigins()
{
    vehicleYawOrigin = vehicle_current_ENU.attitude.roll;
    targetYawOrigin = target_current_RFU.attitude.roll;
    ROS_DEBUG("Syncing YAW target, vehicle %f   %f\n", vehicleYawOrigin, targetYawOrigin);
}

