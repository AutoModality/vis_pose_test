/*
 * pose_proxy.cpp
 *
 */

#include "locator.h"

Locator::Locator(ros::NodeHandle* n, Controller* c)
{
    controller_ = c;
    if (controller_ != NULL)
    {
        controller_->setLocator(this);
    }
    initROS(n);
}

Locator::~Locator()
{
    // TODO Auto-generated destructor stub
}

void Locator::initROS(ros::NodeHandle *n)
{
    ros_node_ = n;

    /*
     * Subscribe to various mavros topics
     */
    // vehicle pose from the FCU
    local_pos_sub_ = ros_node_->subscribe("/mavros/local_position/local", 1000, &Locator::localPositionCallback, this);
    // target pose from the vision system
    target_pos_sub_ = ros_node_->subscribe("/target_pose", 1000, &Locator::targetPoseCallback, this);

    // Advertise the position topic to be published to the FCU
    local_pos_pub_ = ros_node_->advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1000);

    ROS_INFO("^^^ ROS ON POSE PROXY INITIALIZED ^^^");

    return;
}

geometry_msgs::Pose Locator::getTargetCurrentENUWRTOrigin()
{
    geometry_msgs::Pose p = target_current_ENU_.getPose();
    p.position.x -= target_origin_ENU_.position.x;
    p.position.y -= target_origin_ENU_.position.y;
    p.position.z -= target_origin_ENU_.position.z;

    return p;
}

void Locator::localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    vehicle_current_ENU_.setPose(msg->pose);
    //    printf("VEHICLE POSE CALLBACK yaw = %f\n", vehicle_current_ENU.attitude.yaw);
//    if (!vehiclePoseInitialized)
//    {
//        if (targetPoseInitialized)
//        {
//            syncVehicleandTargetPose();
//        }
//    }
    vehiclePoseInitialized_ = true;
    ROS_DEBUG("<<<RECV LATT: r:%f p:%f y:%f",
              vehicle_current_ENU_.attitude.roll,
              vehicle_current_ENU_.attitude.pitch,
              vehicle_current_ENU_.attitude.yaw);
    ROS_DEBUG("<<<RECV POS: x:%f y:%f z:%f",
              vehicle_current_ENU_.position.x,
              vehicle_current_ENU_.position.y,
              vehicle_current_ENU_.position.z);

    // Let the controller know that the target position has been updated
    if (controller_ != NULL)
    {
        controller_->vehiclePositionUpdated();
    }
}

void Locator::targetPoseCallback(const geometry_msgs::PoseArray::ConstPtr& msgArray)
{
	// Check to see if we have any targets
    if (msgArray->poses.size() > 0)
    {
        // We have a target to process
    	geometry_msgs::Pose msg = msgArray->poses.at(0);
        // copy the message header since we need it when we forward it on to mavros
        vision_pose_.header = msgArray->header;

        if (!targetPoseInitialized_)
        {
            // Target pose not initialized so initialize it
            target_current_FLU_.setPose(msg);
            targetPoseInitialized_ = true;
//            if (vehiclePoseInitialized)
//            {
//                syncVehicleandTargetPose();
//            }
//            else
//            {
//                targetYawOrigin = getTargetYaw();
//            }
        }
        else
        {
        	/*
        	 * Note that in this version the filtering of the vision input is commented out to allow
        	 * for unadulterated vision inputs. If one wanted to play with filtering then
        	 * they should uncomment this code and play with the filter appropriately
        	 */
            // Update the target state with the filtered position
//        	geometry_msgs::Pose fPose = filterPosition(msg);
//        	target_current_FLU_.setPose(fPose);

        	// No filtering
        	target_current_FLU_.setPose(msg);
        }
        ROS_DEBUG("GOT TARGET FLU[F:%0.3f L:%0.3f U:%0.3f, R:%0.3f P:%0.3f Y:%0.3f, ]",
        		target_current_FLU_.position.x,
				target_current_FLU_.position.y,
				target_current_FLU_.position.z,
				target_current_FLU_.attitude.roll,
				target_current_FLU_.attitude.pitch,
				target_current_FLU_.attitude.yaw);

        if (vehiclePoseInitialized_)
        {
            // The vision system produces targets in the FLU frame so convert it to ENU
            convertTargetFLUtoENU();
            // Publish target location to the FCU
            updateFCULocation();
        }

        // Notify the controller that the target position has been updated
        if (controller_ != NULL)
        {
        	controller_->targetPositionUpdated();
        }

        // Reset the consecutive frame error count
        position_error_cnt_ = 0;
    }
    else
    {
        // No targets detected so potential error

        // reset the pose intitialization flag
        targetPoseInitialized_ = false;
        if (controller_ != NULL)
        {
            // If this is the first error frame detected then notify the controller
            // Note that you can change this constant it you want to notify the
            // controller after 'X' number of consecutive frame errors or perhaps
            // base the conditional on some time duration
            if (position_error_cnt_ == 1)
            {
                controller_->locationError();
            }
            position_error_cnt_++;
        }
    }
}

void Locator::convertTargetFLUtoENU()
{
    //variables associated with the camera view
    double Xc, Yc, Dc, Ac;
    // Variables associated with the current vehicle state (i.e. yaw)
    double Ay;
    // Calculated Variables associated with the calculated target position in ENU
    double Xt, Yt, At;

    Xc = target_current_FLU_.position.x;
    Yc = target_current_FLU_.position.y;
    Dc = sqrt((Xc*Xc) + (Yc*Yc));
    Ay = vehicle_current_ENU_.attitude.yaw;

    if (Dc > 0)
    {
        Ac = asin(Yc/Dc);
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
    target_current_ENU_.position.x = Xt;
    target_current_ENU_.position.y = Yt;
    target_current_ENU_.position.z = target_current_FLU_.position.z;
    target_current_ENU_.setAttitude(target_current_FLU_.getAttitude());
//    printf("Xc:%0.2f, Yc:%0.2f, Dc:%0.2f, Xt:%0.2f, Yt:%0.2f, YAW:%0.2f, Ay:%0.2f, Ac:%0.2f, At:%0.2f\n",
//    		Xc, Yc, Dc, Xt, Yt, vehicle_current_ENU.attitude.yaw, Ay, Ac, At);
}

geometry_msgs::Point Locator::convertFLUtoENUoff(double forward, double left,
                double up)
{
	geometry_msgs::Point p;

	double u_off = up;
	double n_off = forward;
	double e_off = left;

	double total_xy = sqrt((e_off*e_off)+(n_off*n_off));
	if (total_xy != 0)
	{
		double ang_off;
		ang_off = atan2(e_off, n_off);
		double ang = ang_off + vehicle_current_ENU_.attitude.yaw;
		n_off = total_xy * cos(ang);
		e_off = total_xy * sin(ang);
//		printf("X:%0.2f, Y:%0.2f, D:%0.2f, Xt:%0.2f, Yt:%0.2f, YAW:%0.2f, Ang:%0.2f, At:%0.2f\n",
//				forward, left, total_xy, n_off, e_off, vehicle_current_ENU.attitude.yaw, ang_off, ang);
	}

	p.x = n_off;
	p.y = e_off;
	p.z = u_off;
	return p;
}

geometry_msgs::Pose& Locator::filterPosition(geometry_msgs::Pose pose)
{
    // Only apply the filter if the error count is 0 meaning we have at
    // least two consecutive frames without errors
    if (position_error_cnt_ == 0)
    {
    	filtered_pose_.position.x = (X_ALPHA * pose.position.x) +
                (X_BETA * filtered_pose_.position.x);
    	filtered_pose_.position.y = (Y_ALPHA * pose.position.y) +
                (Y_BETA * filtered_pose_.position.y);
    	filtered_pose_.position.z = (Z_ALPHA * pose.position.z) +
                (Z_BETA * filtered_pose_.position.z);
    }

    return filtered_pose_;
}

void Locator::updateFCULocation()
{
    // Get the current target position wrt to the vehicle and some predefined origin
    vision_pose_.pose = getTargetCurrentENUWRTOrigin();

    // Translate to the target frame
    vision_pose_.pose.position.x = -vision_pose_.pose.position.x;
    vision_pose_.pose.position.y = -vision_pose_.pose.position.y;
    vision_pose_.pose.position.z = -vision_pose_.pose.position.z;
    ROS_DEBUG("Update Vision Pose[%5.3f %5.3f %5.3f]",
    		vision_pose_.pose.position.x,
			vision_pose_.pose.position.y,
			vision_pose_.pose.position.z);

    vision_pose_.header.stamp = ros::Time::now();
    location_update_time_ = vision_pose_.header.stamp;

    // Send the position to the FCU
    local_pos_pub_.publish(vision_pose_);

    return;
}

//void Locator::syncVehicleandTargetPose()
//{
//    vehicleYawOrigin = vehicle_current_ENU.attitude.yaw;
//    targetYawOrigin = getTargetYaw();
//    ROS_DEBUG("Syncing YAW target, vehicle %f   %f\n", vehicleYawOrigin, targetYawOrigin);
//}

