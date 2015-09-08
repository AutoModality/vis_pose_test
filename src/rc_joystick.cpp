/*
 * rc_joystick.cpp
 *
 *  Created on: Apr 24, 2015
 *      Author: dan
 */


#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Joy.h>
#include <mavros/RCIn.h>

#define NODE_NAME 			"rc_joystick"

#define RC_LOW_LEVEL		1100
#define RC_MID_LEVEL		1500
#define RC_HIGH_LEVEL		1900
#define RC_RANGE			(RC_HIGH_LEVEL - RC_LOW_LEVEL)
#define RC_RANGE_BY_2		(RC_RANGE / 2)
#define RC_SWITCH2_0_THRESH	1500
#define RC_SWITCH3_0_THRESH	1300
#define RC_SWITCH3_1_THRESH	1700
#define RC_DEADBAND			0.05

// mavros::RCIn
#define RC_THROTTLE			2 // low at bottom
#define RC_YAW				3 // low at right
#define RC_PITCH			1 // low at bottom
#define RC_ROLL				0 // low at right
#define RC_GEAR				4 // 0=1099, 1 = 1900
#define RC_FLAP				5 // 0=1099, 1 = 1500, 2 = 1900
#define RC_AUX2				6 // 0=1099, 1 = 1500, 2 = 1900
#define RC_AUX3				7 // low clockwise

// sensor_msgs::Joy mapping
#define JOY_AXES_THROTTLE	3
#define JOY_AXES_YAW		2
#define JOY_AXES_PITCH		1
#define JOY_AXES_ROLL		0
#define JOY_AXES_AUX		4
#define JOY_BUTTON_0		0
#define JOY_BUTTON_1		1
#define JOY_BUTTON_2		2

using namespace std;

class AMRCJoystick
{
private:
	ros::NodeHandle nh_;
	sensor_msgs::Joy joy_;
	ros::Subscriber rc_in_sub_;
	ros::Publisher joy_pub_;

public:

AMRCJoystick() :
	nh_("~")
{
	rc_in_sub_ = nh_.subscribe("/mavros/rc/in", 1000, &AMRCJoystick::rcInCB, this);

    joy_pub_ = nh_.advertise<sensor_msgs::Joy>("/joy", 1000);

	// resize joy
	joy_.axes.resize(6);
	joy_.buttons.resize(12);
}

float scaleRC(uint16_t value, bool reversed)
{
	float scaled = ((float)value - RC_MID_LEVEL) / RC_RANGE_BY_2;
	if(scaled < RC_DEADBAND && scaled > -RC_DEADBAND)
	{
		scaled = 0.0;
	}
	else if(scaled > (1.0 - RC_DEADBAND))
	{
		scaled = 1.0;
	}
	else if(scaled < (-1.0 + RC_DEADBAND))
	{
		scaled = -1.0;
	}
	scaled *= reversed ? -1 : 1;
	return (float)scaled;
}

int switchRC3(uint16_t value)
{
	if(value < RC_SWITCH3_0_THRESH)
	{
		return 0;
	}
	else if(value < RC_SWITCH3_1_THRESH)
	{
		return 1;
	}
	else
	{
		return 2;
	}
}

int switchRC2(uint16_t value)
{
	if(value < RC_SWITCH2_0_THRESH)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

void rcInCB(const ros::MessageEvent<mavros::RCIn const>& event)
{
	const std::string& sender = event.getPublisherName();
	const ros::M_string& header = event.getConnectionHeader();
	ros::Time receipt_time = event.getReceiptTime();
	const mavros::RCIn::ConstPtr& msg = event.getMessage();
	const mavros::RCIn rc_in = *(msg.get());

	joy_.axes[JOY_AXES_THROTTLE] = scaleRC(rc_in.channels[RC_THROTTLE], false);
	joy_.axes[JOY_AXES_YAW] = scaleRC(rc_in.channels[RC_YAW], false);
	joy_.axes[JOY_AXES_PITCH] = scaleRC(rc_in.channels[RC_PITCH], false);
	joy_.axes[JOY_AXES_ROLL] = scaleRC(rc_in.channels[RC_ROLL], false);
	joy_.axes[JOY_AXES_AUX] = scaleRC(rc_in.channels[RC_AUX3], false);
	joy_.buttons[JOY_BUTTON_0] = switchRC2(rc_in.channels[RC_GEAR]);
	joy_.buttons[JOY_BUTTON_1] = switchRC3(rc_in.channels[RC_FLAP]);
	joy_.buttons[JOY_BUTTON_2] = switchRC3(rc_in.channels[RC_AUX2]);

	joy_.header.stamp = msg.get()->header.stamp;
	joy_pub_.publish(joy_);
}


};

int main(int argc, char **argv)
{
	ros::init(argc, argv, NODE_NAME);

	AMRCJoystick node;

	ros::spin();
}
