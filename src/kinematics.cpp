/*
 * Kinematics.cpp
 *
 */

#include "kinematics.h"

Kinematics::Kinematics() {
    _init();
}

Kinematics::Kinematics(const geometry_msgs::Pose& p) {
    _init();
    setPose(p);
}

Kinematics::Kinematics(const geometry_msgs::Pose& p, const std::string id)  {
    _init();
    setPose(p);
    object_id = id;
}

Kinematics::~Kinematics() {
    // TODO Auto-generated destructor stub
}

void Kinematics::_init() {
    origin.x = 0.0;
    origin.y = 0.0;
    origin.z = 0.0;
}

void Kinematics::setPose(const geometry_msgs::Pose& p) {
    position = p.position;
    setAttitude(p.orientation);
    setPoseInitialized(true);
}

geometry_msgs::Pose&  Kinematics::getPose() {
    _p.position = position;
    _p.orientation = orientation;
    return(_p);
}

void Kinematics::setPosition(const geometry_msgs::Point& p) {
    position = p;
    setPoseInitialized(true);
}

void Kinematics::setPosition(double x, double y, double z) {
    position.x = x;
    position.y = y;
    position.z = z;
    setPoseInitialized(true);
}

void Kinematics::setOrigin(const geometry_msgs::Point& p) {
    origin = p;
}

void Kinematics::setOrigin(double x, double y, double z) {
    origin.x = x;
    origin.y = y;
    origin.z = z;
}

void Kinematics::setAttitude(const VehicleAttitude& va) {
    attitude = va;
    orientation = tf::createQuaternionMsgFromRollPitchYaw(attitude.roll, attitude.pitch, attitude.yaw);
    setPoseInitialized(true);
}

void Kinematics::setAttitude(double roll, double pitch, double yaw, double throttle) {
    attitude.roll = roll;
    attitude.pitch = pitch;
    attitude.yaw = yaw;
    attitude.throttle = throttle;
    orientation = tf::createQuaternionMsgFromRollPitchYaw(attitude.roll, attitude.pitch, attitude.yaw);
    setPoseInitialized(true);
}

void Kinematics::setAttitude(geometry_msgs::Quaternion q) {
    double throttle = attitude.throttle;
    attitude = quaternionMsg2Att(q);
    attitude.throttle = throttle;
    orientation = q;
    //  ROS_INFO("<<<GLOBAL POS: [x-%f y-%f z-%f w-%f]",
    //	   msg->pose.pose.orientation.x,
    //	   msg->pose.pose.orientation.y,
    //	   msg->pose.pose.orientation.z,
    //	   msg->pose.pose.orientation.w);
    //	ROS_INFO("<<<RECV LATT: [r-%f p-%f y-%f",
    //			vehicleAttitudeCurrent.roll,
    //			vehicleAttitudeCurrent.pitch,
    //			vehicleAttitudeCurrent.yaw);
    //publishRPY(currentAttPub, vehicleAttitudeCurrent.roll, vehicleAttitudeCurrent.pitch, vehicleAttitudeCurrent.yaw, throttleCurrent.data);
    //	vehiclePoseInitialized = true;
    setPoseInitialized(true);
}

void Kinematics::setVelocity(const geometry_msgs::Vector3& linear,
                             const geometry_msgs::Vector3& angular) {
    setLinearVelocity(linear);
    setAngularVelocity(angular);
}

void Kinematics::setAngularVelocity(const geometry_msgs::Vector3& vel) {
    velAngular = vel;
}

void Kinematics::setAngularVelocity(double x, double y, double z) {
    velAngular.x = x;
    velAngular.y = y;
    velAngular.z = z;
}

void Kinematics::setLinearVelocity(const geometry_msgs::Vector3& vel) {
    velLinear = vel;
}

void Kinematics::setLinearVelocity(double x, double y, double z) {
    velLinear.x = x;
    velLinear.y = y;
    velLinear.z = z;
}

/*
void Kinematics::publishTransform() {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(position.x, position.y, position.z) );
    tf::Quaternion q;
    tf::quaternionMsgToTF(orientation, q);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "target"));
}
*/
void Kinematics::test() {
    double roll = 1.0;
    double pitch = 1.1;
    double yaw = 1.2;
    double throttle = 3.1;
    geometry_msgs::PoseStamped p;
    p.pose.position.x = 5.1;
    p.pose.position.y = 5.2;
    p.pose.position.z = 5.3;
    p.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    geometry_msgs::Vector3 vl;
    vl.x = 10.0;
    vl.y = 10.1;
    vl.z = 10.2;
    geometry_msgs::Vector3 van;
    van.x = 20.0;
    van.y = 20.1;
    van.z = 20.2;

    printf("Kinematics testing roll:%f, pitch:%f, yaw:%f, throttle:%f\n",
           roll, pitch, yaw, throttle);
    printf("          Angular velocity x:%f, y:%f, z:%f\n", vl.x, vl.y, vl.z);
    printf("          Linear velocity x:%f, y:%f, z:%f\n", van.x, van.y, van.z);

    printf("Kinematics: setPosition - x,y,z\n");
    setPosition(p.pose.position.x, p.pose.position.y, p.pose.position.z);
    print();

    printf("Kinematics: setPosition - position\n");
    setPosition(p.pose.position);
    print();

    printf("Kinematics: setPose\n");
    setPose(p);
    print();

    printf("Kinematics: setAttitude - va\n");
    VehicleAttitude va;
    va.roll = roll;
    va.pitch = pitch;
    va.yaw = yaw;
    va.throttle = throttle;
    setAttitude(va);
    print();

    printf("Kinematics: setAttitude - RPY\n");
    setAttitude(roll, pitch, yaw, throttle);
    print();

    printf("Kinematics: setAttitude - quaternion\n");
    setAttitude(p.pose.orientation);
    print();

    printf("Kinematics: setVelocity - linear, angular\n");
    setVelocity(vl, van);
    print();

    printf("Kinematics: setAngularVelocity - angular\n");
    setAngularVelocity(van);
    print();

    printf("Kinematics: setAngularVelocity - x, y, z\n");
    setAngularVelocity(van.x, van.y, van.z);
    print();
}

const geometry_msgs::Point& Kinematics::getGlobalPosition() {
    _global_position.x = position.x - origin.x;
    _global_position.y = position.y - origin.y;
    _global_position.z = position.x - origin.z;

    return _global_position;
}

double Kinematics::pointDistance(geometry_msgs::Point p1, geometry_msgs::Point p2) {
    double xd = p2.x - p1.x;
    double yd = p2.y - p1.y;
    double zd = p2.z - p1.z;
    return sqrt((xd*xd) + (yd*yd) + (zd*zd));
}

VehicleAttitude Kinematics::quaternionMsg2Att(geometry_msgs::Quaternion q) {
    tf::Quaternion quat;
    tf::quaternionMsgToTF(q, quat);
    VehicleAttitude va {0.0,0.0,0.0,0.0};
    tf::Matrix3x3(quat).getRPY(va.roll, va.pitch, va.yaw);
    return va;
}

void Kinematics::print() {
    printf("Kinematics position: x:%f, y:%f, z:%f\n", position.x, position.y, position.z);
    printf("           orientation: x:%f, y:%f, z:%f, w:%f\n", orientation.x, orientation.y, orientation.z, orientation.w);
    printf("           Attitude: roll:%f, pitch:%f, yaw:%f, throttle:%f\n",
           attitude.roll, attitude.pitch, attitude.yaw, attitude.throttle);
    printf("           linear veloctiy: x:%f, y:%f, z:%f\n", velLinear.x, velLinear.y, velLinear.z);
    printf("           Angular veloctiy: x:%f, y:%f, z:%f\n", velAngular.x, velAngular.y, velAngular.z);
}
