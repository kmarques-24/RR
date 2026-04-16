#include "estimator.h"

void update_odometry_msg(nav_msgs__msg__Odometry *odom_msg)
{
    // (*p).field = p->field
    odom_msg->header.stamp.sec = 0;
    odom_msg->header.stamp.nanosec = 0;

    odom_msg->pose.covariance[0] = 0;
    odom_msg->pose.pose.position.x = 0;
    odom_msg->pose.pose.position.y = 0;
    odom_msg->pose.pose.position.z = 0;
    odom_msg->pose.pose.orientation.w = 0; // quaternion
    odom_msg->pose.pose.orientation.x = 0;
    odom_msg->pose.pose.orientation.y = 0;
    odom_msg->pose.pose.orientation.z = 0;

    odom_msg->twist.covariance[0] = 0;
    odom_msg->twist.twist.angular.x = 0;
    odom_msg->twist.twist.angular.y = 0;
    odom_msg->twist.twist.angular.z = 0;
    odom_msg->twist.twist.linear.x = 0;
    odom_msg->twist.twist.linear.y = 0;
    odom_msg->twist.twist.linear.z = 0;
}