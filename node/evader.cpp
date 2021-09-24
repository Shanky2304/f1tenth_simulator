//
// Created by Vivek Shankar on 9/23/21.
//

#include <ros/ros.h>

#include "ackermann_msgs/AckermannDriveStamped.h"

#include "ackermann_msgs/AckermannDrive.h"

#include "nav_msgs/Odometry.h"

// for printing
#include <iostream>

// for RAND_MAX
#include <cstdlib>

class Evader {
private:
    ros::NodeHandle n;

    double max_speed, max_steering_angle;

    // Listen for odom & laser scan messages
    ros::Subscriber odom_sub, scan_sub;

    // Publish drive data
    ros::Publisher drive_pub;

    // previous desired steering angle
    double prev_angle=0.0;

public:
    Evader() {

        n = ros::NodeHandle("~");

        std::string evader_drive_topic, odom_topic, scan_topic;
        n.getParam("evader_drive_topic", evader_drive_topic);
        n.getParam("odom_topic", odom_topic);
        n.getParam("scan_topic", scan_topic);

        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle);

        // Pub
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(evader_drive_topic, 10);

        // Subs
        // odom_sub = n.subscribe(odom_topic, 1, &Evader::odom_callback, this);
        // scan_sub = n.subscribe(scan_topic, 1, &Evader::scan_callback, this);

    }

    void odom_callback(const nav_msgs::Odometry & msg){

        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        drive_msg.speed = 2.0;

        /// STEERING ANGLE CALCULATION
        // random number between 0 and 1
        double random = ((double) rand() / RAND_MAX);
        // good range to cause lots of turning
        double range = max_steering_angle / 2.0;
        // compute random amount to change desired angle by (between -range and range)
        double rand_ang = range * random - range / 2.0;

        // sometimes change sign so it turns more (basically add bias to continue turning in current direction)
        random = ((double) rand() / RAND_MAX);
        if ((random > .8) && (prev_angle != 0)) {
            double sign_rand = rand_ang / std::abs(rand_ang);
            double sign_prev = prev_angle / std::abs(prev_angle);
            rand_ang *= sign_rand * sign_prev;
        }

        // set angle (add random change to previous angle)
        drive_msg.steering_angle = std::min(std::max(prev_angle + rand_ang, -max_steering_angle), max_steering_angle);

        // reset previous desired angle
        prev_angle = drive_msg.steering_angle;

        // set drive message in drive stamped message
        drive_st_msg.drive = drive_msg;

        // publish AckermannDriveStamped message to drive topic
        drive_pub.publish(drive_st_msg);
    }

    void scan_callback() {
        // argument should be like const sensor_msgs::..
        // Here when something is very close we need to stop (set speed to zero) and then invoke odom_callback somehow.
    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "evader");
    Evader evader;
    ros::spin();
    return 0;
}