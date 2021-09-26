//
// Created by Vivek Shankar on 9/23/21.
//

#include <ros/ros.h>

#include "ackermann_msgs/AckermannDriveStamped.h"

#include "ackermann_msgs/AckermannDrive.h"

#include "nav_msgs/Odometry.h"

#include "sensor_msgs/LaserScan.h"

// for printing
#include <iostream>

// for RAND_MAX
#include <cstdlib>

class Evader {
private:
    ros::NodeHandle n;

    double max_speed, max_steering_angle;

    float safe_distance_threshold = 1.0;

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
        // odom_sub = n.subscribe(odom_topic, 1, &Evader::random_odom_callback, this);
        scan_sub = n.subscribe(scan_topic, 1, &Evader::scan_callback, this);

    }

    void random_odom_callback(const nav_msgs::Odometry & msg){

        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        drive_msg = get_random_drive();

        // set drive message in drive stamped message
        drive_st_msg.drive = drive_msg;

        // publish AckermannDriveStamped message to drive topic
        drive_pub.publish(drive_st_msg);
    }

    void scan_callback(const sensor_msgs::LaserScan& lc_msg) {
        // Here when something is very close we need to stop (set speed to zero) and then invoke odom_callback somehow.
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        int num_of_beams = lc_msg.ranges.size();
        float ranges_[num_of_beams];


        for (int i = 0; i < num_of_beams; i++) {
            if (lc_msg.ranges[i] < safe_distance_threshold) {
                // collision about to happen set drive to 0.
                stop_car();
                // call random walker logic to get new Ackermann drive
                nav_msgs::Odometry dummy;
                random_odom_callback(dummy);
                break;
            }
        }

    }

    void stop_car() {
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        drive_msg.speed = 0.0;
        drive_msg.acceleration = 0.0;
        drive_msg.steering_angle = 0.0;
        drive_msg.steering_angle_velocity = 0.0;

        // set drive message in drive stamped message
        drive_st_msg.drive = drive_msg;

        // publish AckermannDriveStamped message to drive topic
        drive_pub.publish(drive_st_msg);
    }

    ackermann_msgs::AckermannDrive get_random_drive() {
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

        return drive_msg;
    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "evader");
    Evader evader;
    ros::spin();
    return 0;
}