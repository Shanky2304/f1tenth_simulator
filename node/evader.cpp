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

    double safe_distance_threshold = 0.6;

    // Listen for odom & laser scan messages
    ros::Subscriber scan_sub;

    // Publish drive data
    ros::Publisher drive_pub;

    // previous desired steering angle
    double prev_angle=0.0;

    bool first_run = true;

    int64_t turn = 0;

public:
    Evader() {

        // Private node handle
        n = ros::NodeHandle("~");

        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        std::string evader_drive_topic, odom_topic, scan_topic;
        n.getParam("evader_drive_topic", evader_drive_topic);
        n.getParam("odom_topic", odom_topic);
        n.getParam("scan_topic", scan_topic);

        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle);

        // Pub
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(evader_drive_topic, 10, true);

        // Subs
        // odom_sub = n.subscribe(odom_topic, 1, &Evader::random_odom_callback, this);
        scan_sub = n.subscribe(scan_topic, 1, &Evader::scan_callback, this);

    }

    void publish_random_drive(int max_range_index, double ang_incr) {
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        drive_msg = get_random_drive(max_range_index, ang_incr);

        // set drive message in drive stamped message
        drive_st_msg.drive = drive_msg;

        // publish AckermannDriveStamped message to drive topic
        drive_pub.publish(drive_st_msg);
    }

    void scan_callback(const sensor_msgs::LaserScan & lc_msg) {
        
	if (turn++ % 5 != 0) {
            return;
        }   
    ROS_INFO_STREAM((turn-1) << "th turn.");
	ackermann_msgs::AckermannDriveStamped drive_st_msg;
    ackermann_msgs::AckermannDrive drive_msg;
    int num_of_beams = lc_msg.ranges.size();
	ROS_INFO_STREAM("num of beams = "<<num_of_beams);
	int max_range_index = std::max_element(lc_msg.ranges.begin(), lc_msg.ranges.end()) - lc_msg.ranges.begin();
	double ang_incr = lc_msg.angle_increment;
	ROS_INFO_STREAM("Almost half of num_beams = "<<max_steering_angle<<" "<<max_range_index);
//        int k = 0;
//        double blocked_paths_incr[num_of_beams];
//

        if (first_run) {
            ros::Duration(5).sleep();
            first_run = false;
            drive_msg.speed = 2.0;
            drive_msg.steering_angle = 0.0;

            // set drive message in drive stamped message
            drive_st_msg.drive = drive_msg;

            // publish AckermannDriveStamped message to drive topic
            drive_pub.publish(drive_st_msg);
        }

        // Below loop records the burst of angle increments that are not safe to proceed in.
        // E.g. - blocked_paths_incr[0] = 4 & blocked_paths_incr[1] = 10,
        // min + (4 * ang_incr) to min + (4 * ang_incr) are blocked angles for the car.
//        for (int i = 0; i < num_of_beams; i++) {
//            if (lc_msg.ranges[i] < safe_distance_threshold) {
//                // blocked incr. begin
//                blocked_paths_incr[k++] = i;
//                for (int j = i; j < num_of_beams; j++) {
//                    if (lc_msg.ranges[j] > safe_distance_threshold && j == num_of_beams)
//                        blocked_paths_incr[k] = j;
//                    else if (lc_msg.ranges[j] > safe_distance_threshold) {
//                        // blocked incr. end
//                        blocked_paths_incr[k++] = j - 1;
//                        break;
//                    }
//                }
//            }
//        }

        // For each beam reading make a decision
        for (int i = 0; i < num_of_beams; i++) {
            if (lc_msg.ranges[i] < safe_distance_threshold) {
                // collision about to happen set drive to 0.
                ROS_INFO_STREAM("Max range = " << lc_msg.ranges[max_range_index]);
                stop_car();
                // call random walker logic to get new Ackermann drive
                publish_random_drive(max_range_index, ang_incr);
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
        //ros::Duration(0.3).sleep();
	prev_angle = 0.0;
    }

    ackermann_msgs::AckermannDrive get_random_drive(int max_range_index, double ang_incr) {
        ackermann_msgs::AckermannDrive drive_msg;
        drive_msg.speed = 2.0;

//        /// STEERING ANGLE CALCULATION
//        // random number between 0 and 1
//        double random = ((double) rand() / RAND_MAX);
//        // good range to cause lots of turning
//        double range = max_steering_angle / 2.0;
//        // compute random amount to change desired angle by (between -range and range)
//        double rand_ang = range * random - range / 2.0;
//
//        // sometimes change sign so it turns more (basically add bias to continue turning in current direction)
//        random = ((double) rand() / RAND_MAX);
//        if ((random > .8) && (prev_angle != 0)) {
//            double sign_rand = rand_ang / std::abs(rand_ang);
//            double sign_prev = prev_angle / std::abs(prev_angle);
//            rand_ang *= sign_rand * sign_prev;
//        }
//
//        // set angle (add random change to previous angle)
//        int blocked_range_size = blocked_ang_incr.size();
//        for (int i = 0; i < blocked_range_size; i++) {
//
//        }

        double steering_angle = std::min(std::max(-max_steering_angle + (max_range_index * ang_incr), -max_steering_angle)
                , max_steering_angle);
	ROS_INFO_STREAM(max_range_index<<" is max range index; "<<ang_incr<<" is ang_incr.");
        ROS_INFO_STREAM("Steering angle decided = "<<steering_angle);
        drive_msg.steering_angle = steering_angle;

        return drive_msg;
    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "evader");
    Evader evader;
    ros::spin();
    return 0;
}
