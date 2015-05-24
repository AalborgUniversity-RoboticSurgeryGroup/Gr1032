#include "demo_gr1032.h"

/*** include libraries ***/
#include <string>
#include <vector>
#include <iostream> 
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <time.h> 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>

void demo_func() {
    std::cout << "running demo.." << std::endl;

    ros::NodeHandle node;
    ros::Publisher setpoints_pub_pitch = node.advertise<std_msgs::Float64>("pitch_command", 1);
    ros::Publisher setpoints_pub_slide = node.advertise<std_msgs::Float64>("slide_command", 1);
    ros::Publisher setpoints_pub_roll = node.advertise<std_msgs::Float64>("roll_command", 1);

    long int k = 0;
    while(1) {
         /*** prepare to publish control signal ***/ 
        std::vector<double> ctr;
        std::cout << "iter = " << k <<  std::endl; 

        ctr.push_back(0);
        ctr.push_back(0);
        ctr.push_back(0);
       
        if (k > 1) {
            ctr.at(0) = 0.5;
            ctr.at(1) = 0.0;
            ctr.at(2) = 0.09;
        }
        if (k > 2) {
            ctr.at(0) = 0.1;
            ctr.at(1) = 1.3;
            ctr.at(2) = 0.08;
        }
        if (k > 3) {
            ctr.at(0) = 0.1;
            ctr.at(1) = -1.3;
            ctr.at(2) = -0.08;
        }
        if (k > 4) {
            ctr.at(0) = 0.7;
            ctr.at(1) = -1.3;
            ctr.at(2) = 0.0;
        }
        if (k > 5) {
            ctr.at(0) = 0.70;    // p4_hand_pitch
            ctr.at(1) = 1.3;     // p4_hand_roll
            ctr.at(2) = 0.00;    // p4_instrument_slide
        }
        if (k > 7) {
            ctr.at(0) = 0.70;    // p4_hand_pitch
            ctr.at(1) = 1.3;     // p4_hand_roll
            ctr.at(2) = 0.09;    // p4_instrument_slide
        }
        if (k > 8) {
            ctr.at(0) = 0.70;     // p4_hand_pitch
            ctr.at(1) = 1.3;      // p4_hand_roll
            ctr.at(2) = -0.09;    // p4_instrument_slide
        }
        if (k > 9) {
            ctr.at(0) = 0.70;     // p4_hand_pitch
            ctr.at(1) = -1.3;     // p4_hand_roll
            ctr.at(2) = -0.09;    // p4_instrument_slide
        }
        if (k > 11) {
            ctr.at(0) = -0.70;    // p4_hand_pitch
            ctr.at(1) = -1.3;     // p4_hand_roll
            ctr.at(2) = -0.09;    // p4_instrument_slide
        }
        if (k > 12) {
            ctr.at(0) = -0.0;    // p4_hand_pitch
            ctr.at(1) = -0.0;     // p4_hand_roll
            ctr.at(2) = -0.0;    // p4_instrument_slide
        }
        if (k > 13) {
            ctr.at(0) = -0.0;    // p4_hand_pitch
            ctr.at(1) = -0.0;     // p4_hand_roll
            ctr.at(2) =  0.01;    // p4_instrument_slide
        }

        if (k > 13) {
            k = 0;
        }

        std::cout << "ctr[0] = " << ctr.at(0) << std::endl;
        std::cout << "ctr[1] = " << ctr.at(1) << std::endl;
        std::cout << "ctr[2] = " << ctr.at(2) << std::endl;
        std::cout << "\n";

        /*** execute p4_hand_pith ***/
        std_msgs::Float64 u_msg_0;
        u_msg_0.data = ctr.at(0);
        setpoints_pub_pitch.publish(u_msg_0);

        /*** execute p4_hand_roll ***/
        std_msgs::Float64 u_msg_1;
        u_msg_1.data = ctr.at(1);
        setpoints_pub_roll.publish(u_msg_1);

        /*** execute p4_hand_slide ***/
        std_msgs::Float64 u_msg_2;
        u_msg_2.data = ctr.at(2);
        setpoints_pub_slide.publish(u_msg_2);

        k += 1;
        sleep(2);
    }    
}
