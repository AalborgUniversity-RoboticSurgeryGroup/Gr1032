#include "ik_gr1032.h"

/*** include libraries ***/
#include <string>
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <iostream> 
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <time.h> 
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>

#define pi 3.14159

void ik_pos() {
    std::cout << "Inverse Kinematic Mode!" << std::endl;
    /*** prepare to publish control signal ***/ 
    ros::NodeHandle node_pub;
    ros::Publisher setpoints_pub_pitch = node_pub.advertise<std_msgs::Float64>("pitch_command", 1);
    ros::Publisher setpoints_pub_slide = node_pub.advertise<std_msgs::Float64>("slide_command", 1);
    ros::Publisher setpoints_pub_roll = node_pub.advertise<std_msgs::Float64>("roll_command", 1);
    ros::Publisher setpoints_pub_inst_roll = node_pub.advertise<std_msgs::Float64>("inst_roll_command", 1);
    ros::Publisher setpoints_pub_inst_pitch = node_pub.advertise<std_msgs::Float64>("inst_pitch_command", 1);
    ros::Publisher setpoints_pub_inst_jaw_right = node_pub.advertise<std_msgs::Float64>("inst_jaw_right_command", 1);

    ros::NodeHandle node;
    KDL::Tree my_tree;
    std::string robot_desc_string;
    node.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, my_tree))
    {
       ROS_ERROR("Failed to construct kdl tree");
    }

    /*** use modified geometry ***/
    KDL::Chain my_chain;
    std::string root_link("p4_rcm_base");
    std::string tip_link("needle_driver_jawbone_right");
    if (!my_tree.getChain(root_link, tip_link, my_chain))
    {
        ROS_ERROR("Failed to get chain from tree");
    }

    for (unsigned int i = 0; i < my_chain.getNrOfSegments(); ++i)
    {
        std::cout << my_chain.getSegment(i).getName() << "(" << my_chain.getSegment(i).getJoint().getName() << ")" << std::endl;
    }

    /*** Create solver based on kinematic chain ***/
    KDL::ChainFkSolverPos_recursive fksolver(my_chain);
    KDL::ChainIkSolverVel_pinv iksolverv(my_chain);
    KDL::ChainIkSolverPos_NR iksolver = KDL::ChainIkSolverPos_NR(my_chain,fksolver,iksolverv,100,1e-6);

    KDL::JntArray q(my_chain.getNrOfJoints());
    KDL::JntArray q_init(my_chain.getNrOfJoints());

    while(1) {
        /*** Set destination frame ***/
        double x, y, z;
        std::cout << "Set end-effector position <x y z>:" << std::endl;
        std::cin >> x >> y >> z;

        /*** translation is offset from p4_rcm_base ***/
        x = x + 0.482; //0.494;
        y = y + 0;
        z = z - 0.059;
        KDL::Vector dest_pos(x,y,z);
        KDL::Frame dest_frame(dest_pos);

        // Compute!
        int ret = iksolver.CartToJnt(q_init,dest_frame,q);

        for (unsigned int i = 0; i < q.rows(); ++i)
        {
            std::cout << "Joint #" << i << ": " << q(i) << std::endl;
        }

        std::vector<double> control_signals;

        if (q(0) > 1.5) {
            q(0) = 1.5;
        }
        else if (q(0) < -1.5) {
            q(0) = -1.5;
        } 
        if (q(1) > 0.7) {
            q(1) = 0.7;
        }
        else if (q(1) < -0.7) {
            q(1) = -0.7;
        } 
        if (q(2) > 0.097) {
            q(2) = 0.097;
        }
        else if (q(2) < -0.097) {
            q(2) = -0.097;
        } 

        int j = 0; // control number of adjustments
        int i = -1; // control sign on inst_roll
        /*** adjust roll ***/
        while (q(3) > pi) {
            q(3) = -(pi - (q(3) - pi));
            std::cout << "instrument_roll adjusted to : " << q(3) << std::endl;
            j += 1;
            i = 0;
        }
        while (q(3) < -pi) {  
            q(3) = (pi + (q(3) + pi));
            std::cout << "instrument_roll adjusted to : " << q(3) << std::endl;
            j += 1;
            i = 1;
        }

        /*** adjust jaw_right ***/
        while (q(5) > pi) {
            q(5) = -(pi - (q(5) - pi));
            std::cout << "instrument_jaw_right adjusted to : " << q(5) << std::endl;
            j += 1;
            i = 0;
        }
        while (q(5) < -pi) {  
            q(5) = (pi + (q(5) + pi));
            std::cout << "instrument_jaw_right adjusted to : " << q(5) << std::endl;
            j += 1;
            i = 1;
        }

        /*** adjust instrument_jaw_right ***/
        if (q(5) > 0.65) {
            q(5) = 0.65;
            std::cout << "instrument_jaw_right > 0.65" << std::endl;
        }
        if (q(5) < -0.65) {
            q(5) = -0.65;
            std::cout << "instrument_jaw_right < -0.65" << std::endl;
        }
   
        control_signals.push_back(q(0)); // hand roll
        control_signals.push_back(q(1)); // hand pitch
        control_signals.push_back(q(2)); // instr slide
        control_signals.push_back(q(3)); // inst roll
        control_signals.push_back((q(4)-1.65551)); // inst pitch
        control_signals.push_back(q(5)); // jaw right

        std::cout << "control_signals[0] = " << control_signals[0] << std::endl;
        std::cout << "control_signals[1] = " << control_signals[1] << std::endl;
        std::cout << "control_signals[2] = " << control_signals[2] << std::endl;
        std::cout << "control_signals[3] = " << control_signals[3] << std::endl;
        std::cout << "control_signals[4] = " << control_signals[4] << std::endl;
        std::cout << "control_signals[5] = " << control_signals[5] << std::endl;

        /*** execute p4_hand_roll ***/
        std_msgs::Float64 u_msg_1;
        u_msg_1.data =  control_signals[0];

        /*** execute p4_hand_pith ***/
        std_msgs::Float64 u_msg_0;
        u_msg_0.data = control_signals[1];

        /*** execute p4_hand_slide ***/
        std_msgs::Float64 u_msg_2;
        u_msg_2.data = control_signals[2];

        /*** load p4_instrument_roll ***/
        std_msgs::Float64 u_msg_3;
        u_msg_3.data = control_signals[3];
        
        /*** load p4_instrument_pitch ***/
        std_msgs::Float64 u_msg_4;
        u_msg_4.data = control_signals[4];

        /*** load p4_instrument_jaw_right ***/
        std_msgs::Float64 u_msg_5;
        u_msg_5.data = control_signals[5];
  
        /*** publish setpoints ***/
        setpoints_pub_pitch.publish(u_msg_0);
        setpoints_pub_roll.publish(u_msg_1);
        setpoints_pub_slide.publish(u_msg_2);
        setpoints_pub_inst_roll.publish(u_msg_3);
        setpoints_pub_inst_pitch.publish(u_msg_4);
        setpoints_pub_inst_jaw_right.publish(u_msg_5);

    }
}
