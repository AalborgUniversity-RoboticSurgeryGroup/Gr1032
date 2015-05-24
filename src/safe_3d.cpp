#include "safe_3d.h"

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
#include <math.h>
#include <fstream>
#include <Eigen/Dense>

int compute_fk_chain();
int i_ref;
int N_iter = 70;

long int iter_ref = 0;

double x_inst_slide;
double x_inst_roll;
double x_inst_pitch;
double x_jaw_right;
double x_jaw_left;
double x_hand_roll;
double x_hand_pitch;
double x1;
double x_cart_meas = 0.0;
double y_cart_meas = 0.0;
double z_cart_meas = 0.0;

std::vector<double> x_ref_vec;
std::vector<double> y_ref_vec;
std::vector<double> z_ref_vec;

Eigen::MatrixXd u(3,1);
Eigen::MatrixXd x(3,1);
Eigen::MatrixXd xref(3,1);
Eigen::MatrixXd K(3,3);
Eigen::MatrixXd Nbar(3,3);

class timer {
	private:
		long double begTime;
	public:
		void start() {
			begTime = clock();
		}

		long double elapsedTime() {
			return ((long double) clock() - begTime) / CLOCKS_PER_SEC;
		}

		bool isTimeout(unsigned long Ts) {
			return Ts >= elapsedTime();
        		}
};

int safe_3d() {
    K(0,0) = 0.10;   K(0,1) = 0.00;   K(0,2) = 0.00;
    K(1,0) = 0.00;   K(1,1) = 0.10;   K(1,2) = 0.00;
    K(2,0) = 0.00;   K(2,1) = 0.00;   K(2,2) = 0.10;
 
    Nbar(0,0) = 1.10;   Nbar(0,1) = 0.00;   Nbar(0,2) = 0.00;
    Nbar(1,0) = 0.00;   Nbar(1,1) = 1.10;   Nbar(1,2) = 0.00;
    Nbar(2,0) = 0.00;   Nbar(2,1) = 0.00;   Nbar(2,2) = 1.10;

    xref(0,0) = 0.00;
    xref(1,0) = 0.00;
    xref(2,0) = 0.00;

    x(0,0) = 0.00;
    x(1,0) = 0.00;
    x(2,0) = 0.00;

    std::cout << "K = \n" << K << std::endl;
    std::cout << "Nbar = \n" << Nbar << std::endl;
    std::cout << "xref = \n" << xref << std::endl;
    std::cout << "x = \n" << x << std::endl;

   /*** prepare real time processing ***/ 
   timer t;
   t.start();

    std::cout << "Inverse Kinematic Mode!" << std::endl;

    ros::NodeHandle node;
    KDL::Tree my_tree;
    std::string robot_desc_string;
    node.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, my_tree))
    {
       ROS_ERROR("Failed to construct kdl tree");
    }

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

    //Create solver based on kinematic chain
    KDL::ChainFkSolverPos_recursive fksolver(my_chain);
    KDL::ChainIkSolverVel_pinv iksolverv(my_chain);
    KDL::ChainIkSolverPos_NR iksolver = KDL::ChainIkSolverPos_NR(my_chain,fksolver,iksolverv,100,1e-6);

    KDL::JntArray q(my_chain.getNrOfJoints());
    KDL::JntArray q_init(my_chain.getNrOfJoints());

    /*** prepare to publish control signal ***/ 
    ros::NodeHandle node_pub;
    ros::Publisher setpoints_pub_pitch = node_pub.advertise<std_msgs::Float64>("pitch_command", 1);
    ros::Publisher setpoints_pub_slide = node_pub.advertise<std_msgs::Float64>("slide_command", 1);
    ros::Publisher setpoints_pub_roll  = node_pub.advertise<std_msgs::Float64>("roll_command", 1);

    /*** read 3d references ***/
    std::vector<double> ref_vector;
    double str_ref;
    std::ifstream fin("references_3d.txt"); 
    while (fin >> str_ref) 
    {  
        ref_vector.push_back(str_ref); 
    }
    fin.close(); 

    /*** fetch references ***/
    int j = 0;
    for (int i = 0; i < ref_vector.size(); ++i) {
        //std::cout << ref_vector.at(i) << std::endl;
        if (j == 0) {
            x_ref_vec.push_back(ref_vector.at(i));
            j += 1;
        }
        else if (j == 1) {
            y_ref_vec.push_back(ref_vector.at(i));
            j += 1;
        }
        else if (j == 2) {
            z_ref_vec.push_back(ref_vector.at(i));
            j = 0;
        }
    }

    /*** print references ***/
    std::cout << "\n";
    std::cout << "x references: " << std::endl;
    for (int i = 0; i < x_ref_vec.size(); ++i) 
        std::cout << x_ref_vec.at(i) << std::endl;
    std::cout << "\n";
    std::cout << "y references: " << std::endl;
    for (int i = 0; i < y_ref_vec.size(); ++i) 
        std::cout << y_ref_vec.at(i) << std::endl;
    std::cout << "\n";
    std::cout << "z references: " << std::endl;
    for (int i = 0; i < z_ref_vec.size(); ++i) 
        std::cout << z_ref_vec.at(i) << std::endl;
    std::cout << "\n";

    std::cout << "starting in 1 second.." << std::endl;
    sleep(1);

    /*** start controller ***/
    int iter = 0;
    while(true) {

        /*** subscribe to topics with 100 Hz ***/
        ros::Rate r(100);
         
        /*** prepare real time processing ***/
        timer t;
        t.start();
        double long Ts = 0.01;
        while(true) {
            if (t.elapsedTime() >= Ts) {
              
                /*** read sensor ***/ 
                ros::spinOnce();

                /*** convert to 3D ***/ 
                compute_fk_chain();

                /*** start timer to meassure execution time ***/
                std::clock_t start;
                double dur;
                start = std::clock();               
              
                /*** update trajectory ***/ 
                if (iter_ref > N_iter) {
                    xref(0,0) = x_ref_vec.at(i_ref);
                    xref(1,0) = y_ref_vec.at(i_ref);
                    xref(2,0) = z_ref_vec.at(i_ref);
                    i_ref += 1;
                    iter_ref = 0;
                }
                iter_ref += 1;

                x(0,0) = x_cart_meas;                
                x(1,0) = y_cart_meas;                
                x(2,0) = z_cart_meas;                

                /*** linear controller ***/
                u = Nbar*xref - K*x; 
                std::cout << "u = " << u << std::endl;

                /*** translation is off ***/
                u(0,0) = u(0,0) + 0.490;
                u(1,0) = u(1,0);
                u(2,0) = u(2,0) - 0.059;

                double x_kdl = u(0,0);  
                double y_kdl = u(1,0);  
                double z_kdl = u(2,0);  
              
                KDL::Vector dest_pos(x_kdl,y_kdl,z_kdl);
                KDL::Frame dest_frame(dest_pos);

                /***  Compute ***/
                int ret = iksolver.CartToJnt(q_init,dest_frame,q);
 
                /*
                for (unsigned int i = 0; i < q.rows(); ++i)
                {
                    std::cout << "Joint #" << i << ": " << q(i) << std::endl;
                }
		*/

                std::vector<double> control_signals;

                control_signals.push_back(q(0));
                control_signals.push_back(q(1));
                control_signals.push_back(q(2));

                //std::cout << "control_signals[0] = " << control_signals[0] << std::endl;
                //std::cout << "control_signals[1] = " << control_signals[1] << std::endl;
                //std::cout << "control_signals[2] = " << control_signals[2] << std::endl;

                /*** execute p4_hand_pith ***/
                std_msgs::Float64 u_msg_0;
                u_msg_0.data = control_signals[1];

                /*** execute p4_hand_roll ***/
                std_msgs::Float64 u_msg_1;
                u_msg_1.data =  control_signals[0];

                /*** execute p4_hand_slide ***/
                std_msgs::Float64 u_msg_2;
                u_msg_2.data = control_signals[2];

                setpoints_pub_pitch.publish(u_msg_0);
                setpoints_pub_roll.publish(u_msg_1);
                setpoints_pub_slide.publish(u_msg_2);


                 /*** check execution time ***/
                 dur = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
                 std::cout << "execution time = " << dur << std::endl; 
                 /*** provide some user information ***/
                 iter += 1;
                 std::cout << iter <<  " iterations" << std::endl;
                 std::cout << "\n";

                 if (iter > N_iter*x_ref_vec.size() + N_iter) {
                     ros::shutdown();
                     return 0;
                 }

                 break;
           }
         else {
             /*** do some other stuff if necessary***/
         }
    }
}

}


int compute_fk_chain() {
        KDL::Tree my_tree;
        ros::NodeHandle node_meas;
        std::string robot_desc_string;
        node_meas.param("robot_description", robot_desc_string, std::string());
        if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
           ROS_ERROR("Failed to construct kdl tree");
           return -1;
        }
   
        /*
        // get names: 
        for (KDL::SegmentMap::const_iterator it = my_tree.getSegments().begin(); it != my_tree.getSegments().end(); ++it)
        {
            std::cout << it->second.segment.getName() <<std::endl;
        }
        */
   
        /*** get chain from reduced robot ***/ 
        KDL::Chain my_chain;
        std::string root_link("p4_rcm_base");
        std::string tip_link("needle_driver_jawbone_right");
        if (!(my_tree.getChain(root_link, tip_link, my_chain)))
        {
            ROS_ERROR("Failed to get chain");
            return -1;
        }
    
        // Create solver based on kinematic chain
        KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(my_chain);
    
        // Create joint array
        unsigned int nj = my_chain.getNrOfJoints();
        KDL::JntArray jointpositions = KDL::JntArray(nj);
    
        jointpositions(0) = x_hand_roll;
        jointpositions(1) = x_hand_pitch;
        jointpositions(2) = x_inst_slide;
        jointpositions(3) = 0;
        jointpositions(4) = 0;
        jointpositions(5) = 0;

        // Create the frame that will contain the results
        KDL::Frame cartpos;
    
        // Calculate forward position kinematics
        bool kinematics_status;
        kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
        if (kinematics_status >= 0) {
            //std::cout << cartpos <<std::endl;
            printf("%s \n","FK calculated succesfully");
        } 
        else {
            printf("%s \n","Error: could not calculate FK");
        }

        /*** subtract virtual lenghts ***/
        x_cart_meas = cartpos(0,3) - 0.481998; 
        y_cart_meas = cartpos(1,3) + 0.0010839; 
        z_cart_meas = cartpos(2,3) + 0.0679988; 
        
        /* 
        std::cout << "x_cart_meas = " << x_cart_meas << std::endl;  
        std::cout << "y_cart_meas = " << y_cart_meas << std::endl;  
        std::cout << "z_cart_meas = " << z_cart_meas << std::endl;  
        */

    return 0;
}
