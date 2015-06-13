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

#define pi 3.14149
#define epsilon 0.7777778
#define cx 0
#define cy 0
#define cz 0
#define rx 0.03
#define ry 0.06
#define rz 0.03
#define taux 0.110
#define tauy 0.110
#define tauz 0.110
#define kappa 0.05

int compute_fk_chain();
int write_meas_to_files_3d();

int i_ref;
int N_iter = 20;

long int iter_ref = 0;

double x_inst_slide;
double x_inst_roll;
double x_inst_pitch;
double x_jaw_right;
double x_jaw_left;
double x_hand_roll;
double x_hand_pitch;
double x1;
double x_cart_meas = 0.6;
double y_cart_meas = 0.0;
double z_cart_meas = 0.0;

std::vector<double> x_ref_vec;
std::vector<double> y_ref_vec;
std::vector<double> z_ref_vec;
std::vector<double> x3d_x_vec;
std::vector<double> x3d_y_vec;
std::vector<double> x3d_z_vec;
std::vector<double> x3d_x_ref_vec;
std::vector<double> x3d_y_ref_vec;
std::vector<double> x3d_z_ref_vec;
std::vector<double> sigma3d_vec;
std::vector<double> exe_time_vec;

Eigen::MatrixXd u_3d(3,1);
Eigen::MatrixXd utilde_3d(3,1);
Eigen::MatrixXd x(3,1);
Eigen::MatrixXd xref_3d(3,1);
Eigen::MatrixXd K(3,3);
Eigen::MatrixXd Nbar(3,3);
Eigen::MatrixXd LgB_3d(1,3);
Eigen::MatrixXd LfB_3d(1,1);
Eigen::MatrixXd sigma(1,1);
Eigen::MatrixXd k0(3,1);
Eigen::MatrixXd cbf(1,1);
Eigen::MatrixXd temp(1,1);

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
};

int safe_3d() {
    /*** static matrices ***/
    double gain = 0.02;
    K(0,0) = gain;   K(0,1) = 0.00;   K(0,2) = 0.00;
    K(1,0) = 0.00;   K(1,1) = gain;   K(1,2) = 0.00;
    K(2,0) = 0.00;   K(2,1) = 0.00;   K(2,2) = gain;
 
    Nbar(0,0) = 1 + gain;   Nbar(0,1) = 0.00;       Nbar(0,2) = 0.00;
    Nbar(1,0) = 0.00;       Nbar(1,1) = 1 + gain;   Nbar(1,2) = 0.00;
    Nbar(2,0) = 0.00;       Nbar(2,1) = 0.00;       Nbar(2,2) = 1 + gain;

    /*** initialize variable matrices ***/
    xref_3d(0,0) = 0.06;
    xref_3d(1,0) = 0.00;
    xref_3d(2,0) = 0.00;

    x(0,0) = 0.06;
    x(1,0) = 0.00;
    x(2,0) = 0.00;

    u_3d(0,0) = 0.00;
    u_3d(1,0) = 0.00;
    u_3d(2,0) = 0.00;

    utilde_3d(0,0) = 0.00;
    utilde_3d(1,0) = 0.00;
    utilde_3d(2,0) = 0.00;

    std::cout << "K = \n" << K << std::endl;
    std::cout << "Nbar = \n" << Nbar << std::endl;
    std::cout << "x = \n" << x << std::endl;
    std::cout << "xref_3d = \n" << xref_3d << std::endl;
    std::cout << "utilde_3d = \n" << utilde_3d << std::endl;
    std::cout << "u_3d = \n" << u_3d << std::endl;

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
    ros::Publisher setpoints_pub_inst_roll = node_pub.advertise<std_msgs::Float64>("inst_roll_command", 1);
    ros::Publisher setpoints_pub_inst_pitch = node_pub.advertise<std_msgs::Float64>("inst_pitch_command", 1);
    ros::Publisher setpoints_pub_inst_jaw_right = node_pub.advertise<std_msgs::Float64>("inst_jaw_right_command", 1);

    /*** read 3d references ***/
    std::vector<double> ref_vector;
    double str_ref;
    std::ifstream fin("references_3d_mod.txt"); 
    while (fin >> str_ref) 
    {  
        ref_vector.push_back(str_ref); 
    }
    fin.close(); 

    /*** read references from file ***/
    int j = 0;
    for (int i = 0; i < ref_vector.size(); ++i) {
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

    /*** give some time to digest references ***/
    int i = 1;
    int wait_time = 2;
    while(i < wait_time+1) {
        std::cout << "starting in " << (wait_time-i+1) << " seconds.." << std::endl;
        sleep(1);
        i += 1;
    }

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
                    xref_3d(0,0) = x_ref_vec.at(i_ref);
                    xref_3d(1,0) = y_ref_vec.at(i_ref);
                    xref_3d(2,0) = z_ref_vec.at(i_ref);
                    i_ref += 1;
                    iter_ref = 0;
                }
                iter_ref += 1;

                /*** update state vector ***/
                x(0,0) = x_cart_meas;                
                x(1,0) = y_cart_meas;                
                x(2,0) = z_cart_meas;                

                /*** control barrier function ***/
                cbf(0,0) = -(  pow(((x(0,0)-cx)/rx),2) + pow(((x(1,0)-cy)/ry),2) +  pow(((x(2,0)-cz)/rz),2) -  1 );
                 
                /*** linear non-safe controller ***/
                utilde_3d = Nbar*xref_3d - K*x; 

                /*** lie derivatives ***/
                LgB_3d(0,0) = (2/(pow(rx,2)*taux))*(cx-x(0,0));
                LgB_3d(0,1) = (2/(pow(ry,2)*tauy))*(cy-x(1,0));
                LgB_3d(0,2) = (2/(pow(rz,2)*tauz))*(cz-x(2,0));
                LfB_3d(0,0) =  (-2/(pow(rx,2)*taux))*(cx*x(0,0)-pow(x(0,0),2)) + (-2/(pow(ry,2)*tauy))*(cy*x(1,0)-pow(x(1,0),2)) + (-2/(pow(rz,2)*tauz))*(cz*x(2,0)-pow(x(2,0),2));

                temp = LgB_3d*LgB_3d.transpose();  
                /*** safe control law ***/
                if (LgB_3d.squaredNorm() > 0.000001) {
                    k0 = -LgB_3d.transpose()*sqrt(pow(LfB_3d(0,0),2) + pow(kappa,2)*temp(0,0) )  /(temp(0,0));
                }
                else {
                    k0(0,0) = 0;
                    k0(1,0) = 0;
                    k0(2,0) = 0;
                }          
 
                /*** calculate sigma ***/
                if ((cbf(0,0) < -epsilon) || (cbf(0,0) == -epsilon)) {
                    sigma(0,0) = 0;
                }
                else if ((cbf(0,0) > -epsilon) && (cbf(0,0) < 0)) {
                    sigma(0,0) = (-2*pow(cbf(0,0)/epsilon,3) - 3*pow(cbf(0,0)/epsilon,2) + 1);
                }
                else {
                    sigma(0,0) = 1;
                }
        
                /*** give time to start in X0 ***/
                if (iter < 60) {
                    sigma(0,0) = 0;
                }
 
                /*** control law ***/
                u_3d = sigma(0,0)*k0 + (1 - sigma(0,0))*utilde_3d;

                /*** translation is displaced ***/
                u_3d(0,0) = u_3d(0,0) + 0.482 + 0.01;
                u_3d(1,0) = u_3d(1,0);
                u_3d(2,0) = u_3d(2,0) - 0.059 + 0.01;

                double x_kdl = u_3d(0,0);  
                double y_kdl = u_3d(1,0);  
                double z_kdl = u_3d(2,0);  
              
                KDL::Vector dest_pos(x_kdl,y_kdl,z_kdl);
                KDL::Frame dest_frame(dest_pos);

                /***  Compute ***/
                int ret = iksolver.CartToJnt(q_init,dest_frame,q);
                
                for (unsigned int i = 0; i < q.rows(); ++i)
                {
                    std::cout << "Joint #" << i << ": " << q(i) << std::endl;
                }	

                std::vector<double> control_signals;

                /*** adjust instrument_roll ***/
                while (q(3) > pi) {
                    q(3) = -(pi - (q(3) - pi));
                    std::cout << "instrument_roll adjusted to : " << q(3) << std::endl;
                }
                while (q(3) < -pi) {  
                    q(3) = (pi + (q(3) + pi));
                    std::cout << "instrument_roll adjusted to : " << q(3) << std::endl;
                }

                /*** adjust instrument_jaw_right ***/
                while (q(5) > pi) {
                    q(5) = -(pi - (q(5) - pi));
                    std::cout << "instrument_jaw_right adjusted to : " << q(5) << std::endl;
                }
                while (q(5) < -pi) {  
                    q(5) = (pi + (q(5) + pi));
                    std::cout << "instrument_jaw_right adjusted to : " << q(5) << std::endl;
                }

                /*** adjust instrument_jaw_right ***/
                if (q(5) > 0.1) {
                    q(5) = 0.1;
                    std::cout << "instrument_jaw_right > 0.1" << std::endl;
                }
                if (q(5) < -0.1) {
                    q(5) = -0.1;
                    std::cout << "instrument_jaw_right < -0.1" << std::endl;
                }
   
                /*** physical limits for hand_roll ***/
                if (q(0) > 1.5) {
                    q(0) = 1.5;
                }
                else if (q(0) < -1.5) {
                    q(0) = -1.5;
                } 
                /*** physical limits for hand_pitch ***/
                if (q(1) > 0.7) {
                    q(1) = 0.7;
                }
                else if (q(1) < -0.7) {
                    q(1) = -0.7;
                } 
                /*** physical limits for instrument_slide ***/
                if (q(2) > 0.097) {
                    q(2) = 0.097;
                }
                else if (q(2) < -0.097) {
                    q(2) = -0.097;
                }
                /*** physical limits for instrument_pitch ***/
                if (q(4) > 0.8+1.65551) {
                    q(4) = 0.8+1.65551;
                }
                else if (q(4) < -0.8+1.65551) {
                    q(4) = -0.8+1.65551;
                }
            
                /*** collect all control signals in one vector ***/
                control_signals.push_back(q(0)); // hand roll
                control_signals.push_back(q(1)); // hand pitch
                control_signals.push_back(q(2)); // instr slide
                control_signals.push_back(q(3)); // inst roll
                control_signals.push_back((q(4)-1.65551)); // inst pitch
                control_signals.push_back(q(5)); // jaw right

                /*** execute p4_hand_pith ***/
                std_msgs::Float64 u_msg_0;
                u_msg_0.data = control_signals[1];

                /*** execute p4_hand_roll ***/
                std_msgs::Float64 u_msg_1;
                u_msg_1.data =  control_signals[0];

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

                /*** publish joint angles ***/
                setpoints_pub_pitch.publish(u_msg_0);
                setpoints_pub_roll.publish(u_msg_1);
                setpoints_pub_slide.publish(u_msg_2);
                setpoints_pub_inst_roll.publish(u_msg_3);
                setpoints_pub_inst_pitch.publish(u_msg_4);
                setpoints_pub_inst_jaw_right.publish(u_msg_5);

                /*** check execution time ***/
                dur = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
                std::cout << "execution time = " << dur << std::endl; 
                std::cout  << "sigma = " << sigma(0,0) << std::endl;
                std::cout  << "u = " << u_3d << std::endl;

                /*** provide some user information ***/
                iter += 1;
                std::cout << iter <<  " iterations" << std::endl;
                std::cout << "\n";

                /*** write slide measurements to file after inital transients ***/
                if (iter > 20) {
                    /*** export trajectory ***/
                    x3d_x_vec.push_back(x(0,0));
                    x3d_y_vec.push_back(x(1,0));
                    x3d_z_vec.push_back(x(2,0));
                    /*** export references ***/
                    x3d_x_ref_vec.push_back(xref_3d(0,0));
                    x3d_y_ref_vec.push_back(xref_3d(1,0));
                    x3d_z_ref_vec.push_back(xref_3d(2,0));
                    /*** export sigma ***/
                    sigma3d_vec.push_back(sigma(0,0));
                    /*** export execution time ***/
                    exe_time_vec.push_back(dur);
                }
            
                if (iter > N_iter*x_ref_vec.size() + N_iter) {
                    std::cout << "writing meas to file..." << std::endl;
                    write_meas_to_files_3d();
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
        }
   
        /*** get chain from reduced robot ***/ 
        KDL::Chain my_chain;
        std::string root_link("p4_rcm_base");
        std::string tip_link("needle_driver_jawbone_right");
        if (!(my_tree.getChain(root_link, tip_link, my_chain)))
        {
            ROS_ERROR("Failed to get chain");
        }
    
        /*** Create solver based on kinematic chain ***/
        KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(my_chain);
    
        /*** Create joint array ***/
        unsigned int nj = my_chain.getNrOfJoints();
        KDL::JntArray jointpositions = KDL::JntArray(nj);    
        jointpositions(0) = x_hand_roll;
        jointpositions(1) = x_hand_pitch;
        jointpositions(2) = x_inst_slide;
        jointpositions(3) = x_inst_roll;
        jointpositions(4) = x_inst_pitch;
        jointpositions(5) = x_jaw_right;

        /*** Create the frame that will contain the results ***/
        KDL::Frame cartpos;
    
        /*** Calculate forward position kinematics ***/
        bool kinematics_status;
        kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
        if (kinematics_status >= 0) {
            printf("%s \n","FK calculated succesfully");
        } 
        else {
            printf("%s \n","Error: could not calculate FK");
        }

        /*** subtract translation ***/
        x_cart_meas = cartpos(0,3) - 0.482; 
        y_cart_meas = cartpos(1,3) + 0.00; 
        z_cart_meas = cartpos(2,3) + 0.059; 
        
    return 0;
}

int write_meas_to_files_3d() {
    std::cout << "writing measurements to files.." << std::endl << std::endl;
    /*** print x position trajectory to file ***/
    std::ofstream f1;
    f1.open("x.txt", std::ios::out | std::ios::app | std::ios::binary);
    for (int i = 0; i < x3d_x_vec.size(); i += 1) {
        f1 << x3d_x_vec[i] << std::endl;
    }
    f1.close();
    /*** print y position trajectory to file ***/
    std::ofstream f2;
    f2.open("y.txt", std::ios::out | std::ios::app | std::ios::binary);
    for (int i = 0; i < x3d_y_vec.size(); i += 1) {
        f2 << x3d_y_vec[i] << std::endl;
    }
    f2.close();
    /*** print z position trajectory to file ***/
    std::ofstream f3;
    f3.open("z.txt", std::ios::out | std::ios::app | std::ios::binary);
    for (int i = 0; i < x3d_z_vec.size(); i += 1) {
        f3 << x3d_z_vec[i] << std::endl;
    }
    f3.close();
    /*** print x position reference to file ***/
    std::ofstream f4;
    f4.open("x_ref.txt", std::ios::out | std::ios::app | std::ios::binary);
    for (int i = 0; i < x3d_x_ref_vec.size(); i += 1) {
        f4 << x3d_x_ref_vec[i] << std::endl;
    }
    f4.close();
    /*** print y position reference to file ***/
    std::ofstream f5;
    f5.open("y_ref.txt", std::ios::out | std::ios::app | std::ios::binary);
    for (int i = 0; i < x3d_y_ref_vec.size(); i += 1) {
        f5 << x3d_y_ref_vec[i] << std::endl;
    }
    f5.close();
    /*** print z position reference to file ***/
    std::ofstream f6;
    f6.open("z_ref.txt", std::ios::out | std::ios::app | std::ios::binary);
    for (int i = 0; i < x3d_z_ref_vec.size(); i += 1) {
        f6 << x3d_z_ref_vec[i] << std::endl;
    }
    f6.close();
    /*** print sigma to file ***/
    std::ofstream f7;
    f7.open("sigma_3d.txt", std::ios::out | std::ios::app | std::ios::binary);
    for (int i = 0; i < sigma3d_vec.size(); i += 1) {
        f7 << sigma3d_vec[i] << std::endl;
    }
    f7.close();
    /*** print execution time to file ***/
    std::ofstream f8;
    f8.open("exe_3d.txt", std::ios::out | std::ios::app | std::ios::binary);
    for (int i = 0; i < exe_time_vec.size(); i += 1) {
        f8 << exe_time_vec[i] << std::endl;
    }
    f8.close();

return 0;
}
