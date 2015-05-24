#define FAST2
#include "ik_gr1032.h"
#include "demo_gr1032.h"
#include "safe_3d.h"

/*** include libraries ***/
#include <string>
#include <vector>
#include <iostream> 
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
//#include <trajectory_msgs/JointTrajectory.h>
//#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <time.h> 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <fstream>
#include <Eigen/Dense>

/*** define macros ***/
#define K 0.1
#define Nbar 1.1
#define kappa 1
#define a 1.7778
#define b 0.0889
#define c -0.0089
#define epsilon 0.002488888888889
#define tau 0.1
#define N_samples 100
#define a2 0.07500
#define b2 -4
#define zeta 0.55
#define x10 0.0250
#define wn 17
#define c1 1
#define c2 -1

/*** synopsises ***/
int slide_safety_controller(int model);
int write_meas_to_files();
int slide_angles();
int demo();
int beating_heart();
int write_sine_data_to_file();
int ik_angles();


/*** global doubles ***/
double x1;
double x1_hat;
double x2_hat;
double xref = 0.00;
double LgB;
double LfB;
double k0;
double utilde;
double u = 0;
double sigma = 0;
double cbf;
double Ac1 = -1/tau;
double Gamma1 =  1.095169439874664;
double Bc1 = 1/tau;
double Phi1 = 0.095169439874664;
double N;
double P;
double err;
double x_inst_slide;
double x_inst_roll;
double x_inst_pitch;
double x_jaw_right;
double x_jaw_left;
double x_hand_roll;
double x_hand_pitch;

/*** global vectors ***/
std::vector<double> x1_vec;
std::vector<double> xref_vec;
std::vector<double> sigma_vec;
std::vector<double> u_vec;
std::vector<double> LgB_vec;
std::vector<double> LfB_vec;
std::vector<double> err_vec;
std::vector<double> x1_hat_vec;
std::vector<double> x2_hat_vec;
std::vector<double> dur_vec;
std::vector<double> x1_beat_vec;
std::vector<double> dref_vec;

/*** global matrices ***/
Eigen::MatrixXd Gamma(2,2);
Eigen::MatrixXd Phi(2,1);
Eigen::MatrixXd C(1,2);
Eigen::MatrixXd Kd(1,2);
Eigen::MatrixXd Ld(2,1);
Eigen::MatrixXd x_hat(2,1);
Eigen::MatrixXd x1_eigen(1,1);
Eigen::MatrixXd eigen_temp(1,1);
Eigen::MatrixXd M(2,1);

/*** beating heart matrices ***/
Eigen::MatrixXd A_beat(4,4);
Eigen::MatrixXd B_beat(4,1);
Eigen::MatrixXd K_beat(1,4);
Eigen::MatrixXd x_beat(4,1);

/*** global integers ***/
int ref_counter = 0;

/*** callback function to read position sensor ***/
void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
    { 
        //ROS_INFO("slide postion: %f", msg->position[6]);
        //ROS_INFO("name: %s", msg->name[6].c_str());
        x1 = msg->position[6];
        x_inst_slide = x1;
        x_inst_roll = msg->position[5];
        x_inst_pitch = msg->position[4];
        x_jaw_right = msg->position[3];
        x_jaw_left = msg->position[2];
        x_hand_roll = msg->position[1];
        x_hand_pitch = msg->position[0];
    }

/*** timer class ***/
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "run_controllers", ros::init_options::AnonymousName);
    //std::cout << "static observer matrices:\n" << std::endl;


#ifdef SLOW
    /*** define Gamma for slide safety controller ***/
    Gamma(0,0) = 0.9493;
    Gamma(1,0) = -4.7300;
    Gamma(0,1) = 0.0164;
    Gamma(1,1) = 0.6432;
    /*** define Phi for slide safety controller ***/
    Phi(0,0) = 0.0507;
    Phi(1,0) = 4.73;
    /*** define the output matrix C for slide safety controller ***/
    C(0,0) = 1;
    C(0,1) = 0;
    /*** define the feedback Kd for slide safety controller ***/
    Kd(0,0) = 0.3457;
    Kd(0,1) = -0.0233;
    /*** define the observer gain Ld for slide safety controller ***/
    Ld(0,0) = -1.3925;
    Ld(1,0) = -13.2982;
    x1_eigen(0,0) = 0;
    /*** define gains for slide safety controller ***/
    M(0,0) = 1.8790;
    M(1,0) = 67.4261;
    N = 3.1505;
    P = 0.26500;
#endif
#ifdef FAST
    /*** define Gamma for slide safety controller ***/
    Gamma(0,0) = 0.9864;    Gamma(0,1) = 0.0091;
    Gamma(1,0) = -2.6232 ; Gamma(1,1) = 0.8167;
    /*** define Phi for slide safety controller ***/
    Phi(0,0) = 0.0136;      
    Phi(1,0) = 2.6232;
    /*** define the output matrix C for slide safety controller ***/
    C(0,0) = 1;         C(0,1) = 0;
    /*** define the feedback Kd for slide safety controller ***/
    Kd(0,0) = 0.2546;   Kd(0,1) = -0.033;
    /*** define the observer gain Ld for slide safety controller ***/
    Ld(0,0) = -0.6031;
    Ld(1,0) = -2.5506;
    x1_eigen(0,0) = 0;
    /*** define gains for slide safety controller ***/
    N = 58.06;
    M(0,0) = 0.787;
    M(1,0) = 152.31;
    P = 0.013;
#endif
#ifdef FAST2
    /*** define Gamma for slide safety controller ***/
    Gamma(0,0) = 0.9864;    Gamma(0,1) = 0.0091;
    Gamma(1,0) = -2.6232 ; Gamma(1,1) = 0.8167;
    /*** define Phi for slide safety controller ***/
    Phi(0,0) = 0.0136;      
    Phi(1,0) = 2.6232;
    /*** define the output matrix C for slide safety controller ***/
    C(0,0) = 1;         C(0,1) = 0;
    /*** define the feedback Kd for slide safety controller ***/
    Kd(0,0) = 0.25;   Kd(0,1) = -0.03;
    /*** define the observer gain Ld for slide safety controller ***/
    Ld(0,0) = -0.25;
    Ld(1,0) = -0.02;
    x1_eigen(0,0) = 0;
    /*** define gains for slide safety controller ***/
    N = 58;
    M(0,0) = 0.78;
    M(1,0) = 152.31;
    P = 0.0129;
#endif

    /*** initialize x_hat ***/
    x_hat(0,0) = x1;
    x_hat(1,0) = 0;

    //std::cout << "Gamma = \n" << Gamma << std::endl;
    //std::cout << "Phi = \n" << Phi << std::endl;
    //std::cout << "C = \n" << C << std::endl;
    //std::cout << "Kd = \n" << Kd << std::endl;
    //std::cout << "Ld = \n" << Ld << std::endl;
    //std::cout << "x_hat = \n" << x_hat << std::endl;
    //std::cout << "M = \n" << M << std::endl;
    //std::cout << "\n" << std::endl;

    /*** welcome screen ***/
    std::cout << "Starting program..." << std::endl;
    std::cout << "\n******************************************************" << std::endl;
    std::cout << "The following options are avaiable:" << std::endl;
    std::cout << "------------------------------------------------------" << std::endl;
    std::cout << "press 'a' to run slide safety controller" << std::endl;
    std::cout << "press 'b' to specify custom joint angles (FK mode)" << std::endl;
    std::cout << "press 'c' to run demo" << std::endl;
    std::cout << "press 'd' to run beating heart controller" << std::endl;
    std::cout << "press 'e' to specify custom 3D angles (IK mode)" << std::endl;
    std::cout << "press 'f' to run 3D safety controller" << std::endl;
    std::cout << "******************************************************" << std::endl;

    char choice;
    std::cin >> choice;

    /*** initialize callback function for sensor measurements ***/
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("joint_states", 1000, joint_states_callback);

    while (choice != 'q') {
        if (choice ==  'a'){
            /*** remove old files ***/
            remove("slide_data.txt"); 
            remove("slide_ref.txt");
            remove("control_signal.txt");
            remove("sigma.txt");
            remove("LgB.txt");
            remove("LfB.txt");
            remove("err.txt");
            remove("x1_hat.txt");
            remove("x2_hat.txt");
            remove("dur.txt");

            /*** ask for underlying approximation ***/ 
            int model;
            std::cout << "model order approximation? (1 or 2)" << std::endl;
            std::cin >> model;

            /*** run controller ***/
            slide_safety_controller(model);

            return 0;
        }
        else if (choice == 'b') {
            slide_angles();
            return 0;
        }
        else if (choice == 'c') {
            demo();
            return 0;
        }
        else if (choice == 'd') {
            remove("x1_beat.txt"); 
            remove("sigma.txt"); 
            beating_heart();
            return 0;
        }
        else if (choice == 'e') {
            ik_pos();
            return 0;
        }
        else if (choice == 'f') {
            safe_3d();
            return 0;
        }
        else {
            std::cout << "please press one of the given letters or press 'q' to quit" << std::endl;
            std::cin >> choice;
        }
    }

    return 0;
}

/*** slide safety controller ***/
int slide_safety_controller(int model) {
    std::cout << "Entered safety controller for slide position!" << std::endl;
    std::cout << "running safety controller" << std::endl;
   
    /*** prepare to publish control signal ***/ 
    ros::NodeHandle node;
    ros::Publisher setpoints_pub_pitch = node.advertise<std_msgs::Float64>("pitch_command", 1);
    ros::Publisher setpoints_pub_slide = node.advertise<std_msgs::Float64>("slide_command", 1);

    /*** read setpoints from file  ***/
    std::vector<double> xrefs (200);
    int i = 0;
    int iter = 0;
    std::fstream myfile("references.txt", std::ios_base::in);
    while (myfile >> xrefs[i])
    {
        printf("%f ", xrefs[i]);
        i += 1; 
    }
    int N_xrefs = i;
    std::cout << N_xrefs << " reference points provided" << std::endl;

#ifdef SLOW
    long double Ts = 0.02; 
#endif
#ifdef FAST
    long double Ts = 0.01;
#endif
#ifdef FAST2
    long double Ts = 0.01;
#endif

    /*** run slide controller ***/
    while(true) {

        /*** subscribe to topics with 100 Hz ***/
        ros::Rate r(100);

        /*** prepare real time processing ***/
        timer t;
        t.start();

        while(true) {
            if (t.elapsedTime() >= Ts) {
              
                /*** read sensor ***/ 
                ros::spinOnce();
 
                 /*** determine setpoint ***/
                 if (iter % N_samples == 0) {
                     xref = xrefs[ref_counter];
                     ref_counter += 1; 
                 }
          
                std::cout << "x1 = " << x1 << std::endl;
                std::cout << "xref = " << xref << std::endl;
 
                /*** start timer to meassure execution time ***/
                std::clock_t start;
                double dur;
                start = std::clock();               
               

                /*** barrier function based on first order approximation ***/
                cbf = a*x1*x1 + b*x1 + c;

                if (model == 1) {
                    /*** calculate linear control output ***/
                    utilde = Nbar*xref - K*x1;

                    /*** lie derivatives based on first order approximation ***/
                    LgB = 2*a*x1*(1/tau) + b*(1/tau);
                    LfB = -2*(1/tau)*a*pow(x1,2) - (1/tau)*b*x1;
                }
                else if (model == 2) {
                    /*** calculate linear control output ***/
                    eigen_temp = Kd*x_hat;
                    double temp_ = (double) eigen_temp(0,0);
                    utilde = temp_ + N*xref*P;
                    x1_eigen(0,0) = x1;

                    /*** calculate error ***/
                    double err = (double) x1_eigen(0,0) - x_hat(0,0);
                    std::cout << "error = " << err << std::endl;

                    /*** calculate x_hat(k+1) ***/
                    x_hat = Gamma*x_hat + Phi*Kd*x_hat + Ld*( C*x_hat - x1_eigen ) + M*xref*P;

                    /*** calculate lie derivatives ***/
                    x1_hat = x_hat(0,0);
                    x2_hat = x_hat(1,0);
                    LgB = x2_hat * (2*c1*pow(wn,2)  / pow(b2,2) );
                    LfB = x2_hat * ( (c1*(2*x1_hat+2*x10)) / (pow(a2,2))  -  (2*c1*(2*x1_hat*pow(wn,2) + 2*x2_hat*zeta*wn)) / (pow(b2,2)));          
                    std::cout << "x_hat = \n" << x_hat << std::endl;
                }

                /*** safe control law ***/
                int delta = 0.00001;
                if (abs(LgB) > delta) {
                    k0 = -LgB*( LfB + sqrt( LfB*LfB + kappa*kappa + LgB*LgB ) )/( LgB*LgB );
                }
                else {
                    k0 = 0;
                    std::cout << "k0 = 0!!!" << std::endl;
                }          
 
                /*** calculate sigma ***/
                if ((cbf < -epsilon) || (cbf == -epsilon)) {
                    sigma = 0;
                }
                else if ((cbf > -epsilon) && (cbf < 0)) {
                    sigma = 3*(-2*pow(cbf/epsilon,3) - 3*pow(cbf/epsilon,2	) + 1);
                }
                else {
                    sigma = 1;
                }
                if (sigma > 1) {
                    sigma = 1;
                }
               
                std::cout << "sigma = " << sigma << std::endl;

                /*** control law ***/
                u = sigma*k0 + (1 - sigma)*utilde;

                /*** slide physical limits ***/
                if (u > 0.099)
                     u = 0.099;
                else if (u < -0.099)
                     u = -0.099;
                
                /*** send control signals to robot ***/
		std_msgs::Float64 zero_msg;
		zero_msg.data = 0.7;
                setpoints_pub_pitch.publish(zero_msg);
		std_msgs::Float64 u_msg;
		u_msg.data = u;
                setpoints_pub_slide.publish(u_msg);

                /*** provide some user information ***/
                iter += 1;
                std::cout << iter <<  " iterations" << std::endl;
                std::cout << "model order = " << model << std::endl;

                /*** write slide measurements to file after inital transients ***/
                if (iter > 0) {
                    x1_vec.push_back(x1);
                    xref_vec.push_back(xref);
                    sigma_vec.push_back(sigma);
                    u_vec.push_back(u);
                    LgB_vec.push_back(LgB);
                    LfB_vec.push_back(LfB);
                    if (model == 2) {
                        err_vec.push_back(err);
                        x1_hat_vec.push_back(x1_hat);
                        x2_hat_vec.push_back(x2_hat);
                    }
                    dur_vec.push_back(dur);
                 }

                 /*** check execution time ***/
                 dur = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
                 std::cout << "execution time = " << dur << std::endl; 
                 std::cout << "\n";

                 if (iter > N_xrefs*N_samples) {
                     write_meas_to_files();
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
    return 0;
}
                    
int beating_heart() {
    std::cout << "beating heart controller is entered" << std::endl;

    /*** define local variables ***/
    double wh = 5.7119866;
    double Kbar_beat = 0.1;
    double Nbar_beat = 1.1;

    /*** define augmented feedback vector ***/
    K_beat(0,0) = -Kbar_beat; K_beat(0,1) = Nbar_beat;  K_beat(0,2) = 0;   K_beat(0,3) = Nbar_beat;

    /*** prepare to publish control signal ***/ 
    ros::NodeHandle node;
    ros::Publisher setpoints_pub_pitch = node.advertise<std_msgs::Float64>("pitch_command", 1);
    ros::Publisher setpoints_pub_slide = node.advertise<std_msgs::Float64>("slide_command", 1);

    /*** subscribe to topics with 100 Hz ***/
    ros::Rate r(100);

   /*** initial conditions ***/
    x_beat(0,0) = 0.05;    
    x_beat(1,0) = 0.01;   
    x_beat(2,0) = 0;   
    x_beat(3,0) = 0.03;

    int iter = 0;
    double cbf_beat;
    double epsilon_beat = 0.015;
    long double Ts = 0.01;
    while(true) {
        /*** prepare real time processing ***/
        timer t;
        t.start();
        while(true) {
            if (t.elapsedTime() >= Ts) {
                /*** read sensor ***/ 
                ros::spinOnce();

                /*** update state vector ***/
                x_beat(0,0) = x1; 
                x_beat(1,0) = 0.01*cos(wh*Ts*iter);
                x_beat(2,0) = 0.01*sin(wh*Ts*iter);
               
                if ((iter > 500) && (iter < 700)) {
                    /*** give unsafe distance ***/
                    x_beat(3,0) = -0.02;
                }
                else if (iter > 1000) {
                    x_beat(3,0) = 0.025;
                }
                else {
                    /*** give safe distance ***/
                    x_beat(3,0) = 0.04;
                }
                
                /*** control barrier function ***/
                cbf_beat = x_beat(1,0) - x_beat(0,0);

                /*** calculate linear control output ***/
                eigen_temp = K_beat*x_beat;
                utilde = (double) eigen_temp(0,0);
            
                /*** lie derivatives ***/
                LgB = -1/tau;
                LfB = wh*x_beat(2,0) + x_beat(0,0)/tau;

                /*** safe control law ***/
                int delta = 0.00001;
                if (abs(LgB) > delta) {
                    k0 = -LgB*(LfB + sqrt(LfB*LfB + kappa*kappa + LgB*LgB))/(LgB*LgB);
                }
                else {
                    k0 = 0;
                    std::cout << "k0 = 0" << std::endl;
                }          
 
                /*** calculate sigma ***/
                if ((cbf_beat < -epsilon_beat) || (cbf_beat == -epsilon_beat)) {
                    sigma = 0;
                }
                else if ((cbf_beat > -epsilon_beat) && (cbf_beat < 0)) {
                    sigma = (-2*pow(cbf_beat/epsilon_beat,3) - 3*pow(cbf_beat/epsilon_beat,2) + 1);
                }
                else {
                    sigma = 1;
                }
               
                /*** control law ***/
                u = sigma*k0 + (1 - sigma)*utilde;

                /*** slide physical limits ***/
                if (u > 0.099)
                     u = 0.099;
                else if (u < -0.099)
                     u = -0.099;

               /*** make sure to start robot in a safe area ***/
                if (iter < 50 ) {
                    u = 0.08;
                }
 
                /*** send control signals to robot ***/
		std_msgs::Float64 zero_msg;
		zero_msg.data = 0.0;
                setpoints_pub_pitch.publish(zero_msg);
		std_msgs::Float64 u_msg;
		u_msg.data = u;
                setpoints_pub_slide.publish(u_msg);

                /*** give some user information ***/
                std::cout << "xh1 = " << x_beat(1,0) << std::endl;
                std::cout << "x1 = " << x1 << std::endl;
                std::cout << "sigma = " << sigma << std::endl;
                std::cout << "u = " << u << std::endl;
                std::cout << "iter = " << iter << std::endl;               
                iter += 1;

                /*** save measurements ***/
                if (iter > 0) {
                    x1_beat_vec.push_back(x1);
                    sigma_vec.push_back(sigma);
                    dref_vec.push_back(x_beat(3,0));
                }

                /*** write measurements to file and end section ***/
                if (iter == 1400) {
                    write_sine_data_to_file();
                    return 0;
                }
                break;
            }
            else {
                /*** do some other stuff if necessary***/
            }
        }
    }
    return 0;
}

int write_sine_data_to_file() {
    /*** print position trajectory to file ***/
    std::ofstream f1;
    f1.open("x1_beat.txt", std::ios::out | std::ios::app | std::ios::binary);
    for (int i = 0; i < x1_beat_vec.size(); i += 1) {
        f1 << x1_beat_vec[i] << std::endl;
    }
    f1.close();

    /*** print reference distance to file ***/
    std::ofstream f2;
    f2.open("dref.txt", std::ios::out | std::ios::app | std::ios::binary);
    for (int i = 0; i < dref_vec.size(); i += 1) {
        f2 << dref_vec[i] << std::endl;
    }
    f2.close();

    /*** print sigma values to file ***/
    std::ofstream f3;
    f3.open("sigma.txt", std::ios::out | std::ios::app | std::ios::binary);
    for (int i = 0; i < sigma_vec.size(); i += 1) {
        f3 << sigma_vec[i] << std::endl;
    }
    f3.close();
}

int write_meas_to_files() {
    /*** print position trajectory to file ***/
    std::ofstream f1;
    f1.open("slide_data.txt", std::ios::out | std::ios::app | std::ios::binary);
    for (int i = 0; i < x1_vec.size(); i += 1) {
        f1 << x1_vec[i] << std::endl;
    }
    f1.close();

    /*** print position references to file ***/
    std::ofstream f2;
    f2.open("slide_ref.txt", std::ios::out | std::ios::app | std::ios::binary);
    for (int i = 0; i < xref_vec.size(); i += 1) {
        f2 << xref_vec[i] << std::endl;
    }
    f2.close();

    /*** print sigma values to file ***/
    std::ofstream f3;
    f3.open("sigma.txt", std::ios::out | std::ios::app | std::ios::binary);
    for (int i = 0; i < sigma_vec.size(); i += 1) {
        f3 << sigma_vec[i] << std::endl;
    }
    f3.close();

    /*** print control signals to file ***/
    std::ofstream f4;
    f4.open("control_signal.txt", std::ios::out | std::ios::app | std::ios::binary);
    for (int i = 0; i < u_vec.size(); i += 1) {
        f4 << u_vec[i] << std::endl;
    }
    f4.close();

    /*** print LgB to file ***/
    std::ofstream f5;
    f5.open("LgB.txt", std::ios::out | std::ios::app | std::ios::binary);
    for (int i = 0; i < LgB_vec.size(); i += 1) {
        f5 << LgB_vec[i] << std::endl;
    }
    f5.close();

    /*** print LfB to file ***/
    std::ofstream f6;
    f6.open("LfB.txt", std::ios::out | std::ios::app | std::ios::binary);
    for (int i = 0; i < LfB_vec.size(); i += 1) {
        f6 << LfB_vec[i] << std::endl;
    }
    f6.close();

    /*** print error to file ***/
    std::ofstream f7;
    f7.open("err.txt", std::ios::out | std::ios::app | std::ios::binary);
    for (int i = 0; i < err_vec.size(); i += 1) {
        f7 << err_vec[i] << std::endl;
    }
    f7.close();

    /*** print estimated position to file ***/
    std::ofstream f8;
    f8.open("x1_hat.txt", std::ios::out | std::ios::app | std::ios::binary);
    for (int i = 0; i < err_vec.size(); i += 1) {
        f8 << x1_hat_vec[i] << std::endl;
    }
    f8.close();

    /*** print estimated velocity to file ***/
    std::ofstream f9;
    f9.open("x2_hat.txt", std::ios::out | std::ios::app | std::ios::binary);
    for (int i = 0; i < err_vec.size(); i += 1) {
        f9 << x2_hat_vec[i] << std::endl;
    }
    f7.close();

    /*** print duration to file ***/
    std::ofstream f10;
    f10.open("dur.txt", std::ios::out | std::ios::app | std::ios::binary);
        for (int i = 0; i < dur_vec.size(); i += 1) {
    f10 << dur_vec[i] << std::endl;
    }
    f10.close();
}

int slide_angles() {
    std::cout << "test" << std::endl;
 
    while(1) {
         /*** prepare to publish control signal ***/ 
        ros::NodeHandle node;
        ros::Publisher setpoints_pub_pitch = node.advertise<std_msgs::Float64>("pitch_command", 1);
        ros::Publisher setpoints_pub_slide = node.advertise<std_msgs::Float64>("slide_command", 1);
        ros::Publisher setpoints_pub_roll = node.advertise<std_msgs::Float64>("roll_command", 1);

        std::cout << "Give input in the order: [PITCH - ROLL - SLIDE]" << std::endl;
        double input;
        std::vector<double> control_signals;
        for (int i = 0; i < 3; i += 1) {
            std::cin >> input;
            if (input == 9) {
                return 0;
            }
            control_signals.push_back(input);
        }
   
        /*** execute p4_hand_pith ***/
        std_msgs::Float64 u_msg_0;
        u_msg_0.data = control_signals[0];
        setpoints_pub_pitch.publish(u_msg_0);

        /*** execute p4_hand_roll ***/
        std_msgs::Float64 u_msg_1;
        u_msg_1.data = control_signals[1];
        setpoints_pub_roll.publish(u_msg_1);

        /*** execute p4_hand_slide ***/
        std_msgs::Float64 u_msg_2;
        u_msg_2.data = control_signals[2];
        setpoints_pub_slide.publish(u_msg_2);
        
        sleep(1);
 
    }
    return 0;
}

int demo() {
    std::cout << "running drunning demo.." << std::endl;
    demo_func();
    return 0;

}
