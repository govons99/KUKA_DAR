#include<controller/controller_kuka.hpp>
#include<random>
#include <iostream>
#include <fstream>


int main(int argc, char *argv[])
{
	unsigned int				CycleCounter	=	0;
	double 						Time = 0.0;
	double						frequency = 1.0;
	double 						tf = 800.0;

	std::chrono::time_point<std::chrono::system_clock> Tic, Toc;

	double elapsed_time;

	unsigned int TimeoutValueInMicroSeconds = 0;

	Kuka_Vec Q0;

	Kuka_Vec dQ0 = Kuka_Vec::Constant(NUMBER_OF_JOINTS,0.0);

	Kuka_Vec Q_ref;

	Kuka_Vec dQ_ref;

	Kuka_Vec d2Q_ref;

	Kuka_Vec Q_meas;

	Kuka_Vec Q_meas_old;

	Kuka_Vec dQ_meas; //obtained from the measured positions

	Kuka_Vec dQ_meas_old;

	Kuka_Vec d2Q_meas; //obtained from the measured positions

	Kuka_Vec G;

	Kuka_Vec Torques_ref;

	Kuka_Vec Torques_nom = Kuka_Vec::Constant(0.0);

	Kuka_Vec Torques_measured;

	Kuka_Vec Torques_filtered;

	Kuka_Vec torques_temp;

	Kuka_Vec temp_Vec;

	Kuka_Vec Q_filtered;

	Kuka_Vec dQ_filtered;

	Kuka_Vec d2Q_filtered;

	std::vector<Kuka_Vec> Q_ref_vec;

	std::vector<Kuka_Vec> dQ_ref_vec;

	std::vector<Kuka_Vec> d2Q_ref_vec;

	std::vector<Kuka_Vec> Temp_array;
	
	Kuka_Vec Kuka_temp;

	Kuka_Vec Kuka_temp2;

	std::vector<Eigen::VectorXd> temp;

	Kuka_Mat Mass;

	std::string Mode("impedence");
	
	//std::string Mode("position");
	
	std::string qsave = "Q.txt";
  	std::string dqsave = "dQ.txt";
	std::string d2qsave = "d2Q.txt";
	std::string qsave_filtered = "Q_filtered.txt";
  	std::string dqsave_filtered = "dQ_filtered.txt";
	std::string d2qsave_filtered = "d2Q_filtered.txt";
	std::string dqsave_meas = "dQ_meas.txt";
	std::string torque_meas = "tor_meas.txt";
	std::string torque_th = "tor_th.txt";
	std::string friction = "friction.txt";
	std::string Q_ref_file = "Qref.txt";
	std::string dQ_ref_file = "dQref.txt";
	std::string d2Q_ref_file = "d2Qref.txt";
	std::string qhatsave = "Q_hat.txt";
	std::string dqhatsave = "dQ_hat.txt";
	std::string dqnumsave = "dQ_num.txt";
	std::string torque_save = "torque_ref.txt";

	Kuka_State state;

	bool FLAG = SIM_CONTROL;

	controller_kuka Controller(Mode,FLAG);

	std::cout << "Simulation LOOP" << "\n";

	std::cout << "\n Remember to insert friction compensation for REAL robot experiment \n";

	Controller.Qold = Controller.Q;

	Q0 = Controller.Q;

	Q_meas = Q0;

	dQ_meas = Kuka_Vec::Constant(0.0);

	d2Q_meas = Kuka_Vec::Constant(0.0);

	Q_filtered = Kuka_Vec::Constant(0.0);

	dQ_filtered = Kuka_Vec::Constant(0.0);

	d2Q_filtered = Kuka_Vec::Constant(0.0);

	//Variables for the full state observer

	Kuka_Vec y_tilda = Kuka_Vec::Constant(0.0);
	Kuka_Vec dx1_hat = Kuka_Vec::Constant(0.0);
	Kuka_Vec d2Q_hat = Kuka_Vec::Constant(0.0);

	//Initialization estimated varibales
            
	//Observer
	Controller.Q_hat = Controller.Q;
	Controller.dQ_hat = Kuka_Vec::Constant(0.0);

	Controller.Q_hat_save.push_back(Controller.Q_hat);
    Controller.dQ_hat_save.push_back(Controller.dQ_hat);

	//Initial generalized momentum

	Mass = Controller.GetMass(Controller.Q);
	Controller.p0 = Mass*Controller.dQ;
	Controller.p0_hat = Mass*Controller.dQ_hat;

	//Variables for the 2R planar robot

	double m1 = 2.6;
	double m2 = 2.6;
	double l1 = 0.5;
	double l2 = 0.5;

	Eigen::Matrix<double, 2, 2> Mass_2R;
	Eigen::Matrix<double, 2, 1> Coriolis_2R;
	Eigen::Matrix<double, 2, 2> Coriolis_fact_2R;
	Eigen::Matrix<double, 2, 1> Coriolis_2R_hat;
	Eigen::Matrix<double, 2, 2> Coriolis_fact_2R_hat;
	Eigen::Matrix<double, 2, 1> Gravity_2R;
	
	// horizontal plane
	/*
	Eigen::Matrix<double, 4, 1> State_2R = {Controller.Q(0), Controller.Q(3), 0, 0};
	Eigen::Matrix<double, 2, 1> acc_2R {0, 0};
	Eigen::Matrix<double, 2, 1> vel_2R {0, 0};
	Eigen::Matrix<double, 2, 1> pos_2R {Controller.Q(0), Controller.Q(3)};
	*/
	
	// vertical plane
	
	Eigen::Matrix<double, 4, 1> State_2R = {Controller.Q(1), Controller.Q(3), 0, 0};
	Eigen::Matrix<double, 2, 1> acc_2R {0, 0};
	Eigen::Matrix<double, 2, 1> vel_2R {0, 0};
	Eigen::Matrix<double, 2, 1> pos_2R {Controller.Q(1), Controller.Q(3)};

	Eigen::Matrix<double, 2, 1> z_2R {0, 0};
	Eigen::Matrix<double, 2, 1> dz_2R {0, 0};
	Eigen::Matrix<double, 2, 1> vel_2R_hat {0, 0};

	// old values
	/*
	Eigen::Matrix<double, 2, 4> PD_Gain {
		{-21.4267, -0.8640, -17.2654, -1.0133},
		{-4.8761, -17.7497, -3.9620, -12.6837},
	};
	*/

	// 25/10
	/*
	Eigen::Matrix<double, 2, 4> PD_Gain {
		{-93.1726, -8.4440, -21.5335, -4.7909},
		{-17.7032, -34.4521, -6.6227, -20.5367},
	};
	*/
	
	// Gain with gravity

	Eigen::Matrix<double, 2, 4> PD_Gain;

	PD_Gain(0,0) = 4976;
	PD_Gain(0,1) = 6267;
	PD_Gain(0,2) = 113;
	PD_Gain(0,3) = 769;
	PD_Gain(1,0) = -20189;
	PD_Gain(1,1) = -19397;
	PD_Gain(1,2) = -1424;
	PD_Gain(1,3) = -2541;

	Eigen::Matrix<double, 2, 1> control {0, 0};

	// OUTPUT REGULATION

	// REFERENCE TRAJECTORY: ref = [cos(t);-cos(t)]
	
	// Variables for internal model
	
	Eigen::Matrix<double, 10, 1> eta {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	Eigen::Matrix<double, 10, 1> eta_dot {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(10,10);

	Phi.topRightCorner(8,8).setIdentity();
	Phi(8,2) = -9;
	Phi(9,3) = -9;
	Phi(8,6) = -10;
	Phi(9,7) = -10;

	Eigen::MatrixXd Gamma = Eigen::MatrixXd::Zero(10,2);
	Gamma(8,0) = 1;
	Gamma(9,1) = 1;
	
	// Gain control action

	Eigen::Matrix<double, 2, 14> Theta;

	Theta(0,0) = -9277.13944860092;             
	Theta(0,1) = 3710.17708672289;
	Theta(0,2) = -349.234832428913;
	Theta(0,3) = 266.395117894730;
	Theta(0,4) = -194809.308523000;
	Theta(0,5) = 29590.8387072452;
	Theta(0,6) = -497382.571831891;
	Theta(0,7) = 83024.4056398725;
	Theta(0,8) = -512598.396322723;
	Theta(0,9) = 96517.0532040094;
	Theta(0,10) = -268819.441373604;
	Theta(0,11) = 59336.0150773882;
	Theta(0,12) = -73589.1199037631;
	Theta(0,13) = 20303.8593336261;

	Theta(1,0) = -2029.21453824429;            
	Theta(1,1) = -7028.36935443040;
	Theta(1,2) = -80.0713649245077;
	Theta(1,3) = -523.663369755656;
	Theta(1,4) = -42740.8155134585;
	Theta(1,5) = -58507.6257749021;
	Theta(1,6) = -108901.648552769;
	Theta(1,7) = -163109.287869734;
	Theta(1,8) = -111981.948671448;
	Theta(1,9) = -188381.316026516;
	Theta(1,10) = -58595.8106911847;
	Theta(1,11) = -115090.430611396;
	Theta(1,12) = -16021.4507995792;
	Theta(1,13) = -39186.9823159717;

	Eigen::Matrix<double, 14, 1> state_output;

	double w1 = 0.0;
	double w2 = 0.0;

	double w1_dot = 0.0;
	double w2_dot = 0.0;

	double w1_ddot = 0.0;
	double w2_ddot = 0.0;

	Eigen::MatrixXd qd = Eigen::MatrixXd::Zero(2,1);
	Eigen::MatrixXd qd_dot = Eigen::MatrixXd::Zero(2,1);
	Eigen::MatrixXd qd_ddot = Eigen::MatrixXd::Zero(2,1);

	/*

	// REFERENCE TRAJECTORY: ref = [a0 + a1*cos(t*f) + b1*sin(t*f);a0 + a1*cos(t*f) + b1*sin(t*f)]
	// circular trajectory in the task space

	// Variables trajectory

	double a01 = 0.1444;
	double a11 = -0.4403;
	double b11 = -0.2636;
	double f1 = 5;

	double a02 = 0.9537;
	double a12 = 0.3598;
	double b12 = -0.2501;
	double f2 = 5;
	
	// VERSION 1: SMALLER INTERNAL MODEL
	
	// Variables for internal model

	Eigen::MatrixXd eta = Eigen::MatrixXd::Zero(6,1);
	Eigen::MatrixXd eta_dot = Eigen::MatrixXd::Zero(6,1);

	Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(6,6);

	Phi.topRightCorner(4,4).setIdentity();
	Phi(4,2) = -f1*f1;
	Phi(5,3) = -f1*f1;

	Eigen::MatrixXd Gamma = Eigen::MatrixXd::Zero(6,2);
	Gamma(4,0) = 1;
	Gamma(5,1) = 1;
	
	// Gain control action

	Eigen::Matrix<double, 2, 10> Theta;	         

	Theta(0,0) = -51479.1506564270;             
	Theta(0,1) = 3568.82244932025;
	Theta(0,2) = -11854.7020446130;
	Theta(0,3) = 1079.87266377154;
	Theta(0,4) = -633338.094409722;
	Theta(0,5) = 4333.26737636959;
	Theta(0,6) = 508505.390501465;
	Theta(0,7) = -47862.5287158016;
	Theta(0,8) = -98358.8518500649;
	Theta(0,9) = 6135.56008409700;	         

	Theta(1,0) = 1263.68859842101;            
	Theta(1,1) = -44137.3005591795;
	Theta(1,2) = 480.698288000380;
	Theta(1,3) = -10896.5985110252;
	Theta(1,4) = -9654.94397920896;
	Theta(1,5) = -440771.759801160;
	Theta(1,6) = -20520.6564484375;
	Theta(1,7) = 470082.073219219;
	Theta(1,8) = 1813.31207683121;
	Theta(1,9) = -82035.0166304382;

	Eigen::Matrix<double, 10, 1> state_output;

	Eigen::Matrix<double, 2, 1> error;
	

	// VERSION 2: BIGGER INTERNAL MODEL
	
	// Variables for internal model

	Eigen::MatrixXd eta = Eigen::MatrixXd::Zero(10,1);
	Eigen::MatrixXd eta_dot = Eigen::MatrixXd::Zero(10,1);

	Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(10,10);

	Phi.topRightCorner(8,8).setIdentity();
	Phi(8,2) = -2500;
	Phi(9,3) = -2500;
	Phi(8,6) = -125;
	Phi(9,7) = -125;

	Eigen::MatrixXd Gamma = Eigen::MatrixXd::Zero(10,2);
	Gamma(8,0) = 1;
	Gamma(9,1) = 1;
	
	// Gain control action

	Eigen::Matrix<double, 2, 14> Theta;

	Theta(0,0) = -129063.234329273;
	Theta(0,1) = -8253.61733638287;
	Theta(0,2) = -21258.2154853813;	 
	Theta(0,3) = -1324.23747000441;	
	Theta(0,4) = -145241897.721119;
	Theta(0,5) = -11328810.1611666;
	Theta(0,6) = 163926657.222489;
	Theta(0,7) = 9697194.71524031;
	Theta(0,8) = -36715181.8277190;
	Theta(0,9) = -2585521.34926766;
	Theta(0,10) = 3514222.37199434;
	Theta(0,11) = 206895.178310177;
	Theta(0,12) = -664621.863400813; 
	Theta(0,13) = -44864.4682637700;

	Theta(1,0) = -2587.33817423114; 
	Theta(1,1) = -129886.665528651; 
	Theta(1,2) = -376.307319535656; 
	Theta(1,3) = -21615.2560613970; 
	Theta(1,4) = -5900656.83433174; 
	Theta(1,5) = -132091352.572063; 
	Theta(1,6) = 2120383.84030273; 
	Theta(1,7) = 170544383.395149; 
	Theta(1,8) = -1089047.87621256; 
	Theta(1,9) = -35228301.3475273; 
	Theta(1,10) = 43638.9933715833; 
	Theta(1,11) = 3668404.49718848; 
	Theta(1,12) = -16889.6496944827;            
	Theta(1,13) =  -650798.971089615;

	Eigen::Matrix<double, 14, 1> state_output;

	*/

	// SLIDING MODE CONTROL

	// reference state vector
	Eigen::MatrixXd sr = Eigen::MatrixXd::Zero(2,1);
	Eigen::MatrixXd sr_dot = Eigen::MatrixXd::Zero(2,1);
	// sliding surface
	Eigen::MatrixXd s = Eigen::MatrixXd::Zero(2,1);
	
	// gains
	double lambda = 8.0;
	double k = 150;
	Eigen::MatrixXd phi(2,1);
	phi(0) = 0.09;
	phi(1) = 0.03;

	Eigen::Matrix<double, 2, 1> error;

	Eigen::Matrix<double, 2, 1> vel_error;

	double DELTAT_2R = 0.0001;

	double Time_2R = 0.0;

	double th = 15000;

	// file for the evolution of the internal model

	std::string eta_save = "eta.txt";

	std::ofstream file(eta_save); 

	std::string error_save = "error.txt";

	std::ofstream file_error(error_save);

	// computed torque gains

	double P_Gain = 70.0;
	double D_Gain = 20.0;
	
	//SIMULATION LOOP
	while (Time < tf)
	{		

		Time += DELTAT;	

		w1 = 0.4*cos(Time_2R);
		w2 = -0.4*cos(Time_2R);

		w1_dot = -0.4*sin(Time_2R);
		w2_dot = 0.4*sin(Time_2R);

		w1_ddot = -0.4*cos(Time_2R);
		w2_ddot = 0.4*cos(Time_2R);		

		qd(0) = w1;
		qd(1) = w2;

		qd_dot(0) = w1_dot;
		qd_dot(1) = w2_dot;

		qd_ddot(0) = w1_ddot;
		qd_ddot(1) = w2_ddot;

		/*
		w1 = a01 + a11*cos(Time_2R*f1) + b11*sin(Time_2R*f1);
		w2 = a02 + a12*cos(Time_2R*f2) + b12*sin(Time_2R*f2);
		*/

		error(0) = pos_2R(0)-w1;
		error(1) = pos_2R(1)-w2; 

		// vel_error(0) = vel_2R(0)-w1_dot;
		// vel_error(1) = vel_2R(1)-w1_dot;

		vel_error(0) = vel_2R_hat(0)-w1_dot;
		vel_error(1) = vel_2R_hat(1)-w1_dot;

		std::cout << error << "\n" << std::endl;

		file_error << error.transpose();

		file_error << "\n";

		Time_2R += DELTAT_2R;

		// SLIDING

		sr = qd_dot - lambda*( pos_2R - qd );
		s = vel_2R_hat - sr;
		sr_dot = qd_ddot - lambda*vel_error;

		// INTERNAL MODEL

		eta_dot = Phi * eta + Gamma * error;

		eta = eta + DELTAT_2R * eta_dot;

		file << eta.transpose();

		file << "\n";

		// 2R SIMULATION

		Mass_2R(0,0) = std::pow(2,l2)*m2+2*l1*l2*m2*cos(State_2R(1))+std::pow(2,l1)*(m1+m2);
		Mass_2R(0,1) = std::pow(2,l2)*m2+l1*l2*m2*cos(State_2R(1));
		Mass_2R(1,0) = std::pow(2,l2)*m2+l1*l2*m2*cos(State_2R(1));
		Mass_2R(1,1) = std::pow(2,l2)*m2;

		Coriolis_2R(0) = -m2*l1*l2*sin(State_2R(1))*std::pow(2,State_2R(3))-2*m2*l1*l2*sin(State_2R(1))*State_2R(2)*State_2R(3);
		Coriolis_2R(1) = m2*l1*l2*sin(State_2R(1))*std::pow(2,State_2R(2));

		Coriolis_2R_hat(0) = -m2*l1*l2*sin(State_2R(1))*std::pow(2,vel_2R_hat(1))-2*m2*l1*l2*sin(State_2R(1))*vel_2R_hat(0)*vel_2R_hat(1);
		Coriolis_2R_hat(1) = m2*l1*l2*sin(State_2R(1))*std::pow(2,vel_2R_hat(0));

		Coriolis_fact_2R(0,0) = -l1*l2*m2*sin(State_2R(1))*State_2R(3);
		Coriolis_fact_2R(0,1) = -l1*l2*m2*sin(State_2R(1))*(State_2R(2)+State_2R(3));
		Coriolis_fact_2R(1,0) = l1*l2*m2*sin(State_2R(1))*State_2R(2);
		Coriolis_fact_2R(1,1) = 0.0;

		Coriolis_fact_2R_hat(0,0) = -l1*l2*m2*sin(State_2R(1))*vel_2R_hat(1);
		Coriolis_fact_2R_hat(0,1) = -l1*l2*m2*sin(State_2R(1))*(vel_2R_hat(0)+vel_2R_hat(1));
		Coriolis_fact_2R_hat(1,0) = l1*l2*m2*sin(State_2R(1))*vel_2R_hat(0);
		Coriolis_fact_2R_hat(1,1) = 0.0;
		
		Gravity_2R(0) = cos(State_2R(0)+State_2R(1))*m2*9.81*l2 + cos(State_2R(0))*(m1+m2)*l1*9.81;
		Gravity_2R(1) = cos(State_2R(0)+State_2R(1))*m2*9.81*l2;

		state_output(0) = State_2R(0);
		state_output(1) = State_2R(1);
		state_output(2) = State_2R(2);
		state_output(3) = State_2R(3);
		state_output(4) = eta(0);
		state_output(5) = eta(1);
		state_output(6) = eta(2);
		state_output(7) = eta(3);
		state_output(8) = eta(4);
		state_output(9) = eta(5);
		state_output(10) = eta(6);
		state_output(11) = eta(7);
		state_output(12) = eta(8);
		state_output(13) = eta(9);

		// Static reference: Eduardo controller
		// control = PD_Gain * State_2R;

		// Periodic reference: Eduardo controller
		 control = Theta * state_output;

		// "computed torque" --> FBL + [PD + FFW]
		// control = Mass_2R*( qd_ddot + P_Gain * (qd - pos_2R) + D_Gain * (qd_dot - vel_2R)) + Coriolis_2R + Gravity_2R;
		// control = Mass_2R*( qd_ddot + P_Gain * (qd - pos_2R) + D_Gain * (qd_dot - vel_2R_hat)) + Coriolis_2R_hat + Gravity_2R; 

		// sliding mode control
		// control = Mass_2R*sr_dot + Coriolis_fact_2R*sr + Gravity_2R - k*Controller.switching(s,phi);
		// control = Mass_2R*sr_dot + Coriolis_fact_2R_hat*sr + Gravity_2R - k*Controller.switching(s,phi);
		
		/*
		if ( control(0) >= th )
		{
			control(0) = th;
			std::cout << "-------------------------------" << std::endl;
		}
		if ( control(1) >= th )
		{
			control(1) = th;
			std::cout << "-------------------------------" << std::endl;
		}
		if ( control(0) <= -th )
		{
			control(0) = -th;
			std::cout << "-------------------------------" << std::endl;
		}
		if ( control(1) <= -th )
		{
			control(1) = -th;
			std::cout << "-------------------------------" << std::endl;
		}
		*/

		acc_2R = Mass_2R.inverse()*(control - Coriolis_2R - Gravity_2R);

		vel_2R = vel_2R + DELTAT_2R * acc_2R;

		pos_2R = pos_2R + DELTAT_2R * vel_2R;

		State_2R(0) = pos_2R(0);
		State_2R(1) = pos_2R(1);
		State_2R(2) = vel_2R(0);
		State_2R(3) = vel_2R(1);

		// REDUCED OBSERVER

		//Torques_nom(1) = control(0);
		//Torques_nom(3) = control(1);

		//Controller.dz = Controller.SimReducedObserver(Controller.Q, Controller.dQ_hat, Torques_nom);

		//Controller.z = Controller.EulerIntegration(Controller.dz, Controller.z);

		//Controller.dQ_hat = Controller.z + Controller.k0*Controller.Q;

		dz_2R = Controller.SimReducedObserver2R(pos_2R, vel_2R_hat, control);

		z_2R = z_2R + DELTAT_2R*dz_2R;

		vel_2R_hat = z_2R + Controller.k0*pos_2R;

		// std::cout << vel_2R_hat << "\n" << std::endl;

		// Assigning the values of the 2R to the respective joints of the KUKA

		Controller.Q(1) = pos_2R(0);
		Controller.Q(3) = pos_2R(1);

		Controller.dQ(1) = vel_2R(0);
		Controller.dQ(3) = vel_2R(1);

		Controller.dQ_hat(1) = vel_2R_hat(0);
		Controller.dQ_hat(3) = vel_2R_hat(1);

		// FULL-STATE PBSERVER: Nicosia-Tomei

		/*
		y_tilda = Controller.Q - Controller.Q_hat;

		dx1_hat = Controller.dQ_hat + Controller.kd*y_tilda;

		d2Q_hat = Controller.SimObserver(Controller.Q, y_tilda, dx1_hat, Torques_nom);

		Controller.Q_hat = Controller.EulerIntegration(dx1_hat, Controller.Q_hat);

		Controller.dQ_hat = Controller.EulerIntegration(d2Q_hat, Controller.dQ_hat);
		*/		

		// UPDATING COORDINATES

		Controller.Qold = Controller.Q;

		Controller.dQold = Controller.dQ;

		Controller.dQ = Controller.EulerIntegration(Controller.d2Q,Controller.dQ);

		Controller.Q = Controller.EulerIntegration(Controller.dQ,Controller.Q);

		Controller.dQ_num = Controller.EulerDifferentiation(Controller.Q,Controller.Qold);
		
		Controller.GetState(FLAG); 

		//ARRAY SAVING
		
		Controller.Qsave.push_back(Controller.Q);

		Controller.dQsave.push_back(Controller.dQ);

		Controller.d2Qsave.push_back(Controller.d2Q);

		Controller.dQ_hat_save.push_back(Controller.dQ_hat);

		Controller.Q_hat_save.push_back(Controller.Q_hat);

		Controller.dQ_num_save.push_back(Controller.dQ_num);

		Q_ref_vec.push_back(Q_ref);

		dQ_ref_vec.push_back(dQ_ref);

		d2Q_ref_vec.push_back(d2Q_ref);

		//noisy_torque_vec.push_back(Torques_measured);

		Controller.Tor_th.push_back(Torques_ref);

		CycleCounter++;

		std::cout << "Time = " << Time << std::endl;
	}

	//Writing Simulation Variables

	file.close();

	file_error.close();

	Controller.FromKukaToDyn(temp,Controller.Qsave);
	Controller.writer.write_data(qsave,temp);
	
	Controller.FromKukaToDyn(temp,Controller.dQsave);
	Controller.writer.write_data(dqsave,temp);

	Controller.FromKukaToDyn(temp,Controller.d2Qsave);
	Controller.writer.write_data(d2qsave,temp);

	Controller.FromKukaToDyn(temp,Controller.dQ_hat_save);
	Controller.writer.write_data(dqhatsave,temp);

	Controller.FromKukaToDyn(temp,Controller.Q_hat_save);
	Controller.writer.write_data(qhatsave,temp);

	Controller.FromKukaToDyn(temp,Controller.dQ_num_save);
	Controller.writer.write_data(dqnumsave,temp);	

	Controller.FromKukaToDyn(temp,Controller.Tor_th);
	Controller.writer.write_data(torque_save,temp);	

	Controller.FromKukaToDyn(temp,Q_ref_vec);
	Controller.writer.write_data(Q_ref_file,temp);

	Controller.FromKukaToDyn(temp,dQ_ref_vec);
	Controller.writer.write_data(dQ_ref_file,temp);

	Controller.FromKukaToDyn(temp,d2Q_ref_vec);
	Controller.writer.write_data(d2Q_ref_file,temp);

	//"Measured" velocities

	Controller.FromKukaToDyn(temp,Controller.dQsave_meas);
	Controller.writer.write_data(dqsave_meas,temp);

	//Filtered variables

	Controller.FromKukaToDyn(temp,Controller.Qsave_filtered);
	Controller.writer.write_data(qsave_filtered,temp);

	Controller.FromKukaToDyn(temp,Controller.dQsave_filtered);
	Controller.writer.write_data(dqsave_filtered,temp);

	Controller.FromKukaToDyn(temp,Controller.d2Qsave_filtered);
	Controller.writer.write_data(d2qsave_filtered,temp);
		

return 0;
}
