#include<controller/controller_kuka.hpp>
#include<random>


int main(int argc, char *argv[])
{
	unsigned int				CycleCounter	=	0;
	int							ResultValue		=	0;
	double 						Time = 0.0;
	double						frequency = 1.0;
	double 						tf = 10.0;

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

	Kuka_Vec Torques_nom;

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

	//std::string Mode("impedence");
	
	std::string Mode("position");
	
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

	bool FLAG = ROBOT_CONTROL;

	controller_kuka Controller(Mode,FLAG);

	if (Controller.FRI->IsMachineOK())
	{
		Controller.MeasureJointPositions();

		for(int i=0;i<NUMBER_OF_JOINTS;i++)
		{
			std::cout<<Controller.JointValuesInRad[i] <<"\n";
			Controller.Q(i) = Controller.JointValuesInRad[i];
			Controller.Qold(i) = Controller.Q(i);
			Q0(i) = Controller.Q(i);
		}
	}

	std::cout << "RUN TIME" << RUN_TIME_IN_SECONDS << "\n";	

	Controller.Qold = Controller.Q;

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

	// Gain without uncertainties and no gravity
	/*
	Eigen::Matrix<double, 2, 4> PD_Gain;

	PD_Gain(0,0) = -93.1726;
	PD_Gain(0,1) = -8.4440;
	PD_Gain(0,2) = -21.5335;
	PD_Gain(0,3) = -4.7909;
	PD_Gain(1,0) = -17.7032;
	PD_Gain(1,1) = -34.4521;
	PD_Gain(1,2) = -6.6227;
	PD_Gain(1,3) = -20.5367;
	*/ 

	// Gain with uncertain mass and no gravity
	/*
	Eigen::Matrix<double, 2, 4> PD_Gain;

	PD_Gain(0,0) = -206.7802;
	PD_Gain(0,1) = 25.1567;
	PD_Gain(0,2) = -162.2236;
	PD_Gain(0,3) = 2.4361;
	PD_Gain(1,0) = -20.1485;
	PD_Gain(1,1) = -220.7650;
	PD_Gain(1,2) = -5.0760;
	PD_Gain(1,3) = -131.8218;
	*/

	// Gain with gravity
	/*
	Eigen::Matrix<double, 2, 4> PD_Gain;

	PD_Gain(0,0) = 4976;
	PD_Gain(0,1) = 6267;
	PD_Gain(0,2) = 113;
	PD_Gain(0,3) = 769;
	PD_Gain(1,0) = -20189;
	PD_Gain(1,1) = -19397;
	PD_Gain(1,2) = -1424;
	PD_Gain(1,3) = -2541;
	*/

	// Variables for internal model

	Eigen::Matrix<double, 10, 1> eta;
	
	eta(0) = 0;
	eta(1) = 0;
	eta(2) = 0;
	eta(3) = 0;
	eta(4) = 0;
	eta(5) = 0;
	eta(6) = 0;
	eta(7) = 0;
	eta(8) = 0;
	eta(9) = 0;

	std::vector<Eigen::Matrix<double, 10, 1>> eta_vec;

	Eigen::Matrix<double, 10, 1> eta_dot;
	
	eta_dot(0) = 0;
	eta_dot(1) = 0;
	eta_dot(2) = 0;
	eta_dot(3) = 0;
	eta_dot(4) = 0;
	eta_dot(5) = 0;
	eta_dot(6) = 0;
	eta_dot(7) = 0;
	eta_dot(8) = 0;
	eta_dot(9) = 0;

	Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(10,10);

	Phi.topRightCorner(8,8).setIdentity();
	Phi(8,2) = -9;
	Phi(9,3) = -9;
	Phi(8,6) = -10;
	Phi(9,7) = -10;

	Eigen::MatrixXd Gamma = Eigen::MatrixXd::Zero(10,2);
	Gamma(8,0) = 1;
	Gamma(9,1) = 1;

	// Gain for output regulation

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

	Eigen::Matrix<double, 2, 1> error;

	Eigen::Matrix<double, 2, 1> vel_error;

	Eigen::Matrix<double, 2, 1> control {0.0, 0.0};

	// virtual saturation
	double th1 = 1000;
	double th2 = 1000;

	double DELTAT_2R = 0.0001;

	double Time_2R = 0.0;

	// file for the evolution of the internal model

	std::string eta_save = "eta.txt";

	std::ofstream file(eta_save);

	std::string error_save = "error.txt";

	std::ofstream file_error(error_save);

	// PD Gains

	double P_Gain = 50.0;
	double D_Gain = 10.0;

	// threshold

	double th = 1000.0;
	
	//SIMULATION LOOP
	while ((float)CycleCounter * Controller.FRI->GetFRICycleTime() < RUN_TIME_IN_SECONDS)
	{		

		Time = Controller.FRI->GetFRICycleTime() * (float)CycleCounter;
		
		error(0) = Controller.Q(1)-0.4*cos(Time_2R);
		error(1) = Controller.Q(3)+0.4*cos(Time_2R);

		std::cout << "error \n" << error << std::endl;

		file_error << error.transpose();

		file_error << "\n";

		Time_2R += DELTAT_2R;

		// INTERNAL MODEL

		eta_dot = Phi * eta + Gamma * error;

		eta = eta + DELTAT_2R * eta_dot;

		eta_vec.push_back(eta);

		file << eta.transpose();

		file << "\n";

		// 2R SIMULATION

		Mass_2R(0,0) = std::pow(2,l2)*m2+2*l1*l2*m2*cos(State_2R(1))+std::pow(2,l1)*(m1+m2);
		Mass_2R(0,1) = std::pow(2,l2)*m2+l1*l2*m2*cos(State_2R(1));
		Mass_2R(1,0) = std::pow(2,l2)*m2+l1*l2*m2*cos(State_2R(1));
		Mass_2R(1,1) = std::pow(2,l2)*m2;

		Coriolis_2R(0) = -m2*l1*l2*sin(State_2R(1))*std::pow(2,State_2R(3))-2*m2*l1*l2*sin(State_2R(1))*State_2R(2)*State_2R(3);
		Coriolis_2R(1) = m2*l1*l2*sin(State_2R(1))*std::pow(2,State_2R(2));

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

		//control = PD_Gain * State_2R;
		control = Theta * state_output;
		
		if ( control(0) >= th )
		{
			control(0) = th;
			std::cout << "saturating" << std::endl;
		}
		if ( control(1) >= th )
		{
			control(1) = th;
			std::cout << "saturating" << std::endl;
		}
		if ( control(0) <= -th )
		{
			control(0) = -th;
			std::cout << "saturating" << std::endl;
		}
		if ( control(1) <= -th )
		{
			control(1) = -th;
			std::cout << "saturating" << std::endl;
		}
		
		//acc_2R = Mass_2R.inverse()*(control - Coriolis_2R);
		acc_2R = Mass_2R.inverse()*(control - Coriolis_2R - Gravity_2R);

		vel_2R = vel_2R + DELTAT_2R * acc_2R;

		pos_2R = pos_2R + DELTAT_2R * vel_2R;

		State_2R(0) = pos_2R(0);
		State_2R(1) = pos_2R(1);
		State_2R(2) = vel_2R(0);
		State_2R(3) = vel_2R(1);

		// Assigning the values of the 2R to the respective joints of the KUKA

		Q_ref = Q0;

		// Planar vertical 2R
		Q_ref(1) = pos_2R(0);
		Q_ref(3) = pos_2R(1);

		/*
		Q_ref = Q0 + Kuka_Vec::Constant(0.2*(1.0 - std::cos(Time)));
		
		dQ_ref = Kuka_Vec::Constant(0.2*std::sin(Time));
		
		d2Q_ref = Kuka_Vec::Constant(0.2*std::cos(Time));
		
		Q_ref(0) = Q0(0);
		Q_ref(2) = Q0(2);
		Q_ref(4) = Q0(4);
		Q_ref(5) = Q0(5);
		Q_ref(6) = Q0(6);
		
		dQ_ref(4) = 0.0;
		dQ_ref(5) = 0.0;
		dQ_ref(6) = 0.0;

		d2Q_ref(4) = 0.0;
		d2Q_ref(5) = 0.0;
		d2Q_ref(6) = 0.0;
		*/		

		Tic = std::chrono::system_clock::now();

		Controller.FRI->WaitForKRCTick(TimeoutValueInMicroSeconds);

		Toc = std::chrono::system_clock::now();

		if (!Controller.FRI->IsMachineOK())
		{
			fprintf(stderr, "ERROR, the machine is not ready anymore.\n");
			break;
		}

		state = Controller.GetState(FLAG);

		// FULL-STATE PBSERVER: Nicosia-Tomei

		/*
		y_tilda = Controller.Q - Controller.Q_hat;

		dx1_hat = Controller.dQ_hat + Controller.kd*y_tilda;

		d2Q_hat = Controller.SimObserver(Controller.Q, y_tilda, dx1_hat, Torques_nom);

		Controller.Q_hat = Controller.EulerIntegration(dx1_hat, Controller.Q_hat);

		Controller.dQ_hat = Controller.EulerIntegration(d2Q_hat, Controller.dQ_hat);
		*/	

		Controller.Qsave.push_back(Controller.Q);

		Controller.dQsave.push_back(Controller.dQ);	

		// UPDATING COORDINATES

		Controller.Qold = Controller.Q;

		Controller.dQold = Controller.dQ;

		Controller.dQ = Controller.EulerIntegration(Controller.d2Q,Controller.dQ);

		Controller.Q = Controller.EulerIntegration(Controller.dQ,Controller.Q);

		Controller.dQ_num = Controller.EulerDifferentiation(Controller.Q,Controller.Qold);
		
		Controller.GetState(FLAG); 

		//ARRAY SAVING

		Q_filtered = Controller.Filter(Controller.Qsave,FILTER_LENGTH);

		dQ_filtered = Controller.Filter(Controller.dQsave,FILTER_LENGTH);

		Controller.d2Q = Controller.EulerDifferentiation(dQ_filtered,Controller.dQsave_filtered.back());

		Controller.d2Qsave.push_back(Controller.d2Q);

		d2Q_filtered = Controller.Filter(Controller.d2Qsave,FILTER_LENGTH);

		Controller.Qsave_filtered.push_back(Q_filtered);
		
		Controller.dQsave_filtered.push_back(dQ_filtered);
		
		Controller.d2Qsave_filtered.push_back(d2Q_filtered);

		// Saving state reference values

		Q_ref_vec.push_back(Q_ref);

		dQ_ref_vec.push_back(dQ_ref);

		d2Q_ref_vec.push_back(d2Q_ref);

		std::cout << "Time = " << Time << std::endl;

		//SAFETY CHECK FOR POSITIONS AND VELOCITIES
		
		if((!Controller.VelocitySafety(Controller.dQ)) || (!Controller.JointSafety(Controller.Q)))
		{	
			//EXITING THE CONTROL LOOP
			std::cout << "Breaking safety controllers for either Velocities and Joints position \n";
			break;
		}

		// POSITION CONTROL

		for(int y=0;y<NUMBER_OF_JOINTS;y++)
		{
			Controller.JointValuesInRad[y] = Q_ref(y);
		}
		
		Controller.FRI->SetCommandedJointPositions(Controller.JointValuesInRad);

		//noisy_torque_vec.push_back(Torques_measured);

		Controller.Tor_th.push_back(Torques_ref);

		// REDUCED OBSERVER

		Controller.dz = Controller.SimReducedObserver(Controller.Q, Controller.dQ_hat, Torques_nom);

		Controller.z = Controller.EulerIntegration(Controller.dz, Controller.z);

		Controller.dQ_hat = Controller.z + Controller.k0*Controller.Q;

		// Saving estimated quantities

		Controller.dQ_hat_save.push_back(Controller.dQ_hat);

		Controller.Q_hat_save.push_back(Controller.Q_hat);

		CycleCounter++;
	}

	fprintf(stdout, "Stopping the robot...\n");

	ResultValue	= Controller.FRI->StopRobot();

	if (ResultValue != EOK)
	{
		fprintf(stderr, "An error occurred during stopping the robot...\n");
	}
	else
	{
		fprintf(stdout, "Robot successfully stopped.\n");
	}

	//Writing Experiment Variables

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
