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

	Q_filtered = Q0;

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

	Eigen::Matrix<double, 2, 2> Mass_2R_ss;
	Eigen::Matrix<double, 2, 1> Coriolis_2R_ss;
	Eigen::Matrix<double, 2, 1> Gravity_2R_ss;

	// vertical plane
	Eigen::Matrix<double, 4, 1> State_2R = {Controller.Q(1), Controller.Q(3), 0, 0};
	Eigen::Matrix<double, 2, 1> acc_2R {0, 0};
	Eigen::Matrix<double, 2, 1> vel_2R {0, 0};
	Eigen::Matrix<double, 2, 1> pos_2R {Controller.Q(1), Controller.Q(3)};

	// reduced-order observer 2R

	Eigen::Matrix<double, 2, 1> z_2R {0, 0};
	Eigen::Matrix<double, 2, 1> dz_2R {0, 0};
	Eigen::Matrix<double, 2, 1> vel_2R_hat {0, 0};

	// controller gain
	
	Eigen::Matrix<double, 2, 4> K;

	K(0,0) =-303.4526;             
	K(0,1) =10.9148;
	K(0,2) =-40.4150;
	K(0,3) =-8.9930;
	
	K(1,0) =18.9294;            
	K(1,1) =-203.1480;
	K(1,2) =-8.5954;
	K(1,3) =-25.5580;
	

	Eigen::Matrix<double, 4, 1> state_output;

	Eigen::Matrix<double, 2, 1> error;

	Eigen::Matrix<double, 2, 1> vel_error;

	Eigen::Matrix<double, 2, 1> control {0.0, 0.0};

	Eigen::Matrix<double, 2, 1> control_ss {0.0, 0.0};

	double DELTAT_2R = 0.0005;

	double Time_2R = 0.0;

	std::string error_save = "error.txt";

	std::ofstream file_error(error_save);

	std::string control_save = "control_DAR.txt";

	std::ofstream file_control(control_save);

	double ref1 = 0.0;
	double ref2 = 0.0;
	double dref1 = 0.0;
	double dref2 = 0.0;

	Eigen::Matrix<double, 2, 1> ref {ref1, ref2};
	Eigen::Matrix<double, 4, 1> state_ref {ref1, ref2, dref1, dref2};
	
	//SIMULATION LOOP
	while ((float)CycleCounter * Controller.FRI->GetFRICycleTime() < RUN_TIME_IN_SECONDS)
	{		

		Time = Controller.FRI->GetFRICycleTime() * (float)CycleCounter;

		ref1 = 0.4*cos(Time_2R);
		ref2 = -0.4*cos(Time_2R);
		ref(0) = ref1;
		ref(1) = ref2;

		dref1 = -0.4*sin(Time_2R);
		dref2 = 0.4*sin(Time_2R);

		state_ref(0) = ref1;
		state_ref(1) = ref2;
		state_ref(2) = dref1;
		state_ref(3) = dref2;
		
		error(0) = Controller.Q(1)-0.4*cos(Time_2R);
		error(1) = Controller.Q(3)+0.4*cos(Time_2R);

		std::cout << "error \n" << error << std::endl;

		file_error << error.transpose();

		file_error << "\n";

		Time_2R += DELTAT_2R;

		// 2R SIMULATION

		Mass_2R(0,0) = std::pow(2,l2)*m2+2*l1*l2*m2*cos(State_2R(1))+std::pow(2,l1)*(m1+m2);
		Mass_2R(0,1) = std::pow(2,l2)*m2+l1*l2*m2*cos(State_2R(1));
		Mass_2R(1,0) = std::pow(2,l2)*m2+l1*l2*m2*cos(State_2R(1));
		Mass_2R(1,1) = std::pow(2,l2)*m2;

		Coriolis_2R(0) = -m2*l1*l2*sin(State_2R(1))*std::pow(2,State_2R(3))-2*m2*l1*l2*sin(State_2R(1))*State_2R(2)*State_2R(3);
		Coriolis_2R(1) = m2*l1*l2*sin(State_2R(1))*std::pow(2,State_2R(2));

		Gravity_2R(0) = cos(State_2R(0)+State_2R(1))*m2*9.81*l2 + cos(State_2R(0))*(m1+m2)*l1*9.81;
		Gravity_2R(1) = cos(State_2R(0)+State_2R(1))*m2*9.81*l2;

		// steady-state values

		Mass_2R_ss(0,0) = std::pow(2,l2)*m2+2*l1*l2*m2*cos(ref2)+std::pow(2,l1)*(m1+m2);
		Mass_2R_ss(0,1) = std::pow(2,l2)*m2+l1*l2*m2*cos(ref2);
		Mass_2R_ss(1,0) = std::pow(2,l2)*m2+l1*l2*m2*cos(ref2);
		Mass_2R_ss(1,1) = std::pow(2,l2)*m2;

		Coriolis_2R_ss(0) = -m2*l1*l2*sin(ref2)*std::pow(2,dref2)-2*m2*l1*l2*sin(ref2)*dref1*dref2;
		Coriolis_2R_ss(1) = m2*l1*l2*sin(ref2)*std::pow(2,dref1);

		Gravity_2R_ss(0) = cos(ref1+ref2)*m2*9.81*l2 + cos(ref1)*(m1+m2)*l1*9.81;
		Gravity_2R_ss(1) = cos(ref1+ref2)*m2*9.81*l2;

		state_output(0) = Controller.Q(1);
		state_output(1) = Controller.Q(3);
		state_output(2) = dQ_filtered(1);
		state_output(3) = dQ_filtered(3);

		control_ss = -Mass_2R_ss*ref + Coriolis_2R_ss + Gravity_2R_ss;

		control = control_ss + K * (state_output - state_ref);

		file_control << control.transpose();

		file_control << "\n";
		
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

		Tic = std::chrono::system_clock::now();

		Controller.FRI->WaitForKRCTick(TimeoutValueInMicroSeconds);

		Toc = std::chrono::system_clock::now();

		if (!Controller.FRI->IsMachineOK())
		{
			fprintf(stderr, "ERROR, the machine is not ready anymore.\n");
			break;
		}

		state = Controller.GetState(FLAG);

		Controller.dQ = (Controller.Q - Controller.Qold)/DELTAT_2R;

		Controller.Qsave.push_back(Controller.Q);

		Controller.dQsave.push_back(Controller.dQ);	

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
		/*
		if((!Controller.VelocitySafety(Controller.dQ)) || (!Controller.JointSafety(Controller.Q)))
		{	
			//EXITING THE CONTROL LOOP
			std::cout << "Breaking safety controllers for either Velocities and Joints position \n";
			break;
		}
		*/
		// POSITION CONTROL

		for(int y=0;y<NUMBER_OF_JOINTS;y++)
		{
			Controller.JointValuesInRad[y] = Q_ref(y);
		}
		
		Controller.FRI->SetCommandedJointPositions(Controller.JointValuesInRad);

		//TORQUE MEASUREMENTS

		Controller.MeasureJointTorques();	
		
		Controller.torque_measured(0) = Controller.MeasuredTorquesInNm[0];
		Controller.torque_measured(1) = Controller.MeasuredTorquesInNm[1];
		Controller.torque_measured(2) = Controller.MeasuredTorquesInNm[2];
		Controller.torque_measured(3) = Controller.MeasuredTorquesInNm[3];
		Controller.torque_measured(4) = Controller.MeasuredTorquesInNm[4];
		Controller.torque_measured(5) = Controller.MeasuredTorquesInNm[5];
		Controller.torque_measured(6) = Controller.MeasuredTorquesInNm[6];

		Controller.Tor_meas.push_back(Controller.torque_measured);

		torques_temp = Controller.Filter(Controller.Tor_meas,FILTER_LENGTH);

		Controller.Tor_meas_filtered.push_back(torques_temp);

		//REDUCED-ORDER OBSERVER DYNAMICS			 
		
		Controller.dz = Controller.SimReducedObserver(Controller.Q, Controller.dQ_hat, torques_temp);

		Controller.z = DELTAT_2R*Controller.dz + Controller.z;

		Controller.dQ_hat = Controller.z + Controller.k0*Controller.Q;

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

	file_error.close();

	file_control.close();

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
