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
	Controller.Q_hat(1) = 0;
	Controller.Q_hat(3) = 0;
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

	// vertical plane
	Eigen::Matrix<double, 4, 1> State_2R = {0, 0, 0, 0};

	// reduced-order observer 2R

	Eigen::Matrix<double, 2, 1> dvel_hat {0, 0};
	Eigen::Matrix<double, 2, 1> vel_hat {0, 0};
	Eigen::Matrix<double, 2, 1> dpos_hat {0, 0};
	Eigen::Matrix<double, 2, 1> pos_hat {0, 0};	

	// full-order observer 2R

	Eigen::Matrix<double, 2, 1> full_dvel_hat {0, 0};
	Eigen::Matrix<double, 2, 1> full_vel_hat {0, 0};
	Eigen::Matrix<double, 2, 1> full_dpos_hat {0, 0};
	Eigen::Matrix<double, 2, 1> full_pos_hat {0, 0};
	Eigen::Matrix<double, 2, 1> full_y_tilda;
	Eigen::Matrix<double, 2, 1> pos_2R {Controller.Q(1),Controller.Q(3)};

	// OBSERVER
	/*
	Eigen::Matrix<double, 2, 2> K0_pos;

	K0_pos(0,0)  =42.1;             
	K0_pos(0,1)  =0.07;

	K0_pos(1,0)  =0.09;
	K0_pos(1,1)  =42.2;

	Eigen::Matrix<double, 2, 2> K0_vel;
	
	K0_vel(0,0)  =7750;            
	K0_vel(0,1)  =23.6;
	
	K0_vel(1,0)  =25.9;
	K0_vel(1,1)  =7279;
	*/

	// tau = 1
	
	Eigen::Matrix<double, 2, 2> K0_pos;

	K0_pos(0,0)  =37.7;             
	K0_pos(0,1)  =-0.2;

	K0_pos(1,0)  =3.1;
	K0_pos(1,1)  =33.4;

	Eigen::Matrix<double, 2, 2> K0_vel;
	
	K0_vel(0,0)  =1177;            
	K0_vel(0,1)  =-54.3;
	
	K0_vel(1,0)  =-22.6;
	K0_vel(1,1)  =1113;
	
	// tau = 11
	/*
	Eigen::Matrix<double, 2, 2> K0_pos;

	K0_pos(0,0)  =40.5;             
	K0_pos(0,1)  =-0.4;

	K0_pos(1,0)  =2.5;
	K0_pos(1,1)  =37;

	Eigen::Matrix<double, 2, 2> K0_vel;
	
	K0_vel(0,0)  =1221.7;            
	K0_vel(0,1)  =-71.2;
	
	K0_vel(1,0)  =-27.2;
	K0_vel(1,1)  =1178;
	*/
	// tau = 21
	/*
	Eigen::Matrix<double, 2, 2> K0_pos;

	K0_pos(0,0)  =37.8;             
	K0_pos(0,1)  =0.2;

	K0_pos(1,0)  =2.2;
	K0_pos(1,1)  =41.6;

	Eigen::Matrix<double, 2, 2> K0_vel;
	
	K0_vel(0,0)  =1810.5;            
	K0_vel(0,1)  =-125.7;
	
	K0_vel(1,0)  =-96;
	K0_vel(1,1)  =1214.4;
	*/
	// tau = 
	/*
	Eigen::Matrix<double, 2, 2> K0_pos;

	K0_pos(0,0)  =939;             
	K0_pos(0,1)  =-5.4;

	K0_pos(1,0)  =-6.7;
	K0_pos(1,1)  =924.8;

	Eigen::Matrix<double, 2, 2> K0_vel;
	
	K0_vel(0,0)  =3497.6;            
	K0_vel(0,1)  =-8.3;
	
	K0_vel(1,0)  =-14.4;
	K0_vel(1,1)  =3456.6;
	*/

	Eigen::Matrix<double, 14, 1> state_output;

	Eigen::Matrix<double, 2, 1> error;

	Eigen::Matrix<double, 2, 1> vel_error;

	Eigen::Matrix<double, 2, 1> control {0.0, 0.0};

	double DELTAT_2R = 0.001;

	double Time_2R = 0.0;

	double th = 400;

	// file for the evolution of the internal model

	//std::string eta_save = "eta.txt";

	//std::ofstream file(eta_save);

	std::string error_save = "error.txt";

	std::ofstream file_error(error_save);

	std::string control_save = "control_DAR.txt";

	std::ofstream file_control(control_save);

	std::string qhat_save = "qhat_DAR.txt";

	std::ofstream file_qhat(qhat_save);	

	Eigen::Matrix<double, 2, 2> KD;
    KD(0,0) = 80;
    KD(0,1) = 0;
    KD(1,0) = 0;
    KD(1,1) = 80;

	int f = 4;

	double val = 0.0;
	double t_new = 0.0;

	double val2 = 0.0;
	double t_new2 = 0.0;

	//SIMULATION LOOP
	while ((float)CycleCounter * Controller.FRI->GetFRICycleTime() < RUN_TIME_IN_SECONDS)
	{		

		Time = Controller.FRI->GetFRICycleTime() * (float)CycleCounter;
		
		error(0) = Controller.Q(1)-pos_hat(0);
		error(1) = Controller.Q(3)-pos_hat(1);

		full_y_tilda(0) = Controller.Q(1)-full_pos_hat(0);
		full_y_tilda(1) = Controller.Q(3)-full_pos_hat(1);

		pos_2R(0) = Controller.Q(1);
		pos_2R(1) = Controller.Q(3);

		//std::cout << "error \n" << error << std::endl;

		//std::cout << "pos_hat \n" << pos_hat << std::endl;

		file_error << error.transpose();

		file_error << "\n";

		Time_2R += DELTAT_2R;

		//Q_ref = Q0 + Kuka_Vec::Constant(0.2*(1.0 - std::cos(f*Time_2R)));

		Q_ref = Q0; 

		/*if ( Time <= 50 )
		{
			Q_ref =  Q0 + Time_2R * Kuka_Vec::Constant(0.2);
			val = Time_2R;
		}

		if ( Time > 50 && Time <= 100 )	
		{
			std::cout << "here" << std::endl;
			Q_ref =  Q0 + Kuka_Vec::Constant(val*0.2) - t_new * Kuka_Vec::Constant(0.2);
			t_new = t_new + DELTAT_2R;
			val2 = Time_2R;
		}

		if ( Time > 100 && Time <= 150 )
		{
			Q_ref =  Q0 + Kuka_Vec::Constant(val*0.2) - Kuka_Vec::Constant(t_new*0.2) + t_new2 * Kuka_Vec::Constant(0.2);
			t_new2 = t_new2 + DELTAT_2R;
		}*/

		if ( Time <= 50 )
		{
			Q_ref =  Q0 + Kuka_Vec::Constant(1 - std::exp(-Time_2R) );
			val = Time_2R;
		}

		if ( Time > 50 && Time <= 100 )	
		{
			std::cout << "here" << std::endl;
			Q_ref =  Q0 + Kuka_Vec::Constant(1 - std::exp(-val)) - Kuka_Vec::Constant(1 - std::exp(-t_new) );
			t_new = t_new + DELTAT_2R;
			val2 = Time_2R;
		}

		if ( Time > 100 && Time <= 150 )
		{
			Q_ref =  Q0 + Kuka_Vec::Constant(1 - std::exp(-val)) - Kuka_Vec::Constant(1 - std::exp(-t_new) ) + Kuka_Vec::Constant(1 - std::exp(-t_new2) );
			t_new2 = t_new2 + DELTAT_2R;
		}

		Q_ref(0) = Q0(0);
		Q_ref(2) = Q0(2);
		Q_ref(4) = Q0(4);
		Q_ref(5) = Q0(5);
		Q_ref(6) = Q0(6);

		std::cout << "Q_ref \n" << Q_ref << std::endl;

		std::cout << "Q \n" << Controller.Q << std::endl;

		// 2R SIMULATION

		Mass_2R(0,0) = std::pow(2,l2)*m2+2*l1*l2*m2*cos(State_2R(1))+std::pow(2,l1)*(m1+m2);
		Mass_2R(0,1) = std::pow(2,l2)*m2+l1*l2*m2*cos(State_2R(1));
		Mass_2R(1,0) = std::pow(2,l2)*m2+l1*l2*m2*cos(State_2R(1));
		Mass_2R(1,1) = std::pow(2,l2)*m2;

		Coriolis_2R(0) = -m2*l1*l2*sin(State_2R(1))*std::pow(2,State_2R(3))-2*m2*l1*l2*sin(State_2R(1))*State_2R(2)*State_2R(3);
		Coriolis_2R(1) = m2*l1*l2*sin(State_2R(1))*std::pow(2,State_2R(2));

		Gravity_2R(0) = cos(State_2R(0)+State_2R(1))*m2*9.81*l2 + cos(State_2R(0))*(m1+m2)*l1*9.81;
		Gravity_2R(1) = cos(State_2R(0)+State_2R(1))*m2*9.81*l2;

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

		control(0) = torques_temp(1);
		control(1) = torques_temp(3);

		file_control << control.transpose();

		file_control << "\n";

		//REDUCED-ORDER OBSERVER DYNAMICS			 
		/*
		Controller.dz = Controller.SimReducedObserver(Controller.Q, Controller.dQ_hat, torques_temp);

		Controller.z = DELTAT_2R*Controller.dz + Controller.z;

		Controller.dQ_hat = Controller.z + Controller.k0*Controller.Q;

		Controller.dQ_hat_save.push_back(Controller.dQ_hat);

		Controller.Q_hat_save.push_back(Controller.Q_hat);
		*/

		//FULL-ORDER OBSERVER DYNAMICS
		
		//y_tilda = Controller.Q - Controller.Q_hat;

		//std::cout << "y_tilde \n" << full_y_tilda << std::endl;

		//d2Q_hat = Controller.SimObserver(Controller.Q, y_tilda, Controller.dQ_hat, torques_temp);
		full_dvel_hat = Controller.SimObserver2R(pos_2R, full_vel_hat, control, full_y_tilda);

		//Controller.dQ_hat = Controller.EulerIntegration(d2Q_hat, Controller.dQ_hat);
		full_vel_hat = DELTAT_2R * full_dvel_hat + full_vel_hat;

		full_dpos_hat = full_vel_hat + KD*full_y_tilda;		

		//Controller.Q_hat = Controller.EulerIntegration(dx1_hat, Controller.Q_hat);
		full_pos_hat = DELTAT_2R * full_dpos_hat + full_pos_hat;

		Controller.dQ_hat(1) = full_vel_hat(0);
		Controller.dQ_hat(3) = full_vel_hat(1);

		Controller.Q_hat(1) = full_pos_hat(0);
		Controller.Q_hat(3) = full_pos_hat(1);		

		Controller.dQ_hat_save.push_back(Controller.dQ_hat);
		Controller.Q_hat_save.push_back(Controller.Q_hat);

		//DAR OBSERVER

		dvel_hat = Mass_2R.inverse()*(control - Coriolis_2R - Gravity_2R) + K0_vel * error;

		vel_hat = DELTAT_2R * dvel_hat + vel_hat;

		dpos_hat = vel_hat + K0_pos * error;

		pos_hat = DELTAT_2R * dpos_hat + pos_hat; 

		// Saving the position of the 2R

		State_2R(0) = pos_hat(0);
		State_2R(1) = pos_hat(1);
		State_2R(2) = vel_hat(0);
		State_2R(3) = vel_hat(1);

		file_qhat << State_2R.transpose();

		file_qhat << "\n";

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

	//file.close();

	file_error.close();

	file_control.close();

	file_qhat.close();

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
