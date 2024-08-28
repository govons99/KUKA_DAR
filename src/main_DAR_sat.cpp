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

	//double m1 = 2.6;
	//double m2 = 2.6;
	double m1 = 2.6/4;
	double m2 = 2.6/4;
	double l1 = 0.5;
	double l2 = 0.5;

	Eigen::Matrix<double, 2, 2> Mass_2R;
	Eigen::Matrix<double, 2, 1> Coriolis_2R;
	Eigen::Matrix<double, 2, 1> Gravity_2R;

	// vertical plane
	Eigen::Matrix<double, 4, 1> State_2R = {Controller.Q(1), Controller.Q(3), 0, 0};
	Eigen::Matrix<double, 2, 1> acc_2R {0, 0};
	Eigen::Matrix<double, 2, 1> vel_2R {0, 0};
	Eigen::Matrix<double, 2, 1> pos_2R {Controller.Q(1), Controller.Q(3)};

	// reduced-order observer 2R

	Eigen::Matrix<double, 2, 1> z_2R {0, 0};
	Eigen::Matrix<double, 2, 1> dz_2R {0, 0};
	Eigen::Matrix<double, 2, 1> vel_2R_hat {0, 0};

	// Variables for internal model

	Eigen::MatrixXd eta = Eigen::MatrixXd::Zero(10,1);

	//Experiment 28/08

	// right mass
	//eta(0) =-2.5894e-04;
	//eta(1) =9.3404e-05;
	//eta(2) =-3.8998e-04;
	//eta(3) =9.7250e-05;
	//eta(4) =2.4883e-04;
	//eta(5) =-1.1535e-04;
	//eta(6) =3.8858e-04;
	//eta(7) =-9.8096e-05;
	//eta(8) =-2.5022e-04;
	//eta(9) =1.1404e-04;

	// wrong mass
	eta(0) =-2.5101e-04;
	eta(1) =1.0976e-04;
	eta(2) =-3.9097e-04;
	eta(3) =9.6274e-05;
	eta(4) =2.4863e-04;
	eta(5) =-1.1529e-04;
	eta(6) =3.9052e-04;
	eta(7) =-9.6444e-05;
	eta(8) =-2.4905e-04;
	eta(9) =1.1485e-04;

	//Experiment 21/03
	
	//eta(0) =0.0325;
	//eta(1) =0.0291;
	//eta(2) =-0.0108;
	//eta(3) =0.0123;
	//eta(4) =-0.0245;
	//eta(5) =0.0373;
	//eta(6) =0.0108;
	//eta(7) =-0.0124;
	//eta(8) =0.0245;
	//eta(9) =-0.0374;
	

	// right initialization
	// 0.016620  -0.104218  -0.010833   0.012397  -0.024513   0.037755   0.010874  -0.012395   0.024530  -0.037588
	/*eta(0) =0.016620;
	eta(1) =-0.104218;
	eta(2) =-0.010833;
	eta(3) =0.012397;
	eta(4) =-0.024513;
	eta(5) =0.037755;
	eta(6) =0.010874;
	eta(7) =-0.012395;
	eta(8) =0.024530;
	eta(9) =-0.037588;*/

	/*
	//GOOD VALUES
	eta(0) =0.1024;
	eta(1) =-0.0965;
	eta(2) =-0.0428;
	eta(3) =0.0505;
	eta(4) =-0.0965;
	eta(5) =0.1550;
	eta(6) =0.0424;
	eta(7) =-0.0514;
	eta(8) =0.0959;
	eta(9) =-0.1579;
	*/
	/*
	eta(0) =0.45;
	eta(1) =-0.45;
	eta(2) =-0.1;
	eta(3) =0.0505;
	eta(4) =-0.1;
	eta(5) =0.16;
	eta(6) =0.05;
	eta(7) =-0.06;
	eta(8) =0.1;
	eta(9) =-0.17;
	*/
	Eigen::MatrixXd eta_dot = Eigen::MatrixXd::Zero(10,1);
	
	// Internal model 
	
	Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(10,10);

	Phi.topRightCorner(8,8).setIdentity();
	Phi(8,2) = -4;
	Phi(9,3) = -4;
	Phi(8,6) = -5;
	Phi(9,7) = -5;

	Eigen::MatrixXd Gamma = Eigen::MatrixXd::Zero(10,2);
	Gamma(8,0) = 1;
	Gamma(9,1) = 1;	

	// SATURATION

	Eigen::Matrix<double, 2, 14> Theta;

	// Experiment 28/08

	Theta(0,0)  =-4368.44425858881;
	Theta(0,1)  =-1052.83393209503;
	Theta(0,2)  =-222.990287617679;
	Theta(0,3)  =-61.2217198164157;
	Theta(0,4)  =-2744291.21375116;
	Theta(0,5)  =-367104.827015311;
	Theta(0,6)  =-2924820.78180918;
	Theta(0,7)  =-423272.422283850;
	Theta(0,8)  =-1342532.92441138;
	Theta(0,9)  =-214562.726990049;
	Theta(0,10) =-315546.707575481;
	Theta(0,11) =-55154.2920630574;
	Theta(0,12) =-48931.2110478814;
	Theta(0,13) =-9945.49591103626;

	Theta(1,0)  =-310.222962127482;
	Theta(1,1)  =-1644.31586880527;
	Theta(1,2)  =-36.1304372213891;
	Theta(1,3)  =-84.9460973169524;
	Theta(1,4)  =776329.080750028;
	Theta(1,5)  =-944793.682054709;
	Theta(1,6)  =704552.626865971;
	Theta(1,7)  =-1018001.71006969;
	Theta(1,8)  =249102.423616984;
	Theta(1,9)  =-474164.936494248;
	Theta(1,10) =41555.8216950930;
	Theta(1,11) =-113038.774244754;
	Theta(1,12) =1752.99093220073;
	Theta(1,13) =-17985.0885714010;

	// Experiment 21/03
	//Theta(0,0)  =-910.794862173719;             
	//Theta(0,1)  =-157.513502353182;
	//Theta(0,2)  =-101.331624799208;
	//Theta(0,3)  =-28.4219383780070;
	//Theta(0,4)  =-2345.81527449224;
	//Theta(0,5)  =-296.158869720401;
	//Theta(0,6)  =-2939.74931800422;
	//Theta(0,7)  =-278.001775420082;
	//Theta(0,8)  =-7998.47077734866;
	//Theta(0,9)  =-1038.56100498504;
	//Theta(0,10) =-1854.28229123369;
	//Theta(0,11) =-180.702273975907;
	//Theta(0,12) =-2379.91685532850;
	//Theta(0,13) =-319.508335993080;

	//Theta(1,0)  =490.466135969861;            
	//Theta(1,1)  =-169.058253104426;
	//Theta(1,2)  =5.41420757937376;
	//Theta(1,3)  =-25.6913071211365;
	//Theta(1,4)  =1178.88644624612;
	//Theta(1,5)  =-332.182925798906;
	//Theta(1,6)  =1726.12714863822;
	//Theta(1,7)  =-337.331129423342;
	//Theta(1,8)  =3951.71395723236;
	//Theta(1,9)  =-1163.69877850374;
	//Theta(1,10) =1077.20342791144;
	//Theta(1,11) =-218.993485392001;
	//Theta(1,12) =1149.86636530088;
	//Theta(1,13) =-356.883994908081;

	/*
	Theta(0,0)  =-1779.83495799076;             
	Theta(0,1)  =-345.443499869201;
	Theta(0,2)  =-199.566856245319;
	Theta(0,3)  =-52.1114520784276;
	Theta(0,4)  =-15400.0807669671;
	Theta(0,5)  =-1613.80610564311;
	Theta(0,6)  =-29330.8887730089;
	Theta(0,7)  =-2791.66236173458;
	Theta(0,8)  =-40752.9295186830;
	Theta(0,9)  =-4611.71083121857;
	Theta(0,10) =-15171.0876176395;
	Theta(0,11) =-1515.99488971842;
	Theta(0,12) =-8865.77255375059;
	Theta(0,13) =-1111.31884047448;

	Theta(1,0)  =848.967516233152;            
	Theta(1,1)  =-351.484080200557;
	Theta(1,2)  =2.90770994300065;
	Theta(1,3)  =-49.5752672838276;
	Theta(1,4)  =8363.05478296632;
	Theta(1,5)  =-2047.82849181944;
	Theta(1,6)  =16595.6492919313;
	Theta(1,7)  =-3646.88501468163;
	Theta(1,8)  =21436.4829310435;
	Theta(1,9)  =-5754.25551449206;
	Theta(1,10) =8452.54972153227;
	Theta(1,11) =-1960.23099311435;
	Theta(1,12) =4428.74681544501;
	Theta(1,13) =-1356.17392135576;
	*/

	Eigen::Matrix<double, 14, 1> state_output;

	Eigen::Matrix<double, 2, 1> error;

	Eigen::Matrix<double, 2, 1> vel_error;

	Eigen::Matrix<double, 2, 1> control {0.0, 0.0};

	double DELTAT_2R = 0.0005;

	double Time_2R = 0.0;

	double th = 400;

	// file for the evolution of the internal model

	std::string eta_save = "eta.txt";

	std::ofstream file(eta_save);

	std::string error_save = "error.txt";

	std::ofstream file_error(error_save);

	std::string control_save = "control_DAR.txt";

	std::ofstream file_control(control_save);
	
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

		state_output(0) = Controller.Q(1);
		state_output(1) = Controller.Q(3);
		state_output(2) = dQ_filtered(1);
		state_output(3) = dQ_filtered(3);
		//state_output(0) = State_2R(0);
		//state_output(1) = State_2R(1);
		//state_output(2) = State_2R(2);
		//state_output(3) = State_2R(3);
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

		control = Theta * state_output;
		
		if ( control(0) >= th )
		{
			control(0) = th;
		}
		if ( control(0) <= -th )
		{
			control(0) = -th;
		}
		if ( control(1) >= th )
		{
			control(1) = th;
		}
		if ( control(1) <= -th )
		{
			control(1) = -th;
		}
		
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
		/*
		Controller.Qold = Controller.Q;

		Controller.dQold = Controller.dQ;

		Controller.dQ = Controller.EulerIntegration(Controller.d2Q,Controller.dQ);

		Controller.Q = Controller.EulerIntegration(Controller.dQ,Controller.Q);

		Controller.dQ_num = Controller.EulerDifferentiation(Controller.Q,Controller.Qold);
		
		Controller.GetState(FLAG); 
		*/

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

		//Controller.z = Controller.EulerIntegration(Controller.dz, Controller.z);
		Controller.z = DELTAT_2R*Controller.dz + Controller.z;

		Controller.dQ_hat = Controller.z + Controller.k0*Controller.Q;

		//noisy_torque_vec.push_back(Torques_measured);

		//Controller.Tor_th.push_back(Torques_ref);

		//dz_2R = Controller.SimReducedObserver2R(pos_2R, vel_2R_hat, control);

		//z_2R = z_2R + DELTAT_2R*dz_2R;

		//vel_2R_hat = z_2R + Controller.k0*pos_2R;

		// Saving estimated quantities

		//Controller.dQ_hat(1) = vel_2R_hat(0);
		//Controller.dQ_hat(3) = vel_2R_hat(1);

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
