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

	// vertical plane
	Eigen::Matrix<double, 4, 1> State_2R = {Controller.Q(1), Controller.Q(3), 0, 0};
	Eigen::Matrix<double, 2, 1> acc_2R {0, 0};
	Eigen::Matrix<double, 2, 1> vel_2R {0, 0};
	Eigen::Matrix<double, 2, 1> pos_2R {Controller.Q(1), Controller.Q(3)};

	// reduced-order observer 2R

	Eigen::Matrix<double, 2, 1> z_2R {0, 0};
	Eigen::Matrix<double, 2, 1> dz_2R {0, 0};
	Eigen::Matrix<double, 2, 1> vel_2R_hat {0, 0};

	//***************************************************************
	/*
	// IM2

	Eigen::Matrix<double, 14, 1> state_output;

	// Controller gain

	Eigen::Matrix<double, 2, 14> Theta;

	// Internal model variables
	Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(10,10);
	Eigen::MatrixXd Gamma = Eigen::MatrixXd::Zero(10,2);

	Eigen::MatrixXd eta = Eigen::MatrixXd::Zero(10,1);
	Eigen::MatrixXd eta_dot = Eigen::MatrixXd::Zero(10,1);

	// Anti-Windup gain

	Eigen::MatrixXd E1 = Eigen::MatrixXd::Zero(10,2);

	Phi.topRightCorner(8,8).setIdentity();
	Phi(8,2) = -4;
	Phi(9,3) = -4;
	Phi(8,6) = -5;
	Phi(9,7) = -5;
	
	Gamma(8,0) = 1;
	Gamma(9,1) = 1;
	*/
	//***************************************************************
	// IM3

	Eigen::Matrix<double, 18, 1> state_output;

	// Controller gain

	Eigen::Matrix<double, 2, 18> Theta;

	// Internal model variables
	Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(14,14);
	Eigen::MatrixXd Gamma = Eigen::MatrixXd::Zero(14,2);

	Eigen::MatrixXd eta = Eigen::MatrixXd::Zero(14,1);
	Eigen::MatrixXd eta_dot = Eigen::MatrixXd::Zero(14,1);

	// Anti-Windup gain

	Eigen::MatrixXd E1 = Eigen::MatrixXd::Zero(14,2);

	Phi.topRightCorner(12,12).setIdentity();
	Phi(12,2) = -36;
	Phi(13,3) = -36;
	Phi(12,6) = -49;
	Phi(13,7) = -49;
	Phi(12,10) = -14;
	Phi(13,11) = -14;
	
	Gamma(12,0) = 1;
	Gamma(13,1) = 1;

	//****************************************************************
	// Experiment: IM3 DC 0.5 OPT 1
	
	// initial condition
	eta(0) =0.0112695;
	eta(1) =-0.0136264;
	eta(2) =-0.0042940;
	eta(3) =0.0035940;
	eta(4) =-0.0139494;
	eta(5) =0.0090570;
	eta(6) =0.0043509;
	eta(7) =-0.0035833;
	eta(8) =0.0142034;
	eta(9) =-0.0087919;
	eta(10) =-0.0045638;
	eta(11) =0.0035188;
	eta(12) =-0.0152526;
	eta(13) =0.0076807;

	// deadzone gain
	E1(0,0)   =-3.07117012913785e-08;
	E1(1,0)   =-1.10977534312383e-07;
	E1(2,0)   =2.03165737192334e-07;
	E1(3,0)   =5.92124807631598e-07;
	E1(4,0)   =-5.97767689267814e-07;
	E1(5,0)   =-7.47822360365496e-07;
	E1(6,0)   =-4.17954055836461e-06;
	E1(7,0)   =-2.07851560523106e-05;
	E1(8,0)   =7.73781039907641e-05;
	E1(9,0)   =0.000238557464593412;
	E1(10,0)  =-0.000697496982120275;
	E1(11,0)  =-0.00133117539865004;
	E1(12,0)  =0.00305032814222743;
	E1(13,0)  =0.00198271609452005;

	E1(0,1)   =-2.49001723683347e-08;
	E1(1,1)   =-8.93269368237225e-08;
	E1(2,1)   =1.62315401575568e-07;
	E1(3,1)   =4.66951887756005e-07;
	E1(4,1)   =-4.85696011220656e-07;
	E1(5,1)   =-4.39797659425666e-07;
	E1(6,1)   =-2.53269716811440e-06;
	E1(7,1)   =-1.93063853505954e-05;
	E1(8,1)   =4.15609887979301e-05;
	E1(9,1)   =0.000235447497391396;
	E1(10,1)  =-0.000197003441160659;
	E1(11,1)  =-0.00175742713352849;
	E1(12,1)  =-0.000422141897082796;
	E1(13,1)  =0.00706390065674041;
	
	// controller
	Theta(0,0)  =-730.537908609972;             
	Theta(0,1)  =-96.3985195563127;
	Theta(0,2)  =-105.054025456879;
	Theta(0,3)  =-22.9701735063313;
	Theta(0,4)  =-13456.2674227882;
	Theta(0,5)  =-446.410805407793;
	Theta(0,6)  =-14550.9086231256;
	Theta(0,7)  =925.428146250165;
	Theta(0,8)  =-52586.3415591072;
	Theta(0,9)  =-2016.04691424656;
	Theta(0,10) =-13707.7590455501;
	Theta(0,11) =770.088966137371;
	Theta(0,12) =-23779.0372452071;
	Theta(0,13) =-1054.59693651009;
	Theta(0,14) =-2021.48310163929;
	Theta(0,15) =98.2458132671698;
	Theta(0,16) =-2237.91748491225;
	Theta(0,17) =-114.694548214391;

	Theta(1,0)  =154.900845774850;            
	Theta(1,1)  =-313.145378412470;
	Theta(1,2)  =-10.5380630452562;
	Theta(1,3)  =-52.5133704245347;
	Theta(1,4)  =5663.01976182674;
	Theta(1,5)  =-6124.97878372014;
	Theta(1,6)  =10768.4347271723;
	Theta(1,7)  =-6358.09744875562;
	Theta(1,8)  =21081.9327675526;
	Theta(1,9)  =-24066.0594602436;
	Theta(1,10) =9749.67591499281;
	Theta(1,11) =-6038.08950976521;
	Theta(1,12) =9002.76408969324;
	Theta(1,13) =-10939.9941353830;
	Theta(1,14) =1379.80789272892;
	Theta(1,15) =-897.038580581891;
	Theta(1,16) =791.759842992154;
	Theta(1,17) =-1034.93352705672;
	
	//****************************************************************
	// Experiment: IM3 DC 1 OPT 1
	/*
	// initial condition
	
	eta(0) =0.0038748;
	eta(1) =-0.0035130;
	eta(2) =-0.0028810;
	eta(3) =0.0019590;
	eta(4) =-0.0045030;
	eta(5) =0.0022584;
	eta(6) =0.0029135;
	eta(7) =-0.0019342;
	eta(8) =0.0045391;
	eta(9) =-0.0022021;
	eta(10) =-0.0030401;
	eta(11) =0.0018243;
	eta(12) =-0.0046865;
	eta(13) =0.0019762;

	// deadzone gain
	E1(0,0)   =-2.37691028422290e-08;
	E1(1,0)   =-8.71365097461446e-07;
	E1(2,0)   =1.22566267561731e-07;
	E1(3,0)   =2.93487136109015e-07;
	E1(4,0)   =-1.28052576748674e-07;
	E1(5,0)   =1.20287226903637e-07;
	E1(6,0)   =-5.45949044693381e-06;
	E1(7,0)   =-2.88814066811684e-05;
	E1(8,0)   =7.48062845055870e-05;
	E1(9,0)   =0.000258859147538952;
	E1(10,0)  =-0.000642068676366011;
	E1(11,0)  =-0.00135329765323184;
	E1(12,0)  =0.00296034499252018;
	E1(13,0)  =0.00218762876660164;

	E1(0,1)   =-1.91951235089916e-08;
	E1(1,1)   =-6.82388039376857e-08;
	E1(2,1)   =1.01316495810025e-07;
	E1(3,1)   =2.13571861285608e-07;
	E1(4,1)   =-1.82655820309160e-07;
	E1(5,1)   =1.22919961011648e-07;
	E1(6,1)   =-2.74682200253293e-06;
	E1(7,1)   =-2.67225643515179e-05;
	E1(8,1)   =3.30223941599906e-05;
	E1(9,1)   =0.000258511209002409;
	E1(10,1)  =-0.000135361778029207;
	E1(11,1)  =-0.00178264444400773;
	E1(12,1)  =-0.000517996315606013;
	E1(13,1)  =0.00708698967117157;
	
	// controller
	Theta(0,0)  =-1109.48110322116;             
	Theta(0,1)  =-184.644498049081;
	Theta(0,2)  =-127.116630437709;
	Theta(0,3)  =-28.7979282784834;
	Theta(0,4)  =-54560.7966128677;
	Theta(0,5)  =-2748.63487117307;
	Theta(0,6)  =-107422.129211521;
	Theta(0,7)  =-2736.53010766202;
	Theta(0,8)  =-184042.835168091;
	Theta(0,9)  =-11289.7012509779;
	Theta(0,10) =-89177.7458248478;
	Theta(0,11) =-2916.11248863281;
	Theta(0,12) =-70177.8389591124;
	Theta(0,13) =-5242.91090166614;
	Theta(0,14) =-11433.8659549600;
	Theta(0,15) =-465.527292906784;
	Theta(0,16) =-5418.23587957512;
	Theta(0,17) =-495.284554905836;

	Theta(1,0)  =133.576900573260;            
	Theta(1,1)  =-459.680271699622;
	Theta(1,2)  =-11.9928892190205;
	Theta(1,3)  =-57.0786372142003;
	Theta(1,4)  =22357.7097567098;
	Theta(1,5)  =-21470.1826004412;
	Theta(1,6)  =52624.2784802516;
	Theta(1,7)  =-41461.3071913569;
	Theta(1,8)  =68096.2746236059;
	Theta(1,9)  =-73449.5308520491;
	Theta(1,10) =41199.8356795316;
	Theta(1,11) =-34821.9257077973;
	Theta(1,12) =22674.0202634221;
	Theta(1,13) =-28430.4404044116;
	Theta(1,14) =4939.99297425557;
	Theta(1,15) =-4516.42400116240;
	Theta(1,16) =1441.78625424866;
	Theta(1,17) =-2231.53718043790;
	*/
	//****************************************************************
	// Experiment: IM3 DC 1.5 OPT 1
	/*
	// initial condition
	eta(0) =1.3433e-03;
	eta(1) =-8.9869e-04;
	eta(2) =-1.5608e-03;
	eta(3) =8.5089e-04;
	eta(4) =-1.5132e-03;
	eta(5) =5.1971e-04;
	eta(6) =1.5727e-03;
	eta(7) =-8.3920e-04;
	eta(8) =1.5127e-03;
	eta(9) =-5.1294e-04;
	eta(10) =-1.6190e-03;
	eta(11) =7.8923e-04;
	eta(12) =-1.5098e-03;
	eta(13) =4.9224e-04;

	// deadzone gain
	E1(0,0)   =-1.52994533784970e-08;
	E1(1,0)   =-5.31084299005385e-07;
	E1(2,0)   =5.77816152348609e-07;
	E1(3,0)   =5.08070111882346e-07;
	E1(4,0)   =1.85547578452520e-07;
	E1(5,0)   =2.38879880982027e-07;
	E1(6,0)   =-6.08506085002245e-06;
	E1(7,0)   =-3.24286175642920e-05;
	E1(8,0)   =7.05189641812802e-05;
	E1(9,0)   =0.000261970819062079;
	E1(10,0)  =-0.000587575500896955;
	E1(11,0)  =-0.00135130493537963;
	E1(12,0)  =0.00284261454439066;
	E1(13,0)  =0.00243189124037632;

	E1(0,1)   =-1.29679553552082e-08;
	E1(1,1)   =-4.14862460832039e-08;
	E1(2,1)   =5.72850523249549e-07;
	E1(3,1)   =2.86499256066332e-07;
	E1(4,1)   =-1.45701775123519e-07;
	E1(5,1)   =2.13308857795102e-07;
	E1(6,1)   =-2.55517189119465e-06;
	E1(7,1)   =-2.96962657488323e-05;
	E1(8,1)   =2.53740267285272e-05;
	E1(9,1)   =0.000264597276151446;
	E1(10,1)  =-8.67835532578278e-05;
	E1(11,1)  =-0.00178826250164154;
	E1(12,1)  =-0.000592863810849128;
	E1(13,1)  =0.00728233411611616;
	
	// controller
	Theta(0,0)  =-1565.17797943860;             
	Theta(0,1)  =-296.222877183565;
	Theta(0,2)  =-146.391105108117;
	Theta(0,3)  =-34.9063814910940;
	Theta(0,4)  =-188655.447465541;
	Theta(0,5)  =-13478.9084815713;
	Theta(0,6)  =-410714.605098248;
	Theta(0,7)  =-26683.3161304140;
	Theta(0,8)  =-529491.526633243;
	Theta(0,9)  =-45011.6471932601;
	Theta(0,10) =-290811.683568186;
	Theta(0,11) =-21852.4893745623;
	Theta(0,12) =-163212.337351629;
	Theta(0,13) =-16781.7876212737;
	Theta(0,14) =-31284.8509337034;
	Theta(0,15) =-2724.55600636398;
	Theta(0,16) =-9851.41930528530;
	Theta(0,17) =-1255.78239992057;

	Theta(1,0)  =97.9029031673016;            
	Theta(1,1)  =-619.038651413220;
	Theta(1,2)  =-14.9986761936763;
	Theta(1,3)  =-60.7492144876312;
	Theta(1,4)  =74010.7748371388;
	Theta(1,5)  =-67338.9435813524;
	Theta(1,6)  =168989.981294638;
	Theta(1,7)  =-145925.037546746;
	Theta(1,8)  =181645.039586143;
	Theta(1,9)  =-192745.100829654;
	Theta(1,10) =108562.922018516;
	Theta(1,11) =-105006.366868839;
	Theta(1,12) =45819.1900415768;
	Theta(1,13) =-60777.8631850696;
	Theta(1,14) =10321.9657222543;
	Theta(1,15) =-11491.5815305956;
	Theta(1,16) =1944.11184884502;
	Theta(1,17) =-3771.46253199136;
	*/

	//****************************************************************
	//****************************************************************
	//****************************************************************
	// Experiment: IM2 DC 0.5 OPT 1
	/*
	// initial condition
	eta(0) =0.072137;
	eta(1) =-0.107087;
	eta(2) =-0.047451;
	eta(3) =0.037497;
	eta(4) =-0.097161;
	eta(5) =0.066460;
	eta(6) =0.048058;
	eta(7) =-0.037164;
	eta(8) =0.098218;
	eta(9) =-0.065276;

	// deadzone gain
	E1(0,0)  =-1.21660775457707e-06;
	E1(1,0)  =-3.00135273713434e-06;
	E1(2,0)  =-1.08340705790812e-06;
	E1(3,0)  =-9.91194390801493e-06;
	E1(4,0)  =7.20432777515609e-05;
	E1(5,0)  =0.000219759031943288;
	E1(6,0)  =-0.000775053507169896;
	E1(7,0)  =-0.00153173153504514;
	E1(8,0)  =0.00394346157394864;
	E1(9,0)  =0.00418078924249270;

	E1(0,1)  =-9.59209513434264e-07;
	E1(1,1)  =-2.40735970744410e-06;
	E1(2,1)  =-3.65643179727754e-07;
	E1(3,1)  =-9.77009228403088e-06;
	E1(4,1)  =3.92270921764738e-05;
	E1(5,1)  =0.000219633493543783;
	E1(6,1)  =-0.000243169801102788;
	E1(7,1)  =-0.00197287495475169;
	E1(8,1)  =-2.26244704031670e-05;
	E1(9,1)  =0.00959240122236838;
	
	// controller
	Theta(0,0)  =-627.034499835345;             
	Theta(0,1)  =-75.3165157312916;
	Theta(0,2)  =-95.9640987304513;
	Theta(0,3)  =-21.1864515283374;
	Theta(0,4)  =-1421.04339835451;
	Theta(0,5)  =-50.6427425332388;
	Theta(0,6)  =-1629.36379353228;
	Theta(0,7)  =37.6346174001911;
	Theta(0,8)  =-4915.78950888466;
	Theta(0,9)  =-205.744824525830;
	Theta(0,10) =-1042.28042128432;
	Theta(0,11) =17.6413239771824;
	Theta(0,12) =-1486.00318186456;
	Theta(0,13) =-73.1742421220335;

	Theta(1,0)  =151.444074006939;            
	Theta(1,1)  =-268.385507015554;
	Theta(1,2)  =-10.1518638814824;
	Theta(1,3)  =-48.9907134524863;
	Theta(1,4)  =583.930167054671;
	Theta(1,5)  =-675.207377940040;
	Theta(1,6)  =980.212035039572;
	Theta(1,7)  =-764.125107953904;
	Theta(1,8)  =1910.30691802020;
	Theta(1,9)  =-2343.39081418225;
	Theta(1,10) =603.405456219990;
	Theta(1,11) =-490.604280087192;
	Theta(1,12) =539.164235793783;
	Theta(1,13) =-710.690733690758;
	*/
	//****************************************************************
	// Experiment: IM2 DC 1 OPT 1
	/*
	// initial condition
	eta(0) =0.022562;
	eta(1) =-0.028766;
	eta(2) =-0.027707;
	eta(3) =0.019811;
	eta(4) =-0.028711;
	eta(5) =0.017197;
	eta(6) =0.027972;
	eta(7) =-0.019585;
	eta(8) =0.028710;
	eta(9) =-0.017057;

	// deadzone gain
	E1(0,0)  =-5.61949701147751e-07;
	E1(1,0)  =-6.21140243657033e-07;
	E1(2,0)  =-3.17491686496784e-06;
	E1(3,0)  =-2.02305373142735e-05;
	E1(4,0)  =6.86719854298906e-05;
	E1(5,0)  =0.000241201212322361;
	E1(6,0)  =-0.000678072760944584;
	E1(7,0)  =-0.00149428264726879;
	E1(8,0)  =0.00353959248716208;
	E1(9,0)  =0.00410564107808878;

	E1(0,1)  =-4.84607298472791e-07;
	E1(1,1)  =-2.90127487064614e-07;
	E1(2,1)  =-1.44286620822502e-06;
	E1(3,1)  =-1.96413197546589e-05;
	E1(4,1)  =3.21708696424696e-05;
	E1(5,1)  =0.000244269027236204;
	E1(6,1)  =-0.000173112612451276;
	E1(7,1)  =-0.00191671997439768;
	E1(8,1)  =-0.000172203941182217;
	E1(9,1)  =0.00899166145377578;
	
	// controller
	Theta(0,0)  =-900.930341250381;             
	Theta(0,1)  =-136.168592378465;
	Theta(0,2)  =-116.940936646086;
	Theta(0,3)  =-26.1960565459993;
	Theta(0,4)  =-5578.42916915472;
	Theta(0,5)  =-251.309974959100;
	Theta(0,6)  =-10234.0754866234;
	Theta(0,7)  =-291.445486106908;
	Theta(0,8)  =-15360.6907534648;
	Theta(0,9)  =-886.181620462514;
	Theta(0,10) =-5434.65467728849;
	Theta(0,11) =-196.424554813025;
	Theta(0,12) =-3520.98800882137;
	Theta(0,13) =-264.000275615856;

	Theta(1,0)  =145.705791297269;            
	Theta(1,1)  =-379.546070473930;
	Theta(1,2)  =-11.6631378024203;
	Theta(1,3)  =-54.7894312214199;
	Theta(1,4)  =2236.31256960668;
	Theta(1,5)  =-2301.95595929501;
	Theta(1,6)  =4642.54280931954;
	Theta(1,7)  =-4176.20738428567;
	Theta(1,8)  =5476.12608965113;
	Theta(1,9)  =-6426.04981755301;
	Theta(1,10) =2311.46561697935;
	Theta(1,11) =-2240.47816750127;
	Theta(1,12) =1047.84610955022;
	Theta(1,13) =-1497.09316636331;
	*/
	//****************************************************************
	// Experiment: IM2 DC 1.5 OPT 1
	/*
	// initial condition
	eta(0) =0.0065425;
	eta(1) =-0.0076366;
	eta(2) =-0.0157004;
	eta(3) =0.0097249;
	eta(4) =-0.0084061;
	eta(5) =0.0037010;
	eta(6) =0.0157872;
	eta(7) =-0.0096283;
	eta(8) =0.0082785;
	eta(9) =-0.0037775;

	// deadzone gain
	E1(0,0)  =-1.84198293736724e-07;
	E1(1,0)  =8.23353296756826e-07;
	E1(2,0)  =-4.21261395246973e-06;
	E1(3,0)  =-2.56562765345511e-05;
	E1(4,0)  =6.53757033451929e-05;
	E1(5,0)  =0.000250923358781697;
	E1(6,0)  =-0.000614266903834493;
	E1(7,0)  =-0.00148645405349839;
	E1(8,0)  =0.00329976966428320;
	E1(9,0)  =0.00421687450519944;

	E1(0,1)  =-2.39642428371191e-07;
	E1(1,1)  =9.09208689965391e-07;
	E1(2,1)  =-1.71940538580841e-06;
	E1(3,1)  =-2.44867946401906e-05;
	E1(4,1)  =2.59894644150577e-05;
	E1(5,1)  =0.000255808369182526;
	E1(6,1)  =-0.000122475530177966;
	E1(7,1)  =-0.00190380283766300;
	E1(8,1)  =-0.000294561784677554;
	E1(9,1)  =0.00891334978865242;
	
	// controller
	Theta(0,0)  =-1212.93774125067;             
	Theta(0,1)  =-208.277951818819;
	Theta(0,2)  =-133.992719927290;
	Theta(0,3)  =-30.9806993031309;
	Theta(0,4)  =-17376.4302734371;
	Theta(0,5)  =-1068.72608941300;
	Theta(0,6)  =-33308.1872860329;
	Theta(0,7)  =-1919.70800573772;
	Theta(0,8)  =-36679.9362423997;
	Theta(0,9)  =-2855.18770726606;
	Theta(0,10) =-14047.6531609983;
	Theta(0,11) =-974.857618440535;
	Theta(0,12) =-6115.77868702016;
	Theta(0,13) =-632.371609855654;

	Theta(1,0)  =131.248774457637;            
	Theta(1,1)  =-492.057628772404;
	Theta(1,2)  =-13.3764805286971;
	Theta(1,3)  =-58.2054240715691;
	Theta(1,4)  =6738.73023382882;
	Theta(1,5)  =-6473.14169879752;
	Theta(1,6)  =13271.4481288642;
	Theta(1,7)  =-12387.8826273638;
	Theta(1,8)  =12114.4773365626;
	Theta(1,9)  =-13978.4625734226;
	Theta(1,10) =4998.00689130020;
	Theta(1,11) =-5319.45380722471;
	Theta(1,12) =1490.10288452913;
	Theta(1,13) =-2402.84232340513;
	*/	

	Eigen::Matrix<double, 2, 1> error;

	Eigen::Matrix<double, 2, 1> vel_error;

	Eigen::Matrix<double, 2, 1> control {0.0, 0.0};

	Eigen::Matrix<double, 2, 1> control_sat {0.0, 0.0};

	Eigen::Matrix<double, 2, 1> deadzone {0.0, 0.0};

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
		state_output(14) = eta(10);
		state_output(15) = eta(11);
		state_output(16) = eta(12);
		state_output(17) = eta(13);

		control = Theta * state_output;

		control_sat = control;
		
		if ( control(0) >= th )
		{
			control_sat(0) = th;
			std::cout << "saturating" << std::endl;
		}
		if ( control(0) <= -th )
		{
			control_sat(0) = -th;
			std::cout << "saturating" << std::endl;
		}
		if ( control(1) >= th )
		{
			control_sat(1) = th;
			std::cout << "saturating" << std::endl;
		}
		if ( control(1) <= -th )
		{
			control_sat(1) = -th;
			std::cout << "saturating" << std::endl;
		}

		deadzone = control - control_sat;
		
		file_control << control_sat.transpose();

		file_control << "\n";

		// 2R SIMULATION

		Mass_2R(0,0) = std::pow(2,l2)*m2+2*l1*l2*m2*cos(State_2R(1))+std::pow(2,l1)*(m1+m2);
		Mass_2R(0,1) = std::pow(2,l2)*m2+l1*l2*m2*cos(State_2R(1));
		Mass_2R(1,0) = std::pow(2,l2)*m2+l1*l2*m2*cos(State_2R(1));
		Mass_2R(1,1) = std::pow(2,l2)*m2;

		Coriolis_2R(0) = -m2*l1*l2*sin(State_2R(1))*std::pow(2,State_2R(3))-2*m2*l1*l2*sin(State_2R(1))*State_2R(2)*State_2R(3);
		Coriolis_2R(1) = m2*l1*l2*sin(State_2R(1))*std::pow(2,State_2R(2));

		Gravity_2R(0) = cos(State_2R(0)+State_2R(1))*m2*9.81*l2 + cos(State_2R(0))*(m1+m2)*l1*9.81;
		Gravity_2R(1) = cos(State_2R(0)+State_2R(1))*m2*9.81*l2;
		
		acc_2R = Mass_2R.inverse()*(control_sat - Coriolis_2R - Gravity_2R);

		vel_2R = vel_2R + DELTAT_2R * acc_2R;

		pos_2R = pos_2R + DELTAT_2R * vel_2R;

		State_2R(0) = pos_2R(0);
		State_2R(1) = pos_2R(1);
		State_2R(2) = vel_2R(0);
		State_2R(3) = vel_2R(1);

		// internal model
		
		eta_dot = Phi * eta + Gamma * error + E1 * deadzone;

		eta = eta + DELTAT_2R * eta_dot;

		file << eta.transpose();

		file << "\n";
		
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

		//Controller.z = Controller.EulerIntegration(Controller.dz, Controller.z);
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
