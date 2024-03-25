#ifndef CONTROLLER_KUKA_HPP_
#define CONTROLLER_KUKA_HPP_

#include <controller/controller.hpp>


class controller_kuka : public controller
{
    public:
        //Constructors
        
        controller_kuka(){};
        controller_kuka(std::string MODE,bool FLAG)
        {
	    int	            ResultValue=0;
            
            std::string IMP("impedence");
            std::string POS("position");
	    
            std::string Xdata = "myGP/samples.dat";
	    std::string Ydata = "myGP/observations.dat";            
            
            std::string Dir = "myGP/";

            dQ = Kuka_Vec::Constant(0.0);
	    dQold = Kuka_Vec::Constant(0.0);
            dQ_num = Kuka_Vec::Constant(0.0);
            d2Q = Kuka_Vec::Constant(0.0);
            d2Qold = Kuka_Vec::Constant(0.0);

            z = Kuka_Vec::Constant(0.0);
            dz = Kuka_Vec::Constant(0.0);

            r = Kuka_Vec::Constant(0.0);
            r_ob = Kuka_Vec::Constant(0.0);

            p0 = Kuka_Vec::Constant(0.0);
            p0_hat = Kuka_Vec::Constant(0.0);
            
            Kuka_Vec temp_zero = Kuka_Vec::Constant(0.0);
            
            //std::cout << "\n Initialization of the GP \n";
            
            //Regressor = new learning();
            
            //Regressor->InitializeGp(Xdata,Ydata);

            //Regressor->InitializeMultiGP(Dir);

            //dyn = new CLWR_Dynamic_Model_Lib();

            if(FLAG)
            {
                const float TimeOutValueInSeconds = 120.0;
                //FRI starting
                FRI			=	new FastResearchInterface("/home/kuka_linux/Desktop/Kuka_Controller/external/FRILibrary/etc/980039-FRI-Driver.init");	        
                std::cout << "\n Initialization of the FRI \n";
                //Choosing the controlling mode
                if(!MODE.compare(IMP))
                {
                        ResultValue = FRI->StartRobot(		FastResearchInterface::JOINT_IMPEDANCE_CONTROL, TimeOutValueInSeconds);
                        this->FRI->SetCommandedJointStiffness(CommandedStiffness);
                        this->FRI->SetCommandedJointDamping(CommandedDamping);
                        MeasureJointPositions();
                        this->FRI->SetCommandedJointPositions(JointValuesInRad);
                }
                if(!MODE.compare(POS))
                {
                        ResultValue = FRI->StartRobot(		FastResearchInterface::JOINT_POSITION_CONTROL, TimeOutValueInSeconds);
                }
                else
                {
                        std::cout<<"No allowed controlling mode";
                }
                
                if (ResultValue == EOK)
                {
                                fprintf(stdout, "Robot successfully started.\n");
                }
                else
                {
                        fprintf(stderr, "ERROR, could not start robot: %s\n", strerror(ResultValue));
                }

                fprintf(stdout, "Current system state:\n%s\n", FRI->GetCompleteRobotStateAndInformation());

                MeasureJointPositions();

                for(int i=0;i<NUMBER_OF_JOINTS;i++)
                {
                        Q(i) = JointValuesInRad[i];
                }
            }
            else
            {
                // Q(0) = 0.0;Q(1) = 1.571;Q(2) = 0.0;Q(3) = 1.571;Q(4) = 0.0;Q(5) = 1.571;Q(6) = 0.0;
                /* Q(0) = -1.1;
                Q(1) = M_PI/4;
                Q(2) = 0;
                Q(3) = 1.3*M_PI;
                Q(4) = -1;
                Q(5) = 0;
                Q(6) = 0;
                */

                // DAR simulation horizontal plane
                /*
                Q(0) = 0.4;
                Q(1) = M_PI/2;
                Q(2) = M_PI/2;
                Q(3) = 0.5;
                Q(4) = -1;
                Q(5) = 0;
                Q(6) = 0;
                */

                // DAR simulation vertical plane
                Q(0) = 0;
                Q(1) = 1.1040;
                Q(2) = 0;
                Q(3) = -0.7500;
                Q(4) = 0;
                Q(5) = 0;
                Q(6) = 0;
            }

            // initial momentum when we use the observed velocities
            //p0_hat = GetMass(Q)*dQ_hat;

            // initial momentum when we use the actual velocities
            //p0 = GetMass(Q)*dQ;

            z = -k0*Q;
            
            Qsave.push_back(Q);
            dQsave.push_back(dQ);
            d2Qsave.push_back(d2Q);
            dQ_num_save.push_back(dQ_num);
            r_save.push_back(r);
            r_obs_save.push_back(r_ob);
            Qsave_filtered.push_back(Q);
            dQsave_filtered.push_back(dQ);
            d2Qsave_filtered.push_back(d2Q);
            robot_state << Q, dQ;
            old_robot_state << Q, dQ;
            state_filtered << Q , dQ;
            old_state_filtered << Q, dQ;            
            std::cout << "I am here \n";
        };

        // reduced observer for the 2R

        Eigen::MatrixXd SimReducedObserver2R(Eigen::MatrixXd Q, Eigen::MatrixXd dQ_hat, Eigen::MatrixXd Torque);

        // sign function HOMEMADE

        double sign(double value);

        // switching function for SMC

        Eigen::MatrixXd switching(Eigen::MatrixXd s, Eigen::MatrixXd phi);

        //Set torques in impedence control mode

        void SetTorques(Kuka_Vec torques);

        //Set joints positions in position control

        void SetJointsPositions(Kuka_Vec positions);

        //Get measured joints positions

        void MeasureJointPositions();

        //Get measured joint torques

        void MeasureJointTorques();

        //Calculate complete state [Q,dQ]
        
        Kuka_State GetState(bool FLAG);

        //Get gravity vector of KUKA

        Kuka_Vec GetGravity();

        //Get gravity of the nominal model

        Kuka_Vec GetGravityFL(Kuka_Vec Q);

        //Get gravity of the nominal model

        Kuka_Vec GetGravityFLFake(Kuka_Vec Q);

        // Get friction of the nominal model

        Kuka_Vec GetFriction(Kuka_Vec Qnow, Kuka_Vec dQnow);

        // Get Mass Matrix nominal model
        Kuka_Mat GetMass(Kuka_Vec Qnow);

        // Get Mass Matrix fake model

        Kuka_Mat GetMassFake(Kuka_Vec Qnow);
        
        //Dynamic feedback linearization

        Kuka_Vec FeedbackLinearization(Kuka_Vec Qnow, Kuka_Vec dQnow, Kuka_Vec reference);

        //Dynamic feedback linearization fake model

        Kuka_Vec FeedbackLinearizationFake(Kuka_Vec Qnow, Kuka_Vec dQnow, Kuka_Vec reference);

        //PD Controller

        Kuka_Vec PDController(Kuka_Vec Qnow, Kuka_Vec dQnow, Kuka_Vec d2Qnow, Kuka_Vec Qd, Kuka_Vec dQd, Kuka_Vec d2Qd);

        //Adding of the torque bias for KUKA LWR-4
        Kuka_Vec SignalAdjuster(Kuka_Vec signal, double threshold);
        
        //Numerical differentiation for velocity calculation
        Kuka_Vec EulerDifferentiation(Kuka_Vec X, Kuka_Vec Xold);

        //Signal filter
        Kuka_Vec Filter(std::vector<Kuka_Vec> &signal, int filter_length);

        //Gear Differentiation
        Kuka_Vec GearDiff(std::vector<Kuka_Vec> &signal,int filter_length);

        //From Kuka_Vec to std::Vector<Eigen::VectorXd>
        void FromKukaToDyn(std::vector<Eigen::VectorXd>& IN, std::vector<Kuka_Vec>& OUT);
        
        //Safety Mechanisms

        bool JointSafety(Kuka_Vec Qnow);
        
        bool VelocitySafety(Kuka_Vec dQnow);

        bool TorqueSafety(Kuka_Vec Torque);

        //Simulations routine

        Kuka_Vec SimDynamicModel(Kuka_Vec Qnow,Kuka_Vec dQnow,Kuka_Vec Torque);

        Kuka_Vec SimDynamicModelFake(Kuka_Vec Qnow,Kuka_Vec dQnow,Kuka_Vec Torque);

        Kuka_Vec EulerIntegration(Kuka_Vec dX, Kuka_Vec X);

        //Dynamic of the reduced observer

        Kuka_Vec SimReducedObserver(Kuka_Vec Q, Kuka_Vec dQ_hat, Kuka_Vec Torque);

        //Evaluation of the residual

        Kuka_Vec Residual(Kuka_Vec Qnow, Kuka_Vec dQnow, Kuka_Vec Torque_nominal, Kuka_Vec r, int index, Kuka_Vec& SUM1, Kuka_Vec& SUM2, Kuka_Vec initial_momentum);

        //Generation of an external torque

        Kuka_Vec ExtTorque(Kuka_Vec Torque_nominal, int fault, Kuka_Vec Q, Eigen::Vector3d Force);

        //Direct Kinematics

        Eigen::Vector3d DirKin(Kuka_Vec Q);

        //Jacobian

        Eigen::MatrixXd Jac(Kuka_Vec Q);

        //Derivative of the Jacobian

        Eigen::MatrixXd diff_Jac(Kuka_Vec Q, Kuka_Vec dQ);

        //Damped Least Square Pseudoinverse

        void dls_pinv(const Eigen::MatrixXd& A,double dampingFactor,double e, Eigen::MatrixXd& Apinv);

        //Check if a collision has occurred

        std::array<int,7> collision(Kuka_Vec r,double Time);

        //Full state observer

        Kuka_Vec SimObserver(Kuka_Vec Y, Kuka_Vec y_tilda, Kuka_Vec dX1_hat, Kuka_Vec Torque);

        ~controller_kuka()
        {
                //delete this->FRI;
                //delete this->dyn;
                //delete this->Regressor;
                
        };

        //Attributes definition

        FastResearchInterface	*FRI;
        
	CLWR_Dynamic_Model_Lib *dyn;

       //learning *Regressor;

        Kuka_State robot_state;
        Kuka_State old_robot_state;
        
        Kuka_State state_filtered;
        Kuka_State old_state_filtered;
        
        Kuka_Vec Q;
	Kuka_Vec Qold;

	Kuka_Vec dQ;
	Kuka_Vec dQold;
        Kuka_Vec dQ_num;
        
        Kuka_Vec d2Q;
        Kuka_Vec d2Qold;

        Kuka_Vec z;
        Kuka_Vec dz;
        Kuka_Vec dQ_hat;
        Kuka_Vec r;
        Kuka_Vec r_ob;
        Kuka_Vec p0_hat;
        Kuka_Vec p0;

        Kuka_Vec Q_hat;
        
        Kuka_Vec torque_measured;
        Kuka_Vec torque_assigned;

        float	CommandedTorquesInNm		[NUMBER_OF_JOINTS],
              	CommandedStiffness      	[NUMBER_OF_JOINTS]={0.0},
	        CommandedDamping       		[NUMBER_OF_JOINTS]={0.0},
                CommandedJointPositions  	[NUMBER_OF_JOINTS],
	        MeasuredTorquesInNm		[NUMBER_OF_JOINTS],
	        JointValuesInRad		[NUMBER_OF_JOINTS],
                GravityVector                   [NUMBER_OF_JOINTS];
        float   MassMatrix                    [NUMBER_OF_JOINTS][NUMBER_OF_JOINTS];
        
        Kuka_Vec integralsum = Kuka_Vec::Constant(0.0);

protected:
        //Conversion from Eigen vector to array of float
        void EigToArray(Kuka_Vec IN,float *OUT);

        //Conversion from array of float to Eigen Vector
        void ArrayToEig(float *IN, Kuka_Vec& OUT);
};
#endif /* CONTROLLER_KUKA_HPP_ */
