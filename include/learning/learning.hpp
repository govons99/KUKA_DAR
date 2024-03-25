#ifndef LEARNING_HPP_
#define LEARNING_HPP_

#include<utils/lib.hpp>
#include <utils/data_utils.hpp>
#include <controller/kuka_utilities.h>

#include <limbo/kernel/exp.hpp>
#include <limbo/kernel/squared_exp_ard.hpp>
#include <limbo/mean/data.hpp>
#include<limbo/mean/constant.hpp>
#include <limbo/model/gp.hpp>
#include <limbo/model/sparsified_gp.hpp>
#include <limbo/model/gp/kernel_lf_opt.hpp>
#include <limbo/model/gp/mean_lf_opt.hpp>
#include <limbo/model/multi_gp.hpp>
#include <limbo/model/multi_gp/parallel_lf_opt.hpp>
#include <limbo/tools.hpp>
#include <limbo/tools/macros.hpp>
#include <limbo/serialize/text_archive.hpp>
#include <limbo/stop/max_iterations.hpp>


using namespace limbo;

class learning
{
    public:

    //GP parameters
    
    struct Params 
    {
        
        struct kernel_exp
        {
                BO_PARAM(double, sigma_sq, 1.0);
                BO_PARAM(double, l, 0.2);
        };
        
        struct kernel_maternfivehalves 
        {
                BO_PARAM(double, sigma_sq, 4.0);
                BO_PARAM(double, l, 2.1);
        };

        struct model_sparse_gp
        {
            //FOR 5 ms
            BO_PARAM(int, max_points,150);
            //FOR 10 ms
            //BO_PARAM(int, max_points,230);
        };

        struct kernel_squared_exp_ard
        {
            BO_PARAM(double,sigma_sq,1.0);
            BO_PARAM(int,k,0);
        };

        struct mean_constant
        {
            BO_PARAM(double,constant,0.0);
        };

        struct kernel : public defaults::kernel {};
        //struct kernel_squared_exp_ard : public defaults::kernel_squared_exp_ard {};
        struct opt_rprop : public defaults::opt_rprop {};    
    };
    
    using Kernel_t = kernel::SquaredExpARD<Params>;
    //using Kernel_t = kernel::MaternFiveHalves<Params>;
    //using Mean_t = mean::Data<Params>;
    using Mean_t = mean::Constant<Params>;
    //FOR SPARSE GP
    //using GP_t_sparse = model::SparsifiedGP<Params, Kernel_t, Mean_t, model::gp::KernelLFOpt<Params>>;
    using GP_t_sparse = model::SparsifiedGP<Params, Kernel_t, Mean_t, model::gp::KernelLFOpt<Params>>;
    //using GP_t = model::SparsifiedGP<Params, Kernel_t, Mean_t, model::gp::MeanLFOpt<Params>>;
    //FOR COMPLETE GP
    using GP_t = model::GP<Params, Kernel_t, Mean_t, model::gp::KernelLFOpt<Params>>;

    using MultiGP_t = model::MultiGP<Params, model::SparsifiedGP, Kernel_t, Mean_t, model::multi_gp::ParallelLFOpt<Params, model::gp::KernelLFOpt<Params>>>;

    learning()
    {
        
        //gp = new GP_t_sparse(DIMX,DIMY);

        gp = new MultiGP_t(DIMX,DIMY);

        //gp = new MultiGP_t(12,4);

        //gp = new GP_t(DIMX,DIMY);

        Eigen::MatrixXd temp;
        Eigen::MatrixXd temp2;
        Eigen::MatrixXd temp3;
        Eigen::MatrixXd temp4;
    
        A << 1.0, DELTAT, 0.0, 1.0 ;
        B << std::pow(DELTAT,2) / 2.0 , DELTAT;

        temp = A * B;
        temp2 = B * B.transpose();
        temp3 = temp * temp.transpose();
        W = temp2 + temp3;
        temp4 = -(B.transpose() + temp.transpose());
        Y = temp4 * W.inverse();

        Kuka_Vec Y_Max_Vector;
        Kuka_Vec Y_Min_Vector;

        Input_Vec  X_Max_Vector;
        Input_Vec X_Min_Vector;

        Y_Max_Vector << 10.0,10.0,10.0,10.0,10.0,10.0,10.0;

        Y_Min_Vector << -10.0,-10.0,-10.0,-10.0,-10.0,-10.0,-10.0;

        X_Max_Vector << M_PI,M_PI,M_PI,M_PI,M_PI,M_PI,M_PI,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0;

        X_Min_Vector << -M_PI,-M_PI,-M_PI,-M_PI,-M_PI,-M_PI,-M_PI,-5.0,-5.0,-5.0,-5.0,-5.0,-5.0,-5.0,-5.0,-5.0,-5.0,-5.0,-5.0,-5.0,-5.0;
        

        DeltaY_Vector = 2.0 * (Y_Max_Vector - Y_Min_Vector);

        DeltaX_Vector = 2.0 * (X_Max_Vector - X_Min_Vector);
        
    }

    ~learning()
    {
        delete this->gp;
    };

    void DatasetUpdate(Kuka_State State,Kuka_State OldState, Kuka_Vec reference, Kuka_Vec prediction, Kuka_Mat MassMatrix,std::vector<Kuka_Vec> d2Qvec,bool FLAG);
    Kuka_Vec DataPoint(Kuka_State State, Kuka_State OldState, Kuka_Vec reference, Kuka_Vec prediction, Kuka_Mat MassMatrix,std::vector<Kuka_Vec> d2Qvec,bool FLAG);

    void GpUpdate();
    
    Kuka_Vec GpPredict(Kuka_Vec Qquery, Kuka_Vec dQquery, Kuka_Vec d2Q_ref);
    
    data_manager X_manager;
    data_manager Y_manager;

    std::vector<Eigen::VectorXd> DatasetY;
    std::vector<Eigen::VectorXd> DatasetX;

    Kuka_Vec Y_Max_Vector;
    Kuka_Vec Y_Min_Vector;
    Input_Vec X_Max_Vector;
    Input_Vec X_Min_Vector;
    Kuka_Vec DeltaY_Vector;
    Input_Vec DeltaX_Vector;


    //GP_t_sparse *gp;
    //GP_t *gp;
    MultiGP_t *gp;

    void InitializeGp(std::string &XFile,std::string &YFile);
    
    void InitializeMultiGP(std::string &Dir);

    protected:

    Eigen::Matrix2d A;
    Eigen::Matrix<double, 2, 1>  B;
    Eigen::Matrix2d W;
    Eigen::Matrix<double, 1, 2> Y;
    
    Eigen::VectorXd NormalizeY(Eigen::VectorXd Y);
    Eigen::VectorXd DeNormalizeY(Eigen::VectorXd YNormalized);
    
    Eigen::VectorXd NormalizeX(Eigen::VectorXd X);
    Eigen::VectorXd DeNormalizeX(Eigen::VectorXd XNormalized);
    
    double UnwrapAngle(double angle_old, double angle_new); 
    double GramianCalc(double qnew, double dqnew, double qold, double dqold);
};

#endif /*LEARNING_HPP_*/