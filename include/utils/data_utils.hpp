#include<utils/lib.hpp>

class data_manager
{
    public:
    
    //Class constructors

    data_manager();

    ~data_manager();

    //Read Data for limbo  GP -> <Xtrain,Ytrain>

    bool read_data(std::string &filename, std::vector<Eigen::VectorXd> &container,int width);

    //Write Data for limbo GP -> <Prediction,Variance>

    void write_data(std::string &filename, std::vector<Eigen::VectorXd> &container);

    //Conversion from Limbo input to Eigen Matrix

    Eigen::MatrixXd Limbo_to_Eigen(std::vector<Eigen::VectorXd> &Dataset);

    //Conversion from Eigen Matrix to Limbo input
    std::vector<Eigen::VectorXd> Eigen_to_Limbo(Eigen::MatrixXd &Dataset);

    //Normalize Dataset

    void normalize_data(std::vector<Eigen::VectorXd> &Dataset);
    
    //DeNormalize Dataset

    void de_normalize_data(std::vector<Eigen::VectorXd> &Dataset);
        
    Eigen::VectorXd min_values;
    Eigen::VectorXd max_values;
    Eigen::VectorXd Delta;
};