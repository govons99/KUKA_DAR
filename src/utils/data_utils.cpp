#include <utils/data_utils.hpp>

using namespace std;
using namespace Eigen;
using namespace boost;


//Class constructors

data_manager::data_manager(){};

//Class destructors

data_manager::~data_manager(){};

//Read Data for limbo  GP -> <Xtrain,Ytrain>
//DATA FORMAT FOR READING MUST BE IN THE FORM X(1,1) X(1,2) X(1,3)
//                                            X(2,1) X(2,2) X(2,3) etc...
bool data_manager::read_data(std::string &filename, std::vector<Eigen::VectorXd> &container,int width)
{   
        std::ifstream Datareader;
        std::string buffer;
        std::vector <string> buffer2;
        Eigen::VectorXd temp(width);
        int j;

        Datareader.open (filename, std::fstream::in);
        
        if (!Datareader.is_open())
        {
            std::cout << "Error in opening the file for reading" << "\n";
            return false;
        }

        while(getline(Datareader,buffer))
        {
            j=0;   
            boost::split(buffer2,buffer,is_any_of("\t"));
            for(auto it = buffer2.begin(); it != buffer2.end(); it++,j++)
            {   
                temp(j) = std::stof(*it);
            }

            container.push_back(temp);
        }  
        Datareader.close();
        return true;
};

//Write Data for limbo GP -> <Prediction,Variance>
void data_manager::write_data(std::string &filename, std::vector<Eigen::VectorXd> &container)
{
    std::ofstream Datawriter;

    Datawriter.open (filename, std::fstream::out);
    
    //For row vector
    int length = container[0].rows();

    if (!Datawriter.is_open())
    {
        std::cout << "Error in opening the file for writing" << "\n";
    }
    
    for(auto it=container.begin(); it != container.end();it++)
    {
        for(int i=0;i < length; i++)
        {
            Datawriter << (*it)(i,0) << "\t"; 
        }
        Datawriter<<"\n";
    }
    Datawriter.close();
};

//Conversion from Limbo input to Eigen Matrix

Eigen::MatrixXd data_manager::Limbo_to_Eigen(std::vector<Eigen::VectorXd> &Dataset)
{
    int rows = Dataset.size();
    
    int cols = Dataset[0].size();

    Eigen::MatrixXd Output_Matrix(rows,cols);
    
    for(int i=0;i<rows;i++)
    {   
        Output_Matrix.row(i) = Dataset[i];
    }
    return Output_Matrix;
};

//Conversion from Eigen Matrix to Limbo input
std::vector<Eigen::VectorXd> data_manager::Eigen_to_Limbo(Eigen::MatrixXd &Dataset)
{
    //For stack of row vector
    int rows = Dataset.rows();
    int cols = Dataset.cols();

    //DEBUGGARE I NAN 
    std::vector<Eigen::VectorXd> Output_vector;
    
    for(auto i=0;i<rows;i++)
    {
        Output_vector.push_back(Dataset.row(i));
    }
    return Output_vector;
};

//Normalize Dataset

void data_manager::normalize_data(std::vector<Eigen::VectorXd> &Dataset)
{
    int i;

    Eigen::MatrixXd NormalizedDataMatrix = Limbo_to_Eigen(Dataset);
    int rows = NormalizedDataMatrix.rows();
    int cols = NormalizedDataMatrix.cols();

    //Maximum value for each column
    max_values = NormalizedDataMatrix.colwise().maxCoeff();

    //Minimum value for each column
    min_values = NormalizedDataMatrix.colwise().minCoeff();

    //Maximum values - Minimum values for each column
    Delta = (max_values.array() - min_values.array()).matrix();

    auto temp = Eigen::VectorXd::Constant(Delta.rows(),0.0);

    //Normalization works if Delta has no zero components
    for(i=0;i<cols;i++)
    {
        if(Delta(i) == 0)
        {
            Dataset = Eigen_to_Limbo(NormalizedDataMatrix);
            return;
        }
    }
    for(i=0;i<rows;i++)
    {
            NormalizedDataMatrix.row(i) = ((NormalizedDataMatrix.row(i).array() - min_values.transpose().array()) * Delta.transpose().array().inverse()).matrix(); 
    }
    Dataset = Eigen_to_Limbo(NormalizedDataMatrix);
    return;
};

void data_manager::de_normalize_data(std::vector<Eigen::VectorXd> &Dataset)
{   
    Eigen::MatrixXd DeNormalizedDataMatrix = Limbo_to_Eigen(Dataset);
    int i;
    int rows = DeNormalizedDataMatrix.rows();
    int cols = DeNormalizedDataMatrix.cols();
    auto temp = Eigen::VectorXd::Constant(Delta.rows(),0.0);

    //De-Normalization works if Delta has no zero components
    for(i=0;i<cols;i++)
    {
        if(Delta(i) == 0)
        {
            Dataset = Eigen_to_Limbo(DeNormalizedDataMatrix);
            return;
        }
    }
    for(i=0;i<rows;i++)
    {
            DeNormalizedDataMatrix.row(i) = (((DeNormalizedDataMatrix.row(i).array()) * Delta.transpose().array()) + min_values.transpose().array()).matrix(); 
    }
    Dataset = Eigen_to_Limbo(DeNormalizedDataMatrix);
    return;
};