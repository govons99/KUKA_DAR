#ifndef KUKA_UTILITIES
#define KUKA_UTILITIES

#include<utils/lib.hpp>
#include<utils/data_utils.hpp>

Eigen::Vector3d D_kin(Kuka_Vec q);

Eigen::MatrixXd Jacobian(Kuka_Vec q);

Eigen::MatrixXd diff_Jacobian(Kuka_Vec q, Kuka_Vec dq);

Eigen::MatrixXd diff_Jacobian2(Kuka_Vec q, Kuka_Vec qdot);

Kuka_Vec Filter2(std::vector<Kuka_Vec> &signal, int filter_length);

Eigen::VectorXd Filter3(std::vector<Eigen::VectorXd> &signal, int filter_length);

void dampedPseudoInverse(const Eigen::MatrixXd& A,double dampingFactor,double e,Eigen::MatrixXd& Apinv,unsigned int computationOptions);


#endif // KUKA_UTILITIES
