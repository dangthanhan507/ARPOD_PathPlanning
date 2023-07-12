#include <lib_em_kf.hpp>



ExpectationMaximization::ExpectationMaximization(const Eigen::MatrixXd& A_, 
                                                 const Eigen::MatrixXd& B_, 
                                                 const Eigen::MatrixXd& C_, 
                                                 const Eigen::MatrixXd& Q_, 
                                                 const Eigen::MatrixXd& R_)
: KalmanFilter(A_,B_,C_,Q_,R_)
{

}