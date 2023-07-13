#include <lib_em_kf.hpp>

using Belief = std::tuple<Eigen::VectorXd,Eigen::MatrixXd>;

ExpectationMaximization::ExpectationMaximization(const Eigen::MatrixXd& A_, 
                                                 const Eigen::MatrixXd& B_, 
                                                 const Eigen::MatrixXd& C_, 
                                                 const Eigen::MatrixXd& Q_, 
                                                 const Eigen::MatrixXd& R_
                                                 const size_t time_freq)
: KalmanFilter(A_,B_,C_,Q_,R_)
{
    //set time_freq to be the length of data before EM is performed

}