#ifndef LIB_EM_KF_HPP
#define LIB_EM_KF_HPP

#include <Eigen/Dense>
#include <lib_kalman_filter.hpp>
/** lib_em_kf.hpp
 * SUMMARY:
 * Experimental Research Software for the Expectation Maximization Code.
 * The EM software is mean to estimate the Kalman Filter parameters 
 * probabilistically by finding the Max a Posteriori.
 * 
 * NOTE:
 * If this doesn't work, create plots for benchmarking this then discuss 
 * with Copp on next steps. 
 * 
 * 
 * 
 * MORENOTES:
 * it might be possible to use MPC_MHE
*/

class ExpectationMaximization: public KalmanFilter
{
    public:

    /**
     * @param
    */
    ExpectationMaximization(const Eigen::MatrixXd& A_, 
                            const Eigen::MatrixXd& B_, 
                            const Eigen::MatrixXd& C_, 
                            const Eigen::MatrixXd& Q_, 
                            const Eigen::MatrixXd& R_);


    /**
     * NOTE: Implement these algorithms
    */
    void Estep();
    void Mstep();
    void MAP();

    private:
}

#endif