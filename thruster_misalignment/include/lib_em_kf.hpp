#ifndef LIB_EM_KF_HPP
#define LIB_EM_KF_HPP

#include <Eigen/Dense>
#include <lib_kalman_filter.hpp>
#include <vector>
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

using Belief = std::tuple<Eigen::VectorXd,Eigen::MatrixXd>;
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

    /**
     * Performs EM Maximum Likelihood Estimation
     * 
     * this is done by performing the Estep (Expectation step)
     * to obtain the marginal distribution of states
     * 
     * then the Mstep (Maximization Step) is performed to get the 
     * best parameters.
     * 
    */
    void MLE();

    private:
    std::vector<Belief> beliefs;
    std::vector<Eigen::VectorXd> control_inputs;
}

#endif