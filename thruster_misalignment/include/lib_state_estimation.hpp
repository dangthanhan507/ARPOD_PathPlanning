#ifndef LIB_STATE_ESTIMATION_HPP
#define LIB_STATE_ESTIMATION_HPP
#include <Eigen/Dense>
#include <tuple>

using Belief = std::tuple<Eigen::VectorXd,Eigen::MatrixXd>;
class KalmanFilter
{
    public:
    /**
     * n = # of states
     * m = # of control inputs
     * p = # of sensor dimensions
     * 
     * @param A_: nxn matrix for propagating dynamics
     * @param B_: nxm matrix for control input propagating dynamics
     * @param C_: pxn matrix for converting state to sensor measurement
     * @param Q_: nxn covariance matrix representing how much to trust dynamics model
     * @param R_: pxp covariance matrix representing how much to trust sensor model
    */
    KalmanFilter(const Eigen::MatrixXd& A_, 
                 const Eigen::MatrixXd& B_, 
                 const Eigen::MatrixXd& C_, 
                 const Eigen::MatrixXd& Q_, 
                 const Eigen::MatrixXd& R_);

    /**
     * Everything in a Kalman Filter is represented by beliefs (estimate and our trust in it)
     * which is composed of the estimate and its covariance.
     * 
     * In order to start off the Kalman Filter, we initialize it off using an initial estimate
     * with an initial covariance on the estimate
     * 
     * @param x0: nx1 vector for initial state
     * @param cov0: nxn matrix covariance for the initial state
    */
    void init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& cov0);

    /**
     * Propagate Dynamics based on Kalman Filter predict step
     * x_k+1 = Ax_k + Bu_k
     * 
     * @param u: mx1 vector representing control input
    */
    Belief propagate(const Eigen::VectorXd& u);

    /**
     * Update step: correction rule in Kalman Filter using sensor measurement
     * using y = Cx in the hidden markov model for updating
     * 
     * NOTE: we need predicted state because it is required to perform the correction step
     * of the Kalman Filter
     * 
     * @param y: px1 vector representing measurement
     * @param predicted_belief: Belief tuple representing state and covariance
    */
    Belief update(const Eigen::VectorXd& y, Belief predicted_belief);

    /**
     * Running estimation through the update step
    */
    void estimate(const Eigen::VectorXd& u, const Eigen::VectorXd& y);

    /**
     * Return current best state from Kalman Filter
    */
    Eigen::VectorXd getEstimate()
    {
        return state;
    }

    /**
     * Return current belief of Kalman Filter (state and covariance)
    */
    Belief getBelief()
    {
        return std::make_tuple(state, cov);
    }

    private:
    //kalman filter matrices
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd C;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;

    //tracked belief (state and covariance)
    Eigen::VectorXd state;
    Eigen::MatrixXd cov;
};

#endif