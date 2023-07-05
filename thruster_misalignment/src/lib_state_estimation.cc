#include <lib_state_estimation.hpp>
using Belief = std::tuple<Eigen::VectorXd,Eigen::MatrixXd>;


KalmanFilter::KalmanFilter(const Eigen::MatrixXd& A_, 
                           const Eigen::MatrixXd& B_, 
                           const Eigen::MatrixXd& C_, 
                           const Eigen::MatrixXd& Q_, 
                           const Eigen::MatrixXd& R_)
{
    A = A_;
    B = B_;
    C = C_;
    Q = Q_;
    R = R_;
}

void KalmanFilter::init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& cov0) //state
{
    state = x0;
    cov = cov0;
}

Belief KalmanFilter::propagate(const Eigen::VectorXd& u)
{
    return std::make_tuple(A*state + B*u, A*cov*A.transpose() + Q);
}

Belief KalmanFilter::update(const Eigen::VectorXd& y, Belief predicted_belief)
{
    auto [xhat, covhat] = predicted_belief;
    int n = xhat.size();
    Eigen::MatrixXd K = covhat*C.transpose()*(C*covhat*C.transpose() + R).inverse();

    return std::make_tuple(xhat + K*(y-C*xhat), (Eigen::MatrixXd::Identity(n,n) - K*C)*covhat );
}

void KalmanFilter::estimate(const Eigen::VectorXd& u, const Eigen::VectorXd& y)
{
    Belief beliefhat = propagate(u);
    Belief belief = update(y,beliefhat);
    state = std::get<0>(belief);
    cov   = std::get<1>(belief);
}