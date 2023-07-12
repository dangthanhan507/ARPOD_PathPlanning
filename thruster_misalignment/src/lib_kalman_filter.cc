#include <lib_kalman_filter.hpp>
using Belief = std::tuple<Eigen::VectorXd,Eigen::MatrixXd>;
#include <iostream>

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


void KalmanFilter::kalmanSmooth(std::vector<Belief>& beliefs, const std::vector<Eigen::VectorXd>& us)
{
    //to start off RTS algorithm, we need to have a list of prediction covariances

    std::vector<Eigen::MatrixXd> pred_covs; //save up predicted covs from i = 0 -> n
    std::vector<Eigen::MatrixXd> pred_mus;
    Eigen::MatrixXd pred_cov, pred_mu;
    for (int i = 0; i < beliefs.size()-1; ++i)
    {
        auto [mu, cov] = beliefs.at(i);
        pred_mu = A*mu + B*us.at(i);
        pred_cov = A*cov*A.transpose() + Q;
        pred_mus.push_back(pred_mu);
        pred_covs.push_back(pred_cov);
    }

    //backward pass :) but skip the last state
    Eigen::MatrixXd Ck;
    Eigen::MatrixXd muhat;
    Eigen::MatrixXd covhat;
    for (int i = beliefs.size()-2; i >= 0; --i)
    {
        auto [mu_i, cov_i] = beliefs.at(i);
        auto [mu_i1, cov_i1] = beliefs.at(i+1);

        Ck = cov_i * A.transpose() * pred_covs.at(i).inverse();
        muhat = mu_i + Ck*( mu_i1 - pred_mus.at(i) );
        covhat = cov_i + Ck*(cov_i1 - pred_covs.at(i))*Ck.transpose();

        //save
        beliefs.at(i) = std::make_tuple(muhat,covhat);
    }
    //finish
}