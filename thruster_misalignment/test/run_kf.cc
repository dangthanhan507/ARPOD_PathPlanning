#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <lib_state_estimation.hpp>
#include <iostream>


#define ORBITAL_RATE 1

/**
 * run_kf.cc
 * 
 * Run the one-pass on the Kalman Filter to see if the calculations are correct.
*/
int main()
{
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6,3);
    Eigen::MatrixXd C = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(6,6);
    KalmanFilter kf(A,B,C,Q,R);

    Eigen::VectorXd x0   = Eigen::VectorXd::Ones(6);
    Eigen::MatrixXd cov0 = Eigen::MatrixXd::Identity(6,6);
    Eigen::VectorXd u    = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd y    = Eigen::VectorXd::Ones(6);
    std::cout << "x0: \n" << x0 << std::endl;
    std::cout << "cov0: \n" << cov0 << std::endl << std::endl;
    kf.init(x0,cov0);
    kf.estimate(u,y);
    auto [x,cov] = kf.getBelief();
    std::cout << "x: \n" << x << std::endl;
    std::cout << "cov: \n" << cov << std::endl;
    return 0;
}