#include <iostream>
#include <lib_dynamics.hpp>
#include <lib_kalman_filter.hpp>
#include <random>
#include <vector>

#define ORBITAL_RATE 1
#define DT 1

using namespace drake;
using Belief = std::tuple<Eigen::VectorXd,Eigen::MatrixXd>;

int mainNoNoise()
{
    //eigen matrices
    auto [A,B] = createZOHHCW(ORBITAL_RATE, DT);
    Eigen::MatrixXd C = Eigen::MatrixXd::Identity(6,6);
    
    HCW_Dynamics space_sim(ORBITAL_RATE);
    systems::Simulator<double> simulator(space_sim);
    systems::ContinuousState<double>& state = simulator.get_mutable_context().get_mutable_continuous_state();

    //initialize state
    state[0] = 10;
    state[1] = 10;
    state[2] = 10;
    state[3] = 1;
    state[4] = 1;
    state[5] = 1;

    KalmanFilter state_estimator(A,B,C,Eigen::MatrixXd::Identity(6,6),Eigen::MatrixXd::Identity(6,6));
    state_estimator.init(state.get_vector().CopyToVector(), Eigen::MatrixXd::Identity(6,6));

    VectorX<double> u(3);
    u << 0,0,0;

    
    std::vector<Belief> beliefs;
    std::vector<Eigen::VectorXd> control_inputs;
    std::vector<Eigen::VectorXd> real_values;

    beliefs.push_back(state_estimator.getBelief());
    real_values.push_back(state_estimator.getEstimate());
    for (int t = 0; t < 5; ++t)
    {
        u << t*2, t*2, t*2;

        space_sim.get_input_port(0).FixValue(&simulator.get_mutable_context(), u);
        simulator.AdvanceTo((t+1)*DT);

        auto meas = state.get_vector().CopyToVector();
        state_estimator.estimate(u,meas);


        beliefs.push_back(state_estimator.getBelief());
        control_inputs.push_back(u);

        real_values.push_back(state.get_vector().CopyToVector());
    }
    std::cout << "\n\n";

    state_estimator.kalmanSmooth(beliefs,control_inputs);

    std::cout << "Smoothed Error:\n";
    for (int t = 0; t < 5; ++t)
    {
        std::cout << "t="<<t<<":\n";
        auto [mu, cov] = beliefs.at(t);
        std::cout << mu - real_values.at(t) << std::endl;
    }

    return 0;
}


int mainNoise()
{
    //eigen matrices
    auto [A,B] = createZOHHCW(ORBITAL_RATE, DT);
    Eigen::MatrixXd C = Eigen::MatrixXd::Identity(6,6);
    
    HCW_Dynamics space_sim(ORBITAL_RATE);
    systems::Simulator<double> simulator(space_sim);
    systems::ContinuousState<double>& state = simulator.get_mutable_context().get_mutable_continuous_state();

    //initialize state
    state[0] = 10;
    state[1] = 10;
    state[2] = 10;
    state[3] = 1;
    state[4] = 1;
    state[5] = 1;

    KalmanFilter state_estimator(A,B,C,Eigen::MatrixXd::Identity(6,6),Eigen::MatrixXd::Identity(6,6)*1e-3);
    state_estimator.init(state.get_vector().CopyToVector(), Eigen::MatrixXd::Identity(6,6));

    VectorX<double> u(3);
    u << 0,0,0;

    
    
    std::vector<Belief> beliefs;
    std::vector<Eigen::VectorXd> control_inputs;
    std::vector<Eigen::VectorXd> real_values;

    beliefs.push_back(state_estimator.getBelief());
    real_values.push_back(state_estimator.getEstimate());
    for (int t = 0; t < 5; ++t)
    {
        u << t*2, t*2, t*2;

        space_sim.get_input_port(0).FixValue(&simulator.get_mutable_context(), u);
        simulator.AdvanceTo((t+1)*DT);

        Eigen::VectorXd meas = state.get_vector().CopyToVector() + Eigen::MatrixXd::Random(6,1);
        state_estimator.estimate(u,meas);


        beliefs.push_back(state_estimator.getBelief());
        control_inputs.push_back(u);

        real_values.push_back(state.get_vector().CopyToVector());
    }

    std::cout << "Normal Error:\n";
    Eigen::VectorXd avg = Eigen::MatrixXd::Zero(6,1);
    for (int t = 0; t < 5; ++t)
    {
        std::cout << "t="<<t<<":\n";
        auto [mu, cov] = beliefs.at(t);
        avg += mu - real_values.at(t);
    }
    std::cout << "\tMSE:\n";
    std::cout << avg / 5 << std::endl;

    state_estimator.kalmanSmooth(beliefs,control_inputs);
    std::cout << "\n\n";

    std::cout << "Smoothed Error:\n";

    avg = Eigen::MatrixXd::Zero(6,1);
    for (int t = 0; t < 5; ++t)
    {
        std::cout << "t="<<t<<":\n";
        auto [mu, cov] = beliefs.at(t);
        avg += mu - real_values.at(t);
    }
    std::cout << "\tMSE:\n";
    std::cout << avg / 5 << std::endl;

    return 0;
}



int main()
{
    //run test with no noise
    std::cout << "Test without Noise:\n\n";
    mainNoNoise();
    std::cout << "Test with Noise:\n\n";
    mainNoise();
}