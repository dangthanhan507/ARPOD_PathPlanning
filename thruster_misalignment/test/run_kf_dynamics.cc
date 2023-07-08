#include <lib_dynamics.hpp>
#include <lib_kalman_filter.hpp>
#include <iostream>

#define ORBITAL_RATE 1
#define DT 1

using namespace drake;

int main()
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

    for (int t = 0; t < 5; ++t)
    {
        std::cout << "Time " << t+1 << ": \n";

        space_sim.get_input_port(0).FixValue(&simulator.get_mutable_context(), u);
        std::cout << "Prediction Est: \n";
        std::cout << A*state.get_vector().CopyToVector() + B*u << std::endl;

        simulator.AdvanceTo((t+1)*DT);

        auto meas = state.get_vector().CopyToVector();
        state_estimator.estimate(u,meas);
        std::cout << "Kalman Filter Est: \n";
        std::cout << state_estimator.getEstimate() << std::endl;
        std::cout << "Real Value: \n";
        std::cout << state.get_vector().CopyToVector() << std::endl << std::endl;
    }
    return 0;
}
