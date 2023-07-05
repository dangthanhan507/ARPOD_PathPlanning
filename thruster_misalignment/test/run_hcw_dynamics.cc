#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <lib_dynamics.hpp>
#include <iostream>

#define ORBITAL_RATE 1
namespace drake {

void runMain()
{
    //instead of using a builder, we have one singular block (sim block)
    HCW_Dynamics space_sim(ORBITAL_RATE);
    systems::Simulator<double> simulator(space_sim);
    systems::ContinuousState<double>& state = simulator.get_mutable_context().get_mutable_continuous_state();
    //pos
    state[0] = 10;
    state[1] = 10;
    state[2] = 10;
    state[3] = 1;
    state[4] = 1;
    state[5] = 1;

    //control input
    VectorX<double> u(3);
    u << 1,0,0;
    space_sim.get_input_port(0).FixValue(&simulator.get_mutable_context(), u);

    simulator.AdvanceTo(1); //move forward in seconds
    std::cout << state.get_vector() << std::endl;
}
}//namespace drake

int main()
{
    drake::runMain();
    return 0;
}