
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>

#include <iostream>

//sim in seconds
#define SIM_DT 0.002 


namespace drake {


class HCW_Dynamics : public systems::LeafSystem<double>
{
    public:
    HCW_Dynamics(double n)
    {
        // xdotdot = 3n^2x + 2nydot + ux
        // ydotdot = -2nxdot + uy
        // zdotdot = -n^2z + uz
        
        //state vector = [pos, posdot] or [x,y,z,xdot,ydot,zdot]
        auto state_index = DeclareContinuousState(3,3,0);
        DeclareStateOutputPort("y", state_index); //output port will be integrated state
        DeclareNumericParameter(systems::BasicVector<double>(Vector1<double>(n))); //numeric parameter is the n for orbit rate
        DeclareVectorInputPort("u",3); // this is the control input
    }
    private:
    void DoCalcTimeDerivatives(const systems::Context<double>& context, 
    systems::ContinuousState<double>* derivatives) const override
    {
        //alloc state
        const double x = context.get_continuous_state().get_generalized_position().GetAtIndex(0);
        const double y = context.get_continuous_state().get_generalized_position().GetAtIndex(1);
        const double z = context.get_continuous_state().get_generalized_position().GetAtIndex(2);
        const double xdot = context.get_continuous_state().get_generalized_velocity().GetAtIndex(0);
        const double ydot = context.get_continuous_state().get_generalized_velocity().GetAtIndex(1);
        const double zdot = context.get_continuous_state().get_generalized_velocity().GetAtIndex(2);

        //alloc control input
        const auto u = EvalVectorInput(context, 0)->value(); //returns Vector<T> of three

        const double n = context.get_numeric_parameter(0).GetAtIndex(0);

        //for control input model: apply thrust throughout entire integration
        const double xdotdot = 3*n*n*x + 2*n*ydot + u[0];
        const double ydotdot = -2*n*xdot + u[1];
        const double zdotdot = -n*n*z + u[2];

        derivatives->get_mutable_generalized_position().SetAtIndex(0,xdot);
        derivatives->get_mutable_generalized_position().SetAtIndex(1,ydot);
        derivatives->get_mutable_generalized_position().SetAtIndex(2,zdot);
        derivatives->get_mutable_generalized_velocity().SetAtIndex(0,xdotdot);
        derivatives->get_mutable_generalized_velocity().SetAtIndex(1,ydotdot);
        derivatives->get_mutable_generalized_velocity().SetAtIndex(2,zdotdot);
    }
};

#define ORBITAL_RATE 1
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