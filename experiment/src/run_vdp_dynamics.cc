/**
 * Goal:
 * run_vdp_dynamics.cc
 * 
 * Run the dynamics of HCW equations and plot them
 * This should be simple enough to get used to Drake C++ API
 * 
 * This will also help us setup the bazel build to allow full utilization of drake
 * in our C++ project.
 * 
 * TODO: move this over to a test folder
*/

//drake includes
#include <drake/systems/framework/leaf_system.h>
#include <drake/common/default_scalars.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/system_constraint.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <iostream>

namespace drake {


template <typename T>
class VanDerPolOscillator final : public systems::LeafSystem<T>
{
    public:

    VanDerPolOscillator()
    : systems::LeafSystem<T>(systems::SystemTypeTag<VanDerPolOscillator>{})
    {
        //initialized systems::LeafSystem with a tag

        //DeclareContinuousState(num positions, num velocities, num miscellaneous state variables)
        //auto = ContinuousStateIndex
        auto state_index = this->DeclareContinuousState(1,1,0);

        //first output
        this->DeclareVectorOutputPort(systems::kUseDefaultName, 1, &VanDerPolOscillator::CopyPositionToOutput);
        
        //second output (full state)
        this->DeclareStateOutputPort(systems::kUseDefaultName, state_index);

        //set mu = 1
        this->DeclareNumericParameter(systems::BasicVector<T>(Vector1<T>(1.0)));

        //declare mu >= 0 constraint
        systems::ContextConstraintCalc<T> mu = [](const systems::Context<T>& context, VectorX<T>* value)
        {
            *value = Vector1<T>(context.get_numeric_parameter(0).GetAtIndex(0));
        };

        //add mu into leafsystem constraint
        this->DeclareInequalityConstraint(mu, {Vector1d(0), std::nullopt}, "mu >= 0");
    }


    //converting copy consturctor
    template <typename U>
    explicit VanDerPolOscillator(const VanDerPolOscillator<U>&) : VanDerPolOscillator() {}

    //return output port
    const systems::OutputPort<T>& get_position_output_port() const
    {
        return this->get_output_port(0);
    }

    //return output port for full state
    const systems::OutputPort<T>& get_full_state_output_port() const
    {
        return this->get_output_port(1);
    }

    static Eigen::Matrix2Xd CalcLimitCycle()
    {
        systems::DiagramBuilder<double> builder;

        auto vdp = builder.AddSystem<VanDerPolOscillator<double>>();
        auto logger = LogVectorOutput(vdp->get_full_state_output_port(), &builder);
        auto diagram = builder.Build();

        systems::Simulator<double> simulator(*diagram);

        simulator.get_mutable_context().SetContinuousState(Eigen::Vector2d(-0.1144,2.0578)); //set the state
        simulator.AdvanceTo(6.667); //progress to 6.667 seconds

        return logger->FindLog(simulator.get_context()).data(); //return 2d data
    }

    private:

    //need to calculate for dynamics
    // xdot = f(x)

    //equation:
    // qdotdot + mu(q^2 - 1)qdot + q = 0
    void DoCalcTimeDerivatives(const systems::Context<T>& context, systems::ContinuousState<T>* derivatives) const override
    {
        const T q = context.get_continuous_state().get_generalized_position().GetAtIndex(0);
        const T qdot = context.get_continuous_state().get_generalized_velocity().GetAtIndex(0);
        const T mu = context.get_numeric_parameter(0).GetAtIndex(0);

        //this is dynamics equation 
        const T qdotdot = -mu * (q*q-1)*qdot - q;

        // setting up dynamics xdot=f(x) where x is position and velocity
        derivatives->get_mutable_generalized_position().SetAtIndex(0,qdot);
        derivatives->get_mutable_generalized_velocity().SetAtIndex(0,qdotdot);
    }

    void CopyPositionToOutput(const systems::Context<T>& context, systems::BasicVector<T>* output) const
    {
        output->SetAtIndex(0,context.get_continuous_state().get_generalized_position().GetAtIndex(0));
    }
};


} //namespace drake

int main()
{
    //returns 2dim Eigen Matrix
    const auto cycle = drake::VanDerPolOscillator<double>::CalcLimitCycle();

    const int N = cycle.cols();

    std::cout << cycle;

    return 0;
}