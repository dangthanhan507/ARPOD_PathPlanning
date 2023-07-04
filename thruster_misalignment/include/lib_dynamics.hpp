#ifndef LIB_DYNAMICS_HPP
#define LIB_DYNAMICS_HPP

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>

class HCW_Dynamics : public systems::LeafSystem<T>
{
    public:
    HCW_Dynamics(double n);
    private:
    void DoCalcTimeDerivatives(const systems::Context<double>& context, 
    systems::ContinuousState<double>* derivatives);
};
#endif