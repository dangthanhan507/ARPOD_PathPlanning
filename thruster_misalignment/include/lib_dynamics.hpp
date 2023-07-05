#ifndef LIB_DYNAMICS_HPP
#define LIB_DYNAMICS_HPP

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>

#include <Eigen/Dense>
#include <tuple>
namespace drake{

class HCW_Dynamics : public systems::LeafSystem<double>
{
    public:
    HCW_Dynamics(double n);

    private:
    void DoCalcTimeDerivatives(const systems::Context<double>& context, 
    systems::ContinuousState<double>* derivatives) const override;
};

//TODO: -> add nonlinear orbital dynamics
//      -> add Discrete HCW Dynamics

}//namespace drake

/**
 * Given the orbital rate, we create A,B dynamics matricces fitting equation
 * x_k+1 = Ax_k + Bu_k
 * This requires performing zero order hold on the HCW differential equations
 * which is precomputed
 * @param n: double for orbital rate
 * @param dt: double for timestep to propagate state vector by
*/
std::tuple<Eigen::MatrixXd,Eigen::MatrixXd> createZOHHCW(double n, double dt);

#endif