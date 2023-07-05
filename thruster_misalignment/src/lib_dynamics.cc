#include <lib_dynamics.hpp>
#include <math.h>
namespace drake {

HCW_Dynamics::HCW_Dynamics(double n)
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

void HCW_Dynamics::DoCalcTimeDerivatives(const systems::Context<double>& context, 
systems::ContinuousState<double>* derivatives) const
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

}//namespace drake

std::tuple<Eigen::MatrixXd,Eigen::MatrixXd> createZOHHCW(double n, double dt)
{
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6,6);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6,3);
    //A is 6x6
    A << 4-3*cos(n*dt), 0, 0, sin(n*dt)/n, 2*(1-cos(n*dt))/n, 0,
         6*(sin(n*dt)-n*dt), 1, 0, -2*(1-cos(n*dt))/n, (4*sin(n*dt)-3*n*dt)/n, 0,
         0,0,cos(n*dt),0,0,sin(n*dt)/n,
         3*n*sin(n*dt),0,0,cos(n*dt),2*sin(n*dt),0,
         -6*n*(1-cos(n*dt)), 0, 0, -2*sin(n*dt), 4*cos(n*dt)-3, 0,
         0,0,-n*sin(n*dt), 0, 0, cos(n*dt);
    //B is 6x3
    B << sin(n*dt)/n, 2*(1-cos(n*dt))/n, 0,
         -2*(n*dt - sin(n*dt))/n/n, 4*(1-cos(n*dt))/n/n-(3/2)*dt*dt, 0,
         0,0,(1-cos(n*dt))/n/n,
         sin(n*dt)/n,2*(1-cos(n*dt))/n,0,
         -2*(1-cos(n*dt))/n, 4*sin(n*dt)/n-3*dt, 0,
         0,0,sin(n*dt)/n;
    return std::make_tuple(A,B);
}