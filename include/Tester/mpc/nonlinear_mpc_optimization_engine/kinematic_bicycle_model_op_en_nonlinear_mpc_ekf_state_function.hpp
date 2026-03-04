#ifndef __KINEMATIC_BICYCLE_MODEL_OP_EN_NONLINEAR_MPC_EKF_STATE_FUNCTION_HPP__
#define __KINEMATIC_BICYCLE_MODEL_OP_EN_NONLINEAR_MPC_EKF_STATE_FUNCTION_HPP__

#include "kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_A.hpp"
#include "kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_parameter.hpp"

#include "python_control.hpp"

namespace kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_state_function {

using namespace PythonControl;

using Parameter_Type = kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_parameter::Parameter_Type;

using namespace PythonMath;

using A_Type = kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_A::type;
using X_Type = StateSpaceState_Type<double, A_Type::COLS>;
using U_Type = StateSpaceInput_Type<double, 2>;

inline auto sympy_function(const double v, const double px, const double q0, const double delta_time, const double delta, const double wheel_base, const double q3, const double py) -> X_Type {

    X_Type result;

    double x0 = delta_time * v;

    double x1 = x0 * tan(delta) / (2 * wheel_base);

    double x2 = cos(x1);

    double x3 = sin(x1);

    result.template set<0, 0>(static_cast<double>(px + x0 * (2 * (q0 * q0) - 1)));
    result.template set<1, 0>(static_cast<double>(py + 2 * q0 * q3 * x0));
    result.template set<2, 0>(static_cast<double>(q0 * x2 - q3 * x3));
    result.template set<3, 0>(static_cast<double>(q0 * x3 + q3 * x2));
    
    return result;
}

inline auto function(const X_Type X, const U_Type U, const Parameter_Type Parameters) -> X_Type {

    double px = X.template get<0, 0>();

    double py = X.template get<1, 0>();

    double q0 = X.template get<2, 0>();

    double q3 = X.template get<3, 0>();

    double v = U.template get<0, 0>();

    double delta = U.template get<1, 0>();

    double delta_time = Parameters.delta_time;

    double wheel_base = Parameters.wheel_base;

    return sympy_function(v, px, q0, delta_time, delta, wheel_base, q3, py);
}


} // namespace kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_state_function

#endif // __KINEMATIC_BICYCLE_MODEL_OP_EN_NONLINEAR_MPC_EKF_STATE_FUNCTION_HPP__
