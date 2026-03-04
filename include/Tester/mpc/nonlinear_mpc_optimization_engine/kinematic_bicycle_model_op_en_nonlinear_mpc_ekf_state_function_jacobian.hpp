#ifndef __KINEMATIC_BICYCLE_MODEL_OP_EN_NONLINEAR_MPC_EKF_STATE_FUNCTION_JACOBIAN_HPP__
#define __KINEMATIC_BICYCLE_MODEL_OP_EN_NONLINEAR_MPC_EKF_STATE_FUNCTION_JACOBIAN_HPP__

#include "kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_A.hpp"
#include "kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_C.hpp"
#include "kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_parameter.hpp"

#include "python_control.hpp"

namespace kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_state_function_jacobian {

using namespace PythonControl;

using Parameter_Type = kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_parameter::Parameter_Type;

using namespace PythonMath;

using A_Type = kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_A::type;
using X_Type = StateSpaceState_Type<double, A_Type::COLS>;
using U_Type = StateSpaceInput_Type<double, 2>;

inline auto sympy_function(const double v, const double q0, const double delta_time, const double delta, const double wheel_base, const double q3) -> A_Type {

    A_Type result;

    double x0 = delta_time * v;

    double x1 = q0 * x0;

    double x2 = x0 * tan(delta) / (2 * wheel_base);

    double x3 = cos(x2);

    double x4 = sin(x2);

    result.template set<0, 0>(static_cast<double>(1));
    result.template set<0, 1>(static_cast<double>(0));
    result.template set<0, 2>(static_cast<double>(4 * x1));
    result.template set<0, 3>(static_cast<double>(0));
    result.template set<1, 0>(static_cast<double>(0));
    result.template set<1, 1>(static_cast<double>(1));
    result.template set<1, 2>(static_cast<double>(2 * q3 * x0));
    result.template set<1, 3>(static_cast<double>(2 * x1));
    result.template set<2, 0>(static_cast<double>(0));
    result.template set<2, 1>(static_cast<double>(0));
    result.template set<2, 2>(static_cast<double>(x3));
    result.template set<2, 3>(static_cast<double>(-x4));
    result.template set<3, 0>(static_cast<double>(0));
    result.template set<3, 1>(static_cast<double>(0));
    result.template set<3, 2>(static_cast<double>(x4));
    result.template set<3, 3>(static_cast<double>(x3));
    
    return result;
}

inline auto function(const X_Type X, const U_Type U, const Parameter_Type Parameters) -> A_Type {

    double q0 = X.template get<2, 0>();

    double q3 = X.template get<3, 0>();

    double v = U.template get<0, 0>();

    double delta = U.template get<1, 0>();

    double delta_time = Parameters.delta_time;

    double wheel_base = Parameters.wheel_base;

    return sympy_function(v, q0, delta_time, delta, wheel_base, q3);
}


} // namespace kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_state_function_jacobian

#endif // __KINEMATIC_BICYCLE_MODEL_OP_EN_NONLINEAR_MPC_EKF_STATE_FUNCTION_JACOBIAN_HPP__
