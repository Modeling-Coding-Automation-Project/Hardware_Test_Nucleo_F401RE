#ifndef __KINEMATIC_BICYCLE_MODEL_OP_EN_NONLINEAR_MPC_EKF_MEASUREMENT_FUNCTION_HPP__
#define __KINEMATIC_BICYCLE_MODEL_OP_EN_NONLINEAR_MPC_EKF_MEASUREMENT_FUNCTION_HPP__

#include "kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_A.hpp"
#include "kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_C.hpp"
#include "kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_parameter.hpp"

#include "python_control.hpp"

namespace kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_measurement_function {

using namespace PythonControl;

using Parameter_Type = kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_parameter::Parameter_Type;

using namespace PythonMath;

using A_Type = kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_A::type;
using C_Type = kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_C::type;
using X_Type = StateSpaceState_Type<double, A_Type::COLS>;
using Y_Type = StateSpaceOutput_Type<double, C_Type::COLS>;

inline auto sympy_function(const double q3, const double px, const double py, const double q0) -> Y_Type {

    Y_Type result;

    result.template set<0, 0>(static_cast<double>(px));
    result.template set<1, 0>(static_cast<double>(py));
    result.template set<2, 0>(static_cast<double>(q0));
    result.template set<3, 0>(static_cast<double>(q3));
    
    return result;
}

inline auto function(const X_Type X, const Parameter_Type Parameters) -> Y_Type {

    double px = X.template get<0, 0>();

    double py = X.template get<1, 0>();

    double q0 = X.template get<2, 0>();

    double q3 = X.template get<3, 0>();

    return sympy_function(q3, px, py, q0);
}


} // namespace kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_measurement_function

#endif // __KINEMATIC_BICYCLE_MODEL_OP_EN_NONLINEAR_MPC_EKF_MEASUREMENT_FUNCTION_HPP__
