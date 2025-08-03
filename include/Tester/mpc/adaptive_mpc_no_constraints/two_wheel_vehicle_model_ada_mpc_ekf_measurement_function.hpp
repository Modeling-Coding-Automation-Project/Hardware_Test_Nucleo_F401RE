#ifndef __TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_EKF_MEASUREMENT_FUNCTION_HPP__
#define __TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_EKF_MEASUREMENT_FUNCTION_HPP__

#include "two_wheel_vehicle_model_ada_mpc_ekf_parameter.hpp"
#include "two_wheel_vehicle_model_ekf_A.hpp"
#include "two_wheel_vehicle_model_ekf_C.hpp"

#include "python_control.hpp"

using namespace PythonControl;

using Parameter_Type =
    two_wheel_vehicle_model_ada_mpc_ekf_parameter::Parameter_Type;

using namespace PythonMath;

namespace two_wheel_vehicle_model_ada_mpc_ekf_measurement_function {

using A_Type = two_wheel_vehicle_model_ekf_A::type;
using C_Type = two_wheel_vehicle_model_ekf_C::type;
using X_Type = StateSpaceState_Type<float, A_Type::COLS>;
using Y_Type = StateSpaceOutput_Type<float, C_Type::COLS>;

inline auto sympy_function(const float r, const float theta, const float px,
                           const float V, const float py) -> Y_Type {

  Y_Type result;

  result.template set<0, 0>(static_cast<float>(px));
  result.template set<1, 0>(static_cast<float>(py));
  result.template set<2, 0>(static_cast<float>(theta));
  result.template set<3, 0>(static_cast<float>(r));
  result.template set<4, 0>(static_cast<float>(V));

  return result;
}

inline auto function(const X_Type X, const Parameter_Type Parameters)
    -> Y_Type {

  float px = X.template get<0, 0>();

  float py = X.template get<1, 0>();

  float theta = X.template get<2, 0>();

  float r = X.template get<3, 0>();

  float V = X.template get<5, 0>();

  return sympy_function(r, theta, px, V, py);
}

} // namespace two_wheel_vehicle_model_ada_mpc_ekf_measurement_function

#endif // __TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_EKF_MEASUREMENT_FUNCTION_HPP__
