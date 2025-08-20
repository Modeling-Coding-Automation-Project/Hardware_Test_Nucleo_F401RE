#ifndef __TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_ADA_MPC_EKF_STATE_FUNCTION_HPP__
#define __TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_ADA_MPC_EKF_STATE_FUNCTION_HPP__

#include "two_wheel_vehicle_model_constraints_ada_mpc_ekf_parameter.hpp"
#include "two_wheel_vehicle_model_constraints_ekf_A.hpp"

#include "python_control.hpp"

using namespace PythonControl;

using Parameter_Type =
    two_wheel_vehicle_model_constraints_ada_mpc_ekf_parameter::Parameter_Type;

using namespace PythonMath;

namespace two_wheel_vehicle_model_constraints_ada_mpc_ekf_state_function {

using A_Type = two_wheel_vehicle_model_constraints_ekf_A::type;
using X_Type = StateSpaceState_Type<float, A_Type::COLS>;
using U_Type = StateSpaceInput_Type<float, 2>;

inline auto sympy_function(const float px, const float V, const float K_r,
                           const float beta, const float theta,
                           const float accel, const float r, const float m,
                           const float delta, const float K_f, const float l_f,
                           const float I, const float py, const float l_r)
    -> X_Type {

  X_Type result;

  float x0 = 0.01 * V;

  float x1 = K_f * V;

  float x2 = K_f * r;

  float x3 = V * V;

  result.template set<0, 0>(static_cast<float>(px + x0 * cos(theta)));
  result.template set<1, 0>(static_cast<float>(py + x0 * sin(theta)));
  result.template set<2, 0>(static_cast<float>(0.01 * r + theta));
  result.template set<3, 0>(static_cast<float>(
      r + 0.02 *
              (K_f * V * delta * l_f + K_r * V * beta * l_r -
               K_r * (l_r * l_r) * r - beta * l_f * x1 - l_f * l_f * x2) /
              (I * V)));
  result.template set<4, 0>(static_cast<float>(
      beta + 0.01 *
                 (2 * K_f * V * delta - 2 * K_r * V * beta + 2 * K_r * l_r * r -
                  2 * beta * x1 - 2 * l_f * x2 - m * r * x3) /
                 (m * x3)));
  result.template set<5, 0>(static_cast<float>(V + 0.01 * accel));

  return result;
}

inline auto function(const X_Type X, const U_Type U,
                     const Parameter_Type Parameters) -> X_Type {

  float px = X.template get<0, 0>();

  float py = X.template get<1, 0>();

  float theta = X.template get<2, 0>();

  float r = X.template get<3, 0>();

  float beta = X.template get<4, 0>();

  float V = X.template get<5, 0>();

  float delta = U.template get<0, 0>();

  float accel = U.template get<1, 0>();

  float K_r = Parameters.K_r;

  float m = Parameters.m;

  float K_f = Parameters.K_f;

  float l_f = Parameters.l_f;

  float I = Parameters.I;

  float l_r = Parameters.l_r;

  return sympy_function(px, V, K_r, beta, theta, accel, r, m, delta, K_f, l_f,
                        I, py, l_r);
}

} // namespace two_wheel_vehicle_model_constraints_ada_mpc_ekf_state_function

#endif // __TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_ADA_MPC_EKF_STATE_FUNCTION_HPP__
