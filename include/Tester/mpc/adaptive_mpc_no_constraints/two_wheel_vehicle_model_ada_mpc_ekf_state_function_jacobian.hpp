#ifndef __TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_EKF_STATE_FUNCTION_JACOBIAN_HPP__
#define __TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_EKF_STATE_FUNCTION_JACOBIAN_HPP__

#include "two_wheel_vehicle_model_ada_mpc_ekf_parameter.hpp"
#include "two_wheel_vehicle_model_ekf_A.hpp"
#include "two_wheel_vehicle_model_ekf_C.hpp"

#include "python_control.hpp"

using namespace PythonControl;

using Parameter_Type =
    two_wheel_vehicle_model_ada_mpc_ekf_parameter::Parameter_Type;

using namespace PythonMath;

namespace two_wheel_vehicle_model_ada_mpc_ekf_state_function_jacobian {

using A_Type = two_wheel_vehicle_model_ekf_A::type;
using X_Type = StateSpaceState_Type<float, A_Type::COLS>;
using U_Type = StateSpaceInput_Type<float, 2>;

inline auto sympy_function(const float delta, const float r, const float l_f,
                           const float I, const float K_r, const float K_f,
                           const float l_r, const float m, const float theta,
                           const float beta, const float V) -> A_Type {

  A_Type result;

  float x0 = 0.01 * sin(theta);

  float x1 = 0.01 * cos(theta);

  float x2 = K_f * (l_f * l_f);

  float x3 = K_r * (l_r * l_r);

  float x4 = 0.02 / I;

  float x5 = x4 / V;

  float x6 = K_f * l_f;

  float x7 = V * x6;

  float x8 = V * V;

  float x9 = 1 / x8;

  float x10 = 2 * x6;

  float x11 = m * x8;

  float x12 = 1 / m;

  float x13 = 0.01 * x12 * x9;

  float x14 = 2 * V;

  float x15 = K_f * x14;

  float x16 = K_r * x14;

  float x17 = 2 * beta;

  result.template set<0, 0>(static_cast<float>(1));
  result.template set<0, 1>(static_cast<float>(0));
  result.template set<0, 2>(static_cast<float>(-V * x0));
  result.template set<0, 3>(static_cast<float>(0));
  result.template set<0, 4>(static_cast<float>(0));
  result.template set<0, 5>(static_cast<float>(x1));
  result.template set<1, 0>(static_cast<float>(0));
  result.template set<1, 1>(static_cast<float>(1));
  result.template set<1, 2>(static_cast<float>(V * x1));
  result.template set<1, 3>(static_cast<float>(0));
  result.template set<1, 4>(static_cast<float>(0));
  result.template set<1, 5>(static_cast<float>(x0));
  result.template set<2, 0>(static_cast<float>(0));
  result.template set<2, 1>(static_cast<float>(0));
  result.template set<2, 2>(static_cast<float>(1));
  result.template set<2, 3>(static_cast<float>(0.01));
  result.template set<2, 4>(static_cast<float>(0));
  result.template set<2, 5>(static_cast<float>(0));
  result.template set<3, 0>(static_cast<float>(0));
  result.template set<3, 1>(static_cast<float>(0));
  result.template set<3, 2>(static_cast<float>(0));
  result.template set<3, 3>(static_cast<float>(x5 * (-x2 - x3) + 1));
  result.template set<3, 4>(static_cast<float>(x5 * (K_r * V * l_r - x7)));
  result.template set<3, 5>(
      static_cast<float>(-x4 * x9 *
                             (K_f * V * delta * l_f + K_r * V * beta * l_r -
                              beta * x7 - r * x2 - r * x3) +
                         x5 * (K_r * beta * l_r - beta * x6 + delta * x6)));
  result.template set<4, 0>(static_cast<float>(0));
  result.template set<4, 1>(static_cast<float>(0));
  result.template set<4, 2>(static_cast<float>(0));
  result.template set<4, 3>(
      static_cast<float>(x13 * (2 * K_r * l_r - x10 - x11)));
  result.template set<4, 4>(static_cast<float>(x13 * (-x15 - x16) + 1));
  result.template set<4, 5>(static_cast<float>(
      x13 * (2 * K_f * delta - K_f * x17 - K_r * x17 - m * r * x14) -
      0.02 * x12 *
          (2 * K_f * V * delta + 2 * K_r * l_r * r - beta * x15 - beta * x16 -
           r * x10 - r * x11) /
          (V * V * V)));
  result.template set<5, 0>(static_cast<float>(0));
  result.template set<5, 1>(static_cast<float>(0));
  result.template set<5, 2>(static_cast<float>(0));
  result.template set<5, 3>(static_cast<float>(0));
  result.template set<5, 4>(static_cast<float>(0));
  result.template set<5, 5>(static_cast<float>(1));

  return result;
}

inline auto function(const X_Type X, const U_Type U,
                     const Parameter_Type Parameters) -> A_Type {

  float theta = X.template get<2, 0>();

  float r = X.template get<3, 0>();

  float beta = X.template get<4, 0>();

  float V = X.template get<5, 0>();

  float delta = U.template get<0, 0>();

  float l_f = Parameters.l_f;

  float I = Parameters.I;

  float K_r = Parameters.K_r;

  float K_f = Parameters.K_f;

  float l_r = Parameters.l_r;

  float m = Parameters.m;

  return sympy_function(delta, r, l_f, I, K_r, K_f, l_r, m, theta, beta, V);
}

} // namespace two_wheel_vehicle_model_ada_mpc_ekf_state_function_jacobian

#endif // __TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_EKF_STATE_FUNCTION_JACOBIAN_HPP__
