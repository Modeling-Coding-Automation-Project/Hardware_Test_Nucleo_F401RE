#ifndef __TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_EKF_STATE_EQUATION_JACOBIAN_HPP__
#define __TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_EKF_STATE_EQUATION_JACOBIAN_HPP__

#include "two_wheel_vehicle_model_ada_mpc_ekf_A.hpp"
#include "two_wheel_vehicle_model_ada_mpc_ekf_C.hpp"
#include "two_wheel_vehicle_model_ada_mpc_ekf_parameter.hpp"

#include "python_control.hpp"

namespace two_wheel_vehicle_model_ada_mpc_ekf_state_equation_jacobian {

using namespace PythonControl;

using Parameter_Type =
    two_wheel_vehicle_model_ada_mpc_ekf_parameter::Parameter_Type;

using namespace PythonNumpy;

using A_Type = two_wheel_vehicle_model_ada_mpc_ekf_A::type;
using X_Type = StateSpaceState_Type<float, A_Type::ROWS>;
using U_Type = StateSpaceInput_Type<float, 2>;

inline auto sympy_function(const float K_r, const float m, const float beta,
                           const float r, const float I, const float V,
                           const float l_r, const float theta,
                           const float delta, const float K_f, const float l_f)
    -> A_Type {

  A_Type result;

  float x0 = 0.01 * sin(theta);

  float x1 = 0.01 * cos(theta);

  float x2 = 1 / I;

  float x3 = 1 / V;

  float x4 = K_f * (l_f * l_f);

  float x5 = K_r * (l_r * l_r);

  float x6 = K_f * l_f;

  float x7 = 0.02 * x2;

  float x8 = V * V;

  float x9 = 1 / x8;

  float x10 = 1 / m;

  float x11 = 0.02 * K_f;

  float x12 = 0.02 * K_r;

  float x13 = V * beta;

  float x14 = 0.04 * r;

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
  result.template set<3, 3>(
      static_cast<float>(x2 * x3 * (I * V - 0.02 * x4 - 0.02 * x5)));
  result.template set<3, 4>(static_cast<float>(x7 * (K_r * l_r - x6)));
  result.template set<3, 5>(static_cast<float>(r * x7 * x9 * (x4 + x5)));
  result.template set<4, 0>(static_cast<float>(0));
  result.template set<4, 1>(static_cast<float>(0));
  result.template set<4, 2>(static_cast<float>(0));
  result.template set<4, 3>(static_cast<float>(
      x10 * x9 * (0.02 * K_r * l_r - 0.01 * m * x8 - 0.02 * x6)));
  result.template set<4, 4>(static_cast<float>(x10 * x3 * (V * m - x11 - x12)));
  result.template set<4, 5>(static_cast<float>(
      1.0 * x10 *
      (-K_r * l_r * x14 - V * delta * x11 + x11 * x13 + x12 * x13 + x14 * x6) /
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

  float K_r = Parameters.K_r;

  float m = Parameters.m;

  float I = Parameters.I;

  float l_r = Parameters.l_r;

  float K_f = Parameters.K_f;

  float l_f = Parameters.l_f;

  return sympy_function(K_r, m, beta, r, I, V, l_r, theta, delta, K_f, l_f);
}

} // namespace two_wheel_vehicle_model_ada_mpc_ekf_state_equation_jacobian

#endif // __TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_EKF_STATE_EQUATION_JACOBIAN_HPP__
