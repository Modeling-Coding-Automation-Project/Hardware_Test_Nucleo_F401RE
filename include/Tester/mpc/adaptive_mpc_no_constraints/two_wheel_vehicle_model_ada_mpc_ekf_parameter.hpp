#ifndef __TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_EKF_PARAMETER_HPP__
#define __TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_EKF_PARAMETER_HPP__

namespace two_wheel_vehicle_model_ada_mpc_ekf_parameter {

class Parameter {
public:
  float m = static_cast<float>(2000);
  float l_f = static_cast<float>(1.4);
  float l_r = static_cast<float>(1.6);
  float I = static_cast<float>(4000);
  float K_f = static_cast<float>(12000.0);
  float K_r = static_cast<float>(11000.0);
};

using Parameter_Type = Parameter;

} // namespace two_wheel_vehicle_model_ada_mpc_ekf_parameter

#endif // __TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_EKF_PARAMETER_HPP__
