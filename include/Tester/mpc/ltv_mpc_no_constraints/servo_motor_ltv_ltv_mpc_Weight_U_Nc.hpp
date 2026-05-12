#ifndef SERVO_MOTOR_LTV_LTV_MPC_WEIGHT_U_NC_HPP_
#define SERVO_MOTOR_LTV_LTV_MPC_WEIGHT_U_NC_HPP_

#include "python_numpy.hpp"

namespace servo_motor_ltv_ltv_mpc_Weight_U_Nc {

using namespace PythonNumpy;

using type = DiagMatrix_Type<float, 2>;

inline auto make(void) -> type {

  return make_DiagMatrix<2>(static_cast<float>(0.001),
                            static_cast<float>(0.001));
}

} // namespace servo_motor_ltv_ltv_mpc_Weight_U_Nc

#endif // SERVO_MOTOR_LTV_LTV_MPC_WEIGHT_U_NC_HPP_
