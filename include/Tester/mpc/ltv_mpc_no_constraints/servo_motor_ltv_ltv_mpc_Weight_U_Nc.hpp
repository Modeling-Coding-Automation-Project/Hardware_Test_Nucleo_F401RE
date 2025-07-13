#ifndef __SERVO_MOTOR_LTV_LTV_MPC_WEIGHT_U_NC_HPP__
#define __SERVO_MOTOR_LTV_LTV_MPC_WEIGHT_U_NC_HPP__

#include "python_numpy.hpp"

namespace servo_motor_ltv_ltv_mpc_Weight_U_Nc {

using namespace PythonNumpy;

using type = DiagMatrix_Type<double, 2>;

inline auto make(void) -> type {

  return make_DiagMatrix<2>(
    static_cast<double>(0.001),
    static_cast<double>(0.001)
  );

}

} // namespace servo_motor_ltv_ltv_mpc_Weight_U_Nc

#endif // __SERVO_MOTOR_LTV_LTV_MPC_WEIGHT_U_NC_HPP__
