#ifndef __SERVO_MOTOR_LTV_LTV_MPC_LKF_SS_A_HPP__
#define __SERVO_MOTOR_LTV_LTV_MPC_LKF_SS_A_HPP__

#include "python_numpy.hpp"

namespace servo_motor_ltv_ltv_mpc_lkf_ss_A {

using namespace PythonNumpy;

using SparseAvailable_ltv_mpc_lkf_ss_A =
    SparseAvailable<ColumnAvailable<true, true, false, false>,
                    ColumnAvailable<true, true, true, false>,
                    ColumnAvailable<false, false, true, true>,
                    ColumnAvailable<true, false, true, true>>;

using type = SparseMatrix_Type<float, SparseAvailable_ltv_mpc_lkf_ss_A>;

inline auto make(void) -> type {

  return make_SparseMatrix<SparseAvailable_ltv_mpc_lkf_ss_A>(
      static_cast<float>(1.0), static_cast<float>(0.05),
      static_cast<float>(-2.5603853840856567),
      static_cast<float>(0.9500002466138069),
      static_cast<float>(0.12801926920428283), static_cast<float>(1.0),
      static_cast<float>(0.05), static_cast<float>(6.4009950316892),
      static_cast<float>(-0.32004975158446064),
      static_cast<float>(0.4900000000000001));
}

} // namespace servo_motor_ltv_ltv_mpc_lkf_ss_A

#endif // __SERVO_MOTOR_LTV_LTV_MPC_LKF_SS_A_HPP__
