#ifndef SERVO_MOTOR_LTV_LTV_MPC_LKF_SS_A_HPP_
#define SERVO_MOTOR_LTV_LTV_MPC_LKF_SS_A_HPP_

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

  auto result =
      make_SparseMatrixZeros<float, SparseAvailable_ltv_mpc_lkf_ss_A>();

  result.unsafe_access(0) = static_cast<float>(1.0);
  result.unsafe_access(1) = static_cast<float>(0.05);
  result.unsafe_access(2) = static_cast<float>(-2.5603853840856567);
  result.unsafe_access(3) = static_cast<float>(0.9500002466138069);
  result.unsafe_access(4) = static_cast<float>(0.12801926920428283);
  result.unsafe_access(5) = static_cast<float>(1.0);
  result.unsafe_access(6) = static_cast<float>(0.05);
  result.unsafe_access(7) = static_cast<float>(6.4009950316892);
  result.unsafe_access(8) = static_cast<float>(-0.32004975158446064);
  result.unsafe_access(9) = static_cast<float>(0.4900000000000001);

  return result;
}

} // namespace servo_motor_ltv_ltv_mpc_lkf_ss_A

#endif // SERVO_MOTOR_LTV_LTV_MPC_LKF_SS_A_HPP_
