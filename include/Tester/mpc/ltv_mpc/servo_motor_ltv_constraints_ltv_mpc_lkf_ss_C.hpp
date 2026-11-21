#ifndef SERVO_MOTOR_LTV_CONSTRAINTS_LTV_MPC_LKF_SS_C_HPP_
#define SERVO_MOTOR_LTV_CONSTRAINTS_LTV_MPC_LKF_SS_C_HPP_

#include "python_numpy.hpp"

namespace servo_motor_ltv_constraints_ltv_mpc_lkf_ss_C {

using namespace PythonNumpy;

using SparseAvailable_ltv_mpc_lkf_ss_C =
    SparseAvailable<ColumnAvailable<true, false, false, false>,
                    ColumnAvailable<true, false, true, false>>;

using type = SparseMatrix_Type<float, SparseAvailable_ltv_mpc_lkf_ss_C>;

inline auto make(void) -> type {

  auto result =
      make_SparseMatrixZeros<float, SparseAvailable_ltv_mpc_lkf_ss_C>();

  result.unsafe_access(0) = static_cast<float>(1.0);
  result.unsafe_access(1) = static_cast<float>(1280.19900633784);
  result.unsafe_access(2) = static_cast<float>(-64.00995031689202);

  return result;
}

} // namespace servo_motor_ltv_constraints_ltv_mpc_lkf_ss_C

#endif // SERVO_MOTOR_LTV_CONSTRAINTS_LTV_MPC_LKF_SS_C_HPP_
