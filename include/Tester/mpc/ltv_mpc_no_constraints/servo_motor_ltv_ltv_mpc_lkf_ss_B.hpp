#ifndef SERVO_MOTOR_LTV_LTV_MPC_LKF_SS_B_HPP_
#define SERVO_MOTOR_LTV_LTV_MPC_LKF_SS_B_HPP_

#include "python_numpy.hpp"

namespace servo_motor_ltv_ltv_mpc_lkf_ss_B {

using namespace PythonNumpy;

using SparseAvailable_ltv_mpc_lkf_ss_B =
    SparseAvailable<ColumnAvailable<false>, ColumnAvailable<false>,
                    ColumnAvailable<false>, ColumnAvailable<true>>;

using type = SparseMatrix_Type<float, SparseAvailable_ltv_mpc_lkf_ss_B>;

inline auto make(void) -> type {

  return make_SparseMatrix<SparseAvailable_ltv_mpc_lkf_ss_B>(
      static_cast<float>(0.04999999999999999));
}

} // namespace servo_motor_ltv_ltv_mpc_lkf_ss_B

#endif // SERVO_MOTOR_LTV_LTV_MPC_LKF_SS_B_HPP_
