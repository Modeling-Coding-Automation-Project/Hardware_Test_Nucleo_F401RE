#ifndef __SERVO_MOTOR_LTV_CONSTRAINTS_LTV_MPC_LKF_SS_D_HPP__
#define __SERVO_MOTOR_LTV_CONSTRAINTS_LTV_MPC_LKF_SS_D_HPP__

#include "python_numpy.hpp"

namespace servo_motor_ltv_constraints_ltv_mpc_lkf_ss_D {

using namespace PythonNumpy;

using SparseAvailable_ltv_mpc_lkf_ss_D = SparseAvailable<
    ColumnAvailable<false>,
    ColumnAvailable<false>
>;

using type = SparseMatrix_Type<double, SparseAvailable_ltv_mpc_lkf_ss_D>;

inline auto make(void) -> type {

  return make_SparseMatrixEmpty<double, 2, 1>();

}

} // namespace servo_motor_ltv_constraints_ltv_mpc_lkf_ss_D

#endif // __SERVO_MOTOR_LTV_CONSTRAINTS_LTV_MPC_LKF_SS_D_HPP__
