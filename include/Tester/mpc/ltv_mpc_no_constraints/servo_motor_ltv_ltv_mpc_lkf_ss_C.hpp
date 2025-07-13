#ifndef __SERVO_MOTOR_LTV_LTV_MPC_LKF_SS_C_HPP__
#define __SERVO_MOTOR_LTV_LTV_MPC_LKF_SS_C_HPP__

#include "python_numpy.hpp"

namespace servo_motor_ltv_ltv_mpc_lkf_ss_C {

using namespace PythonNumpy;

using SparseAvailable_ltv_mpc_lkf_ss_C = SparseAvailable<
    ColumnAvailable<true, false, false, false>,
    ColumnAvailable<true, false, true, false>
>;

using type = SparseMatrix_Type<double, SparseAvailable_ltv_mpc_lkf_ss_C>;

inline auto make(void) -> type {

  return make_SparseMatrix<SparseAvailable_ltv_mpc_lkf_ss_C>(
    static_cast<double>(1.0),
    static_cast<double>(1280.19900633784),
    static_cast<double>(-64.00995031689202)
  );

}

} // namespace servo_motor_ltv_ltv_mpc_lkf_ss_C

#endif // __SERVO_MOTOR_LTV_LTV_MPC_LKF_SS_C_HPP__
