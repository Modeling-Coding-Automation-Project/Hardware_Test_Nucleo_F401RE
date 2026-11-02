#ifndef SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_LKF_SS_D_HPP_
#define SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_LKF_SS_D_HPP_

#include "python_numpy.hpp"

namespace servo_motor_constraints_instant_MPC_impc_lkf_ss_D {

using namespace PythonNumpy;

using SparseAvailable_impc_lkf_ss_D =
    SparseAvailable<ColumnAvailable<false>, ColumnAvailable<false>>;

using type = SparseMatrix_Type<float, SparseAvailable_impc_lkf_ss_D>;

inline auto make(void) -> type { return make_SparseMatrixEmpty<float, 2, 1>(); }

} // namespace servo_motor_constraints_instant_MPC_impc_lkf_ss_D

#endif // SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_LKF_SS_D_HPP_
