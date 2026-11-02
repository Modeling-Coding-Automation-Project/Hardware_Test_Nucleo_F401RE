#ifndef SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_WCZT_QK_HPP_
#define SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_WCZT_QK_HPP_

#include "python_numpy.hpp"

namespace servo_motor_constraints_instant_MPC_impc_WCzT_Qk {

using namespace PythonNumpy;

using SparseAvailable_impc_WCzT_Qk = SparseAvailable<
    ColumnAvailable<false, false>, ColumnAvailable<false, false>,
    ColumnAvailable<false, false>, ColumnAvailable<false, false>,
    ColumnAvailable<false, false>, ColumnAvailable<false, false>>;

using type = SparseMatrix_Type<float, SparseAvailable_impc_WCzT_Qk>;

inline auto make(void) -> type { return make_SparseMatrixEmpty<float, 6, 2>(); }

} // namespace servo_motor_constraints_instant_MPC_impc_WCzT_Qk

#endif // SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_WCZT_QK_HPP_
