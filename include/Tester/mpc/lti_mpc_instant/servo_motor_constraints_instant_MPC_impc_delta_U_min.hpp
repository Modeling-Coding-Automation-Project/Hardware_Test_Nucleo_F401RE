#ifndef SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_DELTA_U_MIN_HPP_
#define SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_DELTA_U_MIN_HPP_

#include "python_numpy.hpp"

namespace servo_motor_constraints_instant_MPC_impc_delta_U_min {

using namespace PythonNumpy;

using SparseAvailable_impc_delta_U_min =
    SparseAvailable<ColumnAvailable<false>>;

using type = SparseMatrix_Type<float, SparseAvailable_impc_delta_U_min>;

inline auto make(void) -> type { return make_SparseMatrixEmpty<float, 1, 1>(); }

} // namespace servo_motor_constraints_instant_MPC_impc_delta_U_min

#endif // SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_DELTA_U_MIN_HPP_
