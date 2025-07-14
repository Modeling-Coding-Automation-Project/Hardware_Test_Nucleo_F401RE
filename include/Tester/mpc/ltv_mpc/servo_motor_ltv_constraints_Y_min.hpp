#ifndef __SERVO_MOTOR_LTV_CONSTRAINTS_Y_MIN_HPP__
#define __SERVO_MOTOR_LTV_CONSTRAINTS_Y_MIN_HPP__

#include "python_numpy.hpp"

namespace servo_motor_ltv_constraints_Y_min {

using namespace PythonNumpy;

using SparseAvailable_Y_min =
    SparseAvailable<ColumnAvailable<false>, ColumnAvailable<false>>;

using type = SparseMatrix_Type<float, SparseAvailable_Y_min>;

inline auto make(void) -> type { return make_SparseMatrixEmpty<float, 2, 1>(); }

} // namespace servo_motor_ltv_constraints_Y_min

#endif // __SERVO_MOTOR_LTV_CONSTRAINTS_Y_MIN_HPP__
