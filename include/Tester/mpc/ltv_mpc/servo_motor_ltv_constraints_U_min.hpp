#ifndef __SERVO_MOTOR_LTV_CONSTRAINTS_U_MIN_HPP__
#define __SERVO_MOTOR_LTV_CONSTRAINTS_U_MIN_HPP__

#include "python_numpy.hpp"

namespace servo_motor_ltv_constraints_U_min {

using namespace PythonNumpy;

using type = DenseMatrix_Type<float, 1, 1>;

inline auto make(void) -> type {

  return make_DenseMatrix<1, 1>(static_cast<float>(1.0));
}

} // namespace servo_motor_ltv_constraints_U_min

#endif // __SERVO_MOTOR_LTV_CONSTRAINTS_U_MIN_HPP__
