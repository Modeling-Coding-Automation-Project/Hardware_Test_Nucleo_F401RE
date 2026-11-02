#ifndef SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_BG_HPP_
#define SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_BG_HPP_

#include "python_numpy.hpp"

namespace servo_motor_constraints_instant_MPC_impc_bg {

using namespace PythonNumpy;

using type = DenseMatrix_Type<float, 40, 1>;

inline auto make(void) -> type {

  auto result = make_DenseMatrixZeros<float, 40, 1>();

  result.unsafe_access(0, 0) = static_cast<float>(180.0);
  result.unsafe_access(1, 0) = static_cast<float>(180.0);
  result.unsafe_access(2, 0) = static_cast<float>(180.0);
  result.unsafe_access(3, 0) = static_cast<float>(180.0);
  result.unsafe_access(4, 0) = static_cast<float>(180.0);
  result.unsafe_access(5, 0) = static_cast<float>(180.0);
  result.unsafe_access(6, 0) = static_cast<float>(180.0);
  result.unsafe_access(7, 0) = static_cast<float>(180.0);
  result.unsafe_access(8, 0) = static_cast<float>(180.0);
  result.unsafe_access(9, 0) = static_cast<float>(180.0);
  result.unsafe_access(10, 0) = static_cast<float>(180.0);
  result.unsafe_access(11, 0) = static_cast<float>(180.0);
  result.unsafe_access(12, 0) = static_cast<float>(180.0);
  result.unsafe_access(13, 0) = static_cast<float>(180.0);
  result.unsafe_access(14, 0) = static_cast<float>(180.0);
  result.unsafe_access(15, 0) = static_cast<float>(180.0);
  result.unsafe_access(16, 0) = static_cast<float>(180.0);
  result.unsafe_access(17, 0) = static_cast<float>(180.0);
  result.unsafe_access(18, 0) = static_cast<float>(180.0);
  result.unsafe_access(19, 0) = static_cast<float>(180.0);
  result.unsafe_access(20, 0) = static_cast<float>(180.0);
  result.unsafe_access(21, 0) = static_cast<float>(180.0);
  result.unsafe_access(22, 0) = static_cast<float>(180.0);
  result.unsafe_access(23, 0) = static_cast<float>(180.0);
  result.unsafe_access(24, 0) = static_cast<float>(180.0);
  result.unsafe_access(25, 0) = static_cast<float>(180.0);
  result.unsafe_access(26, 0) = static_cast<float>(180.0);
  result.unsafe_access(27, 0) = static_cast<float>(180.0);
  result.unsafe_access(28, 0) = static_cast<float>(180.0);
  result.unsafe_access(29, 0) = static_cast<float>(180.0);
  result.unsafe_access(30, 0) = static_cast<float>(180.0);
  result.unsafe_access(31, 0) = static_cast<float>(180.0);
  result.unsafe_access(32, 0) = static_cast<float>(180.0);
  result.unsafe_access(33, 0) = static_cast<float>(180.0);
  result.unsafe_access(34, 0) = static_cast<float>(180.0);
  result.unsafe_access(35, 0) = static_cast<float>(180.0);
  result.unsafe_access(36, 0) = static_cast<float>(180.0);
  result.unsafe_access(37, 0) = static_cast<float>(180.0);
  result.unsafe_access(38, 0) = static_cast<float>(180.0);
  result.unsafe_access(39, 0) = static_cast<float>(180.0);

  return result;
}

} // namespace servo_motor_constraints_instant_MPC_impc_bg

#endif // SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_BG_HPP_
