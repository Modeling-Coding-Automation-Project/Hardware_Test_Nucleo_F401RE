#ifndef TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_ADA_MPC_U_MIN_HPP_
#define TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_ADA_MPC_U_MIN_HPP_

#include "python_numpy.hpp"

namespace two_wheel_vehicle_model_constraints_ada_mpc_U_min {

using namespace PythonNumpy;

using type = DenseMatrix_Type<float, 2, 1>;

inline auto make(void) -> type {

  auto result = make_DenseMatrixZeros<float, 2, 1>();

  result.unsafe_access(0, 0) = static_cast<float>(1.0);
  result.unsafe_access(1, 0) = static_cast<float>(1.0);

  return result;
}

} // namespace two_wheel_vehicle_model_constraints_ada_mpc_U_min

#endif // TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_ADA_MPC_U_MIN_HPP_
