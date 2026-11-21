#ifndef TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_ADA_MPC_B_HPP_
#define TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_ADA_MPC_B_HPP_

#include "python_numpy.hpp"

namespace two_wheel_vehicle_model_constraints_ada_mpc_B {

using namespace PythonNumpy;

using SparseAvailable_ada_mpc_B =
    SparseAvailable<ColumnAvailable<false, false>,
                    ColumnAvailable<false, false>,
                    ColumnAvailable<false, false>, ColumnAvailable<true, false>,
                    ColumnAvailable<true, false>, ColumnAvailable<false, true>>;

using type = SparseMatrix_Type<float, SparseAvailable_ada_mpc_B>;

inline auto make(void) -> type {

  auto result = make_SparseMatrixZeros<float, SparseAvailable_ada_mpc_B>();

  result.unsafe_access(0) = static_cast<float>(1.0);
  result.unsafe_access(1) = static_cast<float>(1.0);
  result.unsafe_access(2) = static_cast<float>(1.0);

  return result;
}

} // namespace two_wheel_vehicle_model_constraints_ada_mpc_B

#endif // TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_ADA_MPC_B_HPP_
