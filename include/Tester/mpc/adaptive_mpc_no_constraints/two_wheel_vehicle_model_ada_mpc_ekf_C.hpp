#ifndef TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_EKF_C_HPP_
#define TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_EKF_C_HPP_

#include "python_numpy.hpp"

namespace two_wheel_vehicle_model_ada_mpc_ekf_C {

using namespace PythonNumpy;

using SparseAvailable_ada_mpc_ekf_C =
    SparseAvailable<ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<false, true, false, false, false, false>,
                    ColumnAvailable<false, false, true, false, false, false>,
                    ColumnAvailable<false, false, false, true, false, false>,
                    ColumnAvailable<false, false, false, false, false, true>>;

using type = SparseMatrix_Type<float, SparseAvailable_ada_mpc_ekf_C>;

inline auto make(void) -> type {

  auto result = make_SparseMatrixZeros<float, SparseAvailable_ada_mpc_ekf_C>();

  result.unsafe_access(0) = static_cast<float>(1.0);
  result.unsafe_access(1) = static_cast<float>(1.0);
  result.unsafe_access(2) = static_cast<float>(1.0);
  result.unsafe_access(3) = static_cast<float>(1.0);
  result.unsafe_access(4) = static_cast<float>(1.0);

  return result;
}

} // namespace two_wheel_vehicle_model_ada_mpc_ekf_C

#endif // TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_EKF_C_HPP_
