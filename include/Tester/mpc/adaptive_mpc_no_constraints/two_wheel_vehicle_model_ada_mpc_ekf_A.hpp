#ifndef TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_EKF_A_HPP_
#define TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_EKF_A_HPP_

#include "python_numpy.hpp"

namespace two_wheel_vehicle_model_ada_mpc_ekf_A {

using namespace PythonNumpy;

using SparseAvailable_ada_mpc_ekf_A =
    SparseAvailable<ColumnAvailable<true, false, true, false, false, true>,
                    ColumnAvailable<false, true, true, false, false, true>,
                    ColumnAvailable<false, false, true, true, false, false>,
                    ColumnAvailable<false, false, false, true, true, true>,
                    ColumnAvailable<false, false, false, true, true, true>,
                    ColumnAvailable<false, false, false, false, false, true>>;

using type = SparseMatrix_Type<float, SparseAvailable_ada_mpc_ekf_A>;

inline auto make(void) -> type {

  auto result = make_SparseMatrixZeros<float, SparseAvailable_ada_mpc_ekf_A>();

  result.unsafe_access(0) = static_cast<float>(1.0);
  result.unsafe_access(1) = static_cast<float>(1.0);
  result.unsafe_access(2) = static_cast<float>(1.0);
  result.unsafe_access(3) = static_cast<float>(1.0);
  result.unsafe_access(4) = static_cast<float>(1.0);
  result.unsafe_access(5) = static_cast<float>(1.0);
  result.unsafe_access(6) = static_cast<float>(1.0);
  result.unsafe_access(7) = static_cast<float>(1.0);
  result.unsafe_access(8) = static_cast<float>(1.0);
  result.unsafe_access(9) = static_cast<float>(1.0);
  result.unsafe_access(10) = static_cast<float>(1.0);
  result.unsafe_access(11) = static_cast<float>(1.0);
  result.unsafe_access(12) = static_cast<float>(1.0);
  result.unsafe_access(13) = static_cast<float>(1.0);
  result.unsafe_access(14) = static_cast<float>(1.0);

  return result;
}

} // namespace two_wheel_vehicle_model_ada_mpc_ekf_A

#endif // TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_EKF_A_HPP_
