#ifndef TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_ADA_MPC_WEIGHT_U_NC_HPP_
#define TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_ADA_MPC_WEIGHT_U_NC_HPP_

#include "python_numpy.hpp"

namespace two_wheel_vehicle_model_constraints_ada_mpc_Weight_U_Nc {

using namespace PythonNumpy;

using type = DiagMatrix_Type<float, 2>;

inline auto make(void) -> type {

  auto result = make_DiagMatrixZeros<float, 2>();

  result.unsafe_access(0) = static_cast<float>(0.1);
  result.unsafe_access(1) = static_cast<float>(0.1);

  return result;
}

} // namespace two_wheel_vehicle_model_constraints_ada_mpc_Weight_U_Nc

#endif // TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_ADA_MPC_WEIGHT_U_NC_HPP_
