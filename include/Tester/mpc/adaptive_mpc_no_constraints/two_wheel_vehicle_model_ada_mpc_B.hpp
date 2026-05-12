#ifndef TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_B_HPP_
#define TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_B_HPP_

#include "python_numpy.hpp"

namespace two_wheel_vehicle_model_ada_mpc_B {

using namespace PythonNumpy;

using SparseAvailable_ada_mpc_B =
    SparseAvailable<ColumnAvailable<false, false>,
                    ColumnAvailable<false, false>,
                    ColumnAvailable<false, false>, ColumnAvailable<true, false>,
                    ColumnAvailable<true, false>, ColumnAvailable<false, true>>;

using type = SparseMatrix_Type<float, SparseAvailable_ada_mpc_B>;

inline auto make(void) -> type {

  return make_SparseMatrix<SparseAvailable_ada_mpc_B>(static_cast<float>(1.0),
                                                      static_cast<float>(1.0),
                                                      static_cast<float>(1.0));
}

} // namespace two_wheel_vehicle_model_ada_mpc_B

#endif // TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_B_HPP_
