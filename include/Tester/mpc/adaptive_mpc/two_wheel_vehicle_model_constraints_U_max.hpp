#ifndef __TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_U_MAX_HPP__
#define __TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_U_MAX_HPP__

#include "python_numpy.hpp"

namespace two_wheel_vehicle_model_constraints_U_max {

using namespace PythonNumpy;

using type = DenseMatrix_Type<float, 2, 1>;

inline auto make(void) -> type {

  return make_DenseMatrix<2, 1>(static_cast<float>(1.0),
                                static_cast<float>(1.0));
}

} // namespace two_wheel_vehicle_model_constraints_U_max

#endif // __TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_U_MAX_HPP__
