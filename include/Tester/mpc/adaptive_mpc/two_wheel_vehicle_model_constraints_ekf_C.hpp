#ifndef __TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_EKF_C_HPP__
#define __TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_EKF_C_HPP__

#include "python_numpy.hpp"

namespace two_wheel_vehicle_model_constraints_ekf_C {

using namespace PythonNumpy;

using SparseAvailable_ekf_C =
    SparseAvailable<ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<false, true, false, false, false, false>,
                    ColumnAvailable<false, false, true, false, false, false>,
                    ColumnAvailable<false, false, false, true, false, false>,
                    ColumnAvailable<false, false, false, false, false, true>>;

using type = SparseMatrix_Type<float, SparseAvailable_ekf_C>;

inline auto make(void) -> type {

  return make_SparseMatrix<SparseAvailable_ekf_C>(
      static_cast<float>(1.0), static_cast<float>(1.0), static_cast<float>(1.0),
      static_cast<float>(1.0), static_cast<float>(1.0));
}

} // namespace two_wheel_vehicle_model_constraints_ekf_C

#endif // __TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_EKF_C_HPP__
