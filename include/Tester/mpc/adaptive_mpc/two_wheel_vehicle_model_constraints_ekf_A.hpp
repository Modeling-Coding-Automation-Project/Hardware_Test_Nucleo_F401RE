#ifndef __TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_EKF_A_HPP__
#define __TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_EKF_A_HPP__

#include "python_numpy.hpp"

namespace two_wheel_vehicle_model_constraints_ekf_A {

using namespace PythonNumpy;

using SparseAvailable_ekf_A =
    SparseAvailable<ColumnAvailable<true, false, true, false, false, true>,
                    ColumnAvailable<false, true, true, false, false, true>,
                    ColumnAvailable<false, false, true, true, false, false>,
                    ColumnAvailable<false, false, false, true, true, true>,
                    ColumnAvailable<false, false, false, true, true, true>,
                    ColumnAvailable<false, false, false, false, false, true>>;

using type = SparseMatrix_Type<float, SparseAvailable_ekf_A>;

inline auto make(void) -> type {

  return make_SparseMatrix<SparseAvailable_ekf_A>(
      static_cast<float>(1.0), static_cast<float>(1.0), static_cast<float>(1.0),
      static_cast<float>(1.0), static_cast<float>(1.0), static_cast<float>(1.0),
      static_cast<float>(1.0), static_cast<float>(1.0), static_cast<float>(1.0),
      static_cast<float>(1.0), static_cast<float>(1.0), static_cast<float>(1.0),
      static_cast<float>(1.0), static_cast<float>(1.0),
      static_cast<float>(1.0));
}

} // namespace two_wheel_vehicle_model_constraints_ekf_A

#endif // __TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_EKF_A_HPP__
