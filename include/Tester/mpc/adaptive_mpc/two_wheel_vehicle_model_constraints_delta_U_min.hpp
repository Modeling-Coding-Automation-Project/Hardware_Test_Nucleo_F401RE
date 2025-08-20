#ifndef __TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_DELTA_U_MIN_HPP__
#define __TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_DELTA_U_MIN_HPP__

#include "python_numpy.hpp"

namespace two_wheel_vehicle_model_constraints_delta_U_min {

using namespace PythonNumpy;

using SparseAvailable_delta_U_min =
    SparseAvailable<ColumnAvailable<false>, ColumnAvailable<false>>;

using type = SparseMatrix_Type<float, SparseAvailable_delta_U_min>;

inline auto make(void) -> type { return make_SparseMatrixEmpty<float, 2, 1>(); }

} // namespace two_wheel_vehicle_model_constraints_delta_U_min

#endif // __TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_DELTA_U_MIN_HPP__
