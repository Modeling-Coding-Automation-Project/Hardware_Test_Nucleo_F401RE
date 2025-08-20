#ifndef __TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_Y_MAX_HPP__
#define __TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_Y_MAX_HPP__

#include "python_numpy.hpp"

namespace two_wheel_vehicle_model_constraints_Y_max {

using namespace PythonNumpy;

using SparseAvailable_Y_max =
    SparseAvailable<ColumnAvailable<false>, ColumnAvailable<false>,
                    ColumnAvailable<false>, ColumnAvailable<false>,
                    ColumnAvailable<false>>;

using type = SparseMatrix_Type<float, SparseAvailable_Y_max>;

inline auto make(void) -> type { return make_SparseMatrixEmpty<float, 5, 1>(); }

} // namespace two_wheel_vehicle_model_constraints_Y_max

#endif // __TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_Y_MAX_HPP__
