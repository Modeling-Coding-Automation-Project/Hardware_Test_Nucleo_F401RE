#ifndef __KINEMATIC_BICYCLE_MODEL_OP_EN_OE_MEASUREMENT_FUNCTION_HPP__
#define __KINEMATIC_BICYCLE_MODEL_OP_EN_OE_MEASUREMENT_FUNCTION_HPP__

#include "python_math.hpp"

namespace kinematic_bicycle_model_op_en_oe_measurement_function {

using namespace PythonMath;

template <typename X_Type, typename U_Type, typename Parameter_Type, typename Y_Type>
class Function {
public:
static inline auto sympy_function(const double q3, const double px, const double py, const double q0) -> Y_Type {

    Y_Type result;

    result.template set<0, 0>(static_cast<double>(px));
    result.template set<1, 0>(static_cast<double>(py));
    result.template set<2, 0>(static_cast<double>(q0));
    result.template set<3, 0>(static_cast<double>(q3));
    
    return result;
}

static inline auto function(const X_Type X, const U_Type U, const Parameter_Type Parameters) -> Y_Type {

    double px = X.template get<0, 0>();

    double py = X.template get<1, 0>();

    double q0 = X.template get<2, 0>();

    double q3 = X.template get<3, 0>();

    return sympy_function(q3, px, py, q0);
}

};

} // namespace kinematic_bicycle_model_op_en_oe_measurement_function

#endif // __KINEMATIC_BICYCLE_MODEL_OP_EN_OE_MEASUREMENT_FUNCTION_HPP__
