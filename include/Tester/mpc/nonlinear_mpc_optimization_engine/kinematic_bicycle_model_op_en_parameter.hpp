#ifndef __KINEMATIC_BICYCLE_MODEL_OP_EN_PARAMETER_HPP__
#define __KINEMATIC_BICYCLE_MODEL_OP_EN_PARAMETER_HPP__

namespace kinematic_bicycle_model_op_en_parameter {

class Parameter {
public:
  double wheel_base = static_cast<double>(2.8);
  double delta_time = static_cast<double>(0.1);
};

} // namespace kinematic_bicycle_model_op_en_parameter

#endif // __KINEMATIC_BICYCLE_MODEL_OP_EN_PARAMETER_HPP__
