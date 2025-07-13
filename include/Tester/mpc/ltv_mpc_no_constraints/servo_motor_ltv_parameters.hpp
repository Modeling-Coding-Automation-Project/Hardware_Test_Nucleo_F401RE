#ifndef __SERVO_MOTOR_LTV_PARAMETERS_HPP__
#define __SERVO_MOTOR_LTV_PARAMETERS_HPP__

namespace servo_motor_ltv_parameters {

class Parameter {
public:
  float Lshaft = static_cast<float>(1.0);
  float dshaft = static_cast<float>(0.02);
  float shaftrho = static_cast<float>(7850.0);
  float G = static_cast<float>(81500000000.0);
  float Mmotor = static_cast<float>(100.0);
  float Rmotor = static_cast<float>(0.1);
  float Bmotor = static_cast<float>(0.1);
  float R = static_cast<float>(20.0);
  float Kt = static_cast<float>(10.0);
  float Bload = static_cast<float>(25.0);
};

using Parameter_Type = Parameter;

} // namespace servo_motor_ltv_parameters

#endif // __SERVO_MOTOR_LTV_PARAMETERS_HPP__
