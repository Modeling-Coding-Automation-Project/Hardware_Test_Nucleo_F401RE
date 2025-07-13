#ifndef __SERVO_MOTOR_LTV_CONSTRAINTS_PARAMETERS_HPP__
#define __SERVO_MOTOR_LTV_CONSTRAINTS_PARAMETERS_HPP__

namespace servo_motor_ltv_constraints_parameters {

class Parameter {
public:
  double Lshaft = static_cast<double>(1.0);
  double dshaft = static_cast<double>(0.02);
  double shaftrho = static_cast<double>(7850.0);
  double G = static_cast<double>(81500000000.0);
  double Mmotor = static_cast<double>(100.0);
  double Rmotor = static_cast<double>(0.1);
  double Bmotor = static_cast<double>(0.1);
  double R = static_cast<double>(20.0);
  double Kt = static_cast<double>(10.0);
  double Bload = static_cast<double>(25.0);
};

using Parameter_Type = Parameter;

} // namespace servo_motor_ltv_constraints_parameters

#endif // __SERVO_MOTOR_LTV_CONSTRAINTS_PARAMETERS_HPP__
