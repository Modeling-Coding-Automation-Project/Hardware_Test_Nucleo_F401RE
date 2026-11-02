#ifndef PYTHON_LTI_MPC_INSTANT_TESTER_HPP_
#define PYTHON_LTI_MPC_INSTANT_TESTER_HPP_

#include "servo_motor_constraints_instant_MPC_impc.hpp"

#include <array>

#include <Arduino.h>

class Python_LTI_MPC_Instant_Tester {
public:
  /* Constant, Type */
  static constexpr std::size_t INPUT_SIZE =
      servo_motor_constraints_instant_MPC_impc::INPUT_SIZE;
  static constexpr std::size_t STATE_SIZE =
      servo_motor_constraints_instant_MPC_impc::STATE_SIZE;
  static constexpr std::size_t OUTPUT_SIZE =
      servo_motor_constraints_instant_MPC_impc::OUTPUT_SIZE;

  static constexpr std::size_t SIM_STEP_MAX = 100;

  using Tester_MPC_Type = servo_motor_constraints_instant_MPC_impc::type;

public:
  /* Constructor */
  Python_LTI_MPC_Instant_Tester();

  /* Functions */
  void test_mpc(void);

private:
  /* Variables */
  Tester_MPC_Type _mpc;
};

#endif // PYTHON_LTI_MPC_INSTANT_TESTER_HPP_
