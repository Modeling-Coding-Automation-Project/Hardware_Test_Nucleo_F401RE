#ifndef __PYTHON_LTV_MPC_TESTER_HPP__
#define __PYTHON_LTV_MPC_TESTER_HPP__

#include "python_mpc.hpp"
#include "servo_motor_ltv_ltv_mpc.hpp"
#include "servo_motor_ltv_parameters.hpp"

#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdint.h>

#include <Arduino.h>

#define LTI_MPC_USE_CONSTRAINTS (0)

class Python_LTV_MPC_Tester {
public:
  /* Constant, Type */
  static constexpr std::size_t INPUT_SIZE = servo_motor_ltv_ltv_mpc::INPUT_SIZE;
  static constexpr std::size_t STATE_SIZE = servo_motor_ltv_ltv_mpc::STATE_SIZE;
  static constexpr std::size_t OUTPUT_SIZE =
      servo_motor_ltv_ltv_mpc::OUTPUT_SIZE;

  static constexpr std::size_t SIM_STEP_MAX = 100;

#if LTI_MPC_USE_CONSTRAINTS == 0
  using Tester_MPC_Type = servo_motor_ltv_ltv_mpc::type;
#else
#endif // LTI_MPC_USE_CONSTRAINTS == 0

public:
  /* Constructor */
  Python_LTV_MPC_Tester();

  /* Destructor */
  ~Python_LTV_MPC_Tester();

public:
  /* Functions */
  void test_mpc(void);

private:
  /* Variables */
  Tester_MPC_Type _mpc;
};

#endif // __PYTHON_LTV_MPC_TESTER_HPP__
