#ifndef __PYTHON_LTV_MPC_TESTER_HPP__
#define __PYTHON_LTV_MPC_TESTER_HPP__

#include "python_mpc.hpp"
#include "servo_motor_ltv_constraints_ltv_mpc.hpp"
#include "servo_motor_ltv_ltv_mpc.hpp"
#include "servo_motor_ltv_parameters.hpp"

#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdint.h>

#include <Arduino.h>

#define LTI_MPC_USE_CONSTRAINTS (1)

#if LTI_MPC_USE_CONSTRAINTS == 0

namespace ltv_mpc_namespace = servo_motor_ltv_ltv_mpc;
using MPC_StateSpace_Updater =
    servo_motor_mpc_state_space_updater::MPC_StateSpace_Updater;
using Parameter_Type = servo_motor_ltv_parameters::Parameter;
using Ref_Type = servo_motor_ltv_ltv_mpc::Ref_Type;
using Tester_MPC_Type = ltv_mpc_namespace::type;

#else

namespace ltv_mpc_namespace = servo_motor_ltv_constraints_ltv_mpc;
using MPC_StateSpace_Updater =
    servo_motor_ltv_constraints_mpc_state_space_updater::MPC_StateSpace_Updater;
using Parameter_Type = servo_motor_ltv_constraints_parameters::Parameter;
using Ref_Type = servo_motor_ltv_constraints_ltv_mpc::Ref_Type;
using Tester_MPC_Type = servo_motor_ltv_constraints_ltv_mpc::type;

#endif // LTI_MPC_USE_CONSTRAINTS == 0

class Python_LTV_MPC_Tester {
public:
  /* Constant, Type */
  static constexpr std::size_t INPUT_SIZE = ltv_mpc_namespace::INPUT_SIZE;
  static constexpr std::size_t STATE_SIZE = ltv_mpc_namespace::STATE_SIZE;
  static constexpr std::size_t OUTPUT_SIZE = ltv_mpc_namespace::OUTPUT_SIZE;

  static constexpr std::size_t SIM_STEP_MAX = 100;

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
