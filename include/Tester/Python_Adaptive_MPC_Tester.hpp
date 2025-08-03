#ifndef __PYTHON_ADAPTIVE_MPC_TESTER_HPP__
#define __PYTHON_ADAPTIVE_MPC_TESTER_HPP__

#include "python_mpc.hpp"
#include "two_wheel_vehicle_model_ada_mpc.hpp"
#include "two_wheel_vehicle_model_ada_mpc_ekf_parameter.hpp"

#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdint.h>

#include <Arduino.h>

namespace ada_mpc_namespace = two_wheel_vehicle_model_ada_mpc;
namespace state_function = two_wheel_vehicle_model_ada_mpc_ekf_state_function;
namespace measurement_function =
    two_wheel_vehicle_model_ada_mpc_ekf_measurement_function;

using Parameter_Type = two_wheel_vehicle_model_ada_mpc_ekf_parameter::Parameter;
using Ref_Type = two_wheel_vehicle_model_ada_mpc::Ref_Type;
using Tester_MPC_Type = two_wheel_vehicle_model_ada_mpc::type;

class Python_Adaptive_MPC_Tester {
public:
  /* Constant, Type */
  static constexpr std::size_t INPUT_SIZE = ada_mpc_namespace::INPUT_SIZE;
  static constexpr std::size_t STATE_SIZE = ada_mpc_namespace::STATE_SIZE;
  static constexpr std::size_t OUTPUT_SIZE = ada_mpc_namespace::OUTPUT_SIZE;

public:
  /* Constructor */
  Python_Adaptive_MPC_Tester();

  /* Destructor */
  ~Python_Adaptive_MPC_Tester();

public:
  /* Functions */
  void test_mpc(void);

private:
  /* Variables */
  Tester_MPC_Type _mpc;
};

#endif // __PYTHON_ADAPTIVE_MPC_TESTER_HPP__
