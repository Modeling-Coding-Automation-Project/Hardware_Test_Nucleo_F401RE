#ifndef PYTHON_ADAPTIVE_MPC_TESTER_HPP_
#define PYTHON_ADAPTIVE_MPC_TESTER_HPP_

#define ADA_MPC_USE_CONSTRAINTS (1)

#include "python_mpc.hpp"

#if ADA_MPC_USE_CONSTRAINTS == 0
#include "two_wheel_vehicle_model_ada_mpc.hpp"
#include "two_wheel_vehicle_model_ada_mpc_ekf_parameter.hpp"
#else
#include "two_wheel_vehicle_model_constraints_ada_mpc.hpp"
#include "two_wheel_vehicle_model_constraints_ada_mpc_ekf_parameter.hpp"
#endif // ADA_MPC_USE_CONSTRAINTS

#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdint.h>

#include <Arduino.h>

#if ADA_MPC_USE_CONSTRAINTS == 0

namespace ada_mpc_namespace = two_wheel_vehicle_model_ada_mpc;
namespace state_equation = two_wheel_vehicle_model_ada_mpc_ekf_state_equation;
namespace measurement_equation =
    two_wheel_vehicle_model_ada_mpc_ekf_measurement_equation;

using Parameter_Type = two_wheel_vehicle_model_ada_mpc_ekf_parameter::Parameter;
using Reference_Type = two_wheel_vehicle_model_ada_mpc::Reference_Type;
using Tester_MPC_Type = two_wheel_vehicle_model_ada_mpc::type;

#else

namespace ada_mpc_namespace = two_wheel_vehicle_model_constraints_ada_mpc;
namespace state_equation =
    two_wheel_vehicle_model_constraints_ada_mpc_ekf_state_equation;
namespace measurement_equation =
    two_wheel_vehicle_model_constraints_ada_mpc_ekf_measurement_equation;

using Parameter_Type =
    two_wheel_vehicle_model_constraints_ada_mpc_ekf_parameter::Parameter;
using Reference_Type =
    two_wheel_vehicle_model_constraints_ada_mpc::Reference_Type;
using Tester_MPC_Type = two_wheel_vehicle_model_constraints_ada_mpc::type;

#endif // ADA_MPC_USE_CONSTRAINTS

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

#endif // PYTHON_ADAPTIVE_MPC_TESTER_HPP_
