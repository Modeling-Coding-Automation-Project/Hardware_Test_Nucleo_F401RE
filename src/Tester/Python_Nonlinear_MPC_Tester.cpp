#include "Python_Nonlinear_MPC_Tester.hpp"

Python_Nonlinear_MPC_Tester::Python_Nonlinear_MPC_Tester() {

  this->_mpc = nonlinear_mpc_namespace::make();

  this->_mpc.set_solver_max_iteration(5);
}

Python_Nonlinear_MPC_Tester::~Python_Nonlinear_MPC_Tester() {}

void Python_Nonlinear_MPC_Tester::test_mpc(void) {
  /* Simulation Setting */
  constexpr double SIMULATION_TIME = 60.0;
  constexpr double DELTA_TIME = 0.1;
  constexpr std::size_t MAX_STEP =
      static_cast<std::size_t>(SIMULATION_TIME / DELTA_TIME) + 1;

  Parameter_Type parameters;

  auto X_initial = this->_mpc.get_X();

  PythonControl::StateSpaceState_Type<float, STATE_SIZE> X = X_initial;
  PythonControl::StateSpaceInput_Type<float, INPUT_SIZE> U;
  PythonControl::StateSpaceOutput_Type<float, OUTPUT_SIZE> Y;
}
