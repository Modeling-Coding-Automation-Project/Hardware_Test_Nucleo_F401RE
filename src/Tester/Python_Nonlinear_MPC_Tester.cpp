#include "Python_Nonlinear_MPC_Tester.hpp"

Python_Nonlinear_MPC_Tester::Python_Nonlinear_MPC_Tester() {

  this->_mpc = nonlinear_mpc_namespace::make();

  this->_mpc.set_solver_max_iteration(5);
}

Python_Nonlinear_MPC_Tester::~Python_Nonlinear_MPC_Tester() {}

void Python_Nonlinear_MPC_Tester::test_mpc(void) {
  /* Simulation Setting */
  constexpr double SIMULATION_TIME = 20.0;
  constexpr double DELTA_TIME = 0.1;
  constexpr std::size_t MAX_STEP =
      static_cast<std::size_t>(SIMULATION_TIME / DELTA_TIME) + 1;

  /* Define MPC */
  constexpr std::size_t STATE_SIZE = Python_Nonlinear_MPC_Tester::STATE_SIZE;
  constexpr std::size_t INPUT_SIZE = Python_Nonlinear_MPC_Tester::INPUT_SIZE;
  constexpr std::size_t OUTPUT_SIZE = Python_Nonlinear_MPC_Tester::OUTPUT_SIZE;

  constexpr std::size_t NP = Python_Nonlinear_MPC_Tester::NP;

  Parameter_Type parameters;

  auto X_initial = this->_mpc.get_X();

  PythonControl::StateSpaceState_Type<float, STATE_SIZE> X = X_initial;
  PythonControl::StateSpaceInput_Type<float, INPUT_SIZE> U;
  PythonControl::StateSpaceOutput_Type<float, OUTPUT_SIZE> Y;
  ReferenceTrajectory_Type reference_trajectory;

  /* Simulation */
  unsigned long time_start[MAX_STEP] = {0};
  unsigned long time_end[MAX_STEP] = {0};

  std::array<PythonControl::StateSpaceOutput_Type<float, OUTPUT_SIZE>, MAX_STEP>
      y_array;
  std::array<PythonControl::StateSpaceInput_Type<float, INPUT_SIZE>, MAX_STEP>
      u_array;

  std::size_t reference_index = 0;

  for (std::size_t sim_step = 0; sim_step < MAX_STEP; ++sim_step) {
    /* system response */
    X = state_function::function(X, U, parameters);
    Y = measurement_function::function(X, parameters);

    /* controller */
    for (std::size_t j = 0; j < NP; ++j) {
      for (std::size_t i = 0; i < OUTPUT_SIZE; ++i) {

        reference_trajectory(i, j) =
            nonlinear_mpc_reference_path::reference_path_data[reference_index +
                                                              i +
                                                              j * OUTPUT_SIZE];
      }
    }
    reference_index += OUTPUT_SIZE;

    time_start[sim_step] = micros(); // start measuring.

    U = this->_mpc.update_manipulation(reference_trajectory, Y);

    time_end[sim_step] = micros(); // end measuring.

    /* store result */
    y_array[sim_step] = Y;
    u_array[sim_step] = U;
  }
}
