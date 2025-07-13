#include "Python_LTV_MPC_Tester.hpp"

Python_LTV_MPC_Tester::Python_LTV_MPC_Tester() {

#if LTI_MPC_USE_CONSTRAINTS == 0
  this->_mpc = servo_motor_ltv_ltv_mpc::make();
#else
#endif // LTI_MPC_USE_CONSTRAINTS == 0
}

Python_LTV_MPC_Tester::~Python_LTV_MPC_Tester() {}

void Python_LTV_MPC_Tester::test_mpc(void) {

  /* Define State Space */
  using A_Type = PythonNumpy::SparseAvailable<
      PythonNumpy::ColumnAvailable<true, true, false, false>,
      PythonNumpy::ColumnAvailable<true, true, true, false>,
      PythonNumpy::ColumnAvailable<false, false, true, true>,
      PythonNumpy::ColumnAvailable<true, false, true, true>>;

  auto A = PythonNumpy::make_SparseMatrix<A_Type>(
      static_cast<float>(1.0), static_cast<float>(0.05),
      static_cast<float>(-2.5603853840856576),
      static_cast<float>(0.9500002466138069),
      static_cast<float>(0.12801926920428286), static_cast<float>(1.0),
      static_cast<float>(0.05), static_cast<float>(6.400995031689203),
      static_cast<float>(-0.3200497515844601),
      static_cast<float>(0.4900000000000001));

  using B_Type = PythonNumpy::SparseAvailable<
      PythonNumpy::ColumnAvailable<false>, PythonNumpy::ColumnAvailable<false>,
      PythonNumpy::ColumnAvailable<false>, PythonNumpy::ColumnAvailable<true>>;

  auto B = PythonNumpy::make_SparseMatrix<B_Type>(
      static_cast<float>(0.04999999999999999));

  using C_Type = PythonNumpy::SparseAvailable<
      PythonNumpy::ColumnAvailable<true, false, false, false>,
      PythonNumpy::ColumnAvailable<true, false, true, false>>;

  auto C = PythonNumpy::make_SparseMatrix<C_Type>(
      static_cast<float>(1.0), static_cast<float>(1280.1990063378407),
      static_cast<float>(-64.00995031689203));

  // using D_Type =
  //     PythonNumpy::SparseAvailable<PythonNumpy::ColumnAvailable<false>,
  //                                  PythonNumpy::ColumnAvailable<false>>;

  auto D = PythonNumpy::make_SparseMatrixEmpty<float, 2, 1>();

  float dt = 0.05F;

  auto sys = PythonControl::make_DiscreteStateSpace(A, B, C, D, dt);

  /* Define parameters */
  servo_motor_ltv_parameters::Parameter plant_parameters;
  servo_motor_ltv_parameters::Parameter controller_parameters;

  /* Define reference */
  servo_motor_ltv_ltv_mpc::Ref_Type ref;

  auto U = PythonControl::make_StateSpaceInput<INPUT_SIZE>(0.0F);

  /* State Space Simulation */
  unsigned long time_start[Python_LTV_MPC_Tester::SIM_STEP_MAX] = {0};
  unsigned long time_end[Python_LTV_MPC_Tester::SIM_STEP_MAX] = {0};

  std::array<PythonControl::StateSpaceOutput_Type<
                 float, Python_LTV_MPC_Tester::OUTPUT_SIZE>,
             Python_LTV_MPC_Tester::SIM_STEP_MAX>
      y_array;
  std::array<PythonControl::StateSpaceInput_Type<
                 float, Python_LTV_MPC_Tester::INPUT_SIZE>,
             Python_LTV_MPC_Tester::SIM_STEP_MAX>
      u_array;

  constexpr std::size_t PARAMETER_CHANGE_STEP = 400;
  bool parameter_changed = false;
  constexpr double MPC_UPDATE_STEP = 800;
  bool MPC_updated = false;

  /* Initialize reference */
  for (std::size_t i = 0; i < ref.rows(); ++i) {
    ref(0, i) = 1.0F;
  }

  for (std::size_t sim_step = 0; sim_step < Python_LTV_MPC_Tester::SIM_STEP_MAX;
       ++sim_step) {
    if (!parameter_changed && sim_step >= PARAMETER_CHANGE_STEP) {
      plant_parameters.Mmotor = 250.0;
      mpc_state_space_updater::MPC_StateSpace_Updater::update(plant_parameters,
                                                              sys);
      parameter_changed = true;

      for (std::size_t i = 0; i < ref.rows(); ++i) {
        ref(0, i) = -1.0;
      }
    }

    /* system response */
    sys.update(U);

    /* controller */
    if (!MPC_updated && sim_step >= MPC_UPDATE_STEP) {
      controller_parameters.Mmotor = 250.0;

      MPC_updated = true;

      for (std::size_t i = 0; i < ref.rows(); ++i) {
        ref(0, i) = 1.0;
      }

      time_start[sim_step] = micros(); // start measuring.
      this->_mpc.update_parameters(controller_parameters);
    } else {
      time_start[sim_step] = micros(); // start measuring.
    }

    time_start[sim_step] = micros(); // start measuring.

    U = this->_mpc.update_manipulation(ref, sys.get_Y());

    time_end[sim_step] = micros(); // end measuring.

    /* store result */
    y_array[sim_step] = sys.get_Y();
    u_array[sim_step] = U;
  }

  /* send result */
  Serial.begin(9600);

  delay(5000);

  Serial.println("Result: \n");
  String result_text;
  result_text.reserve(256);

  result_text += "Y[0], Y[1], U[0], Calculation time[us]\n";

  for (std::size_t i = 0; i < Python_LTV_MPC_Tester::SIM_STEP_MAX; i++) {
    result_text += String(y_array[i](0, 0), 7);
    result_text += ", ";
    result_text += String(y_array[i](1, 0), 7);
    result_text += ", ";
    result_text += String(u_array[i](0, 0), 7);
    result_text += ", ";
    result_text += String(time_end[i] - time_start[i]);
    result_text += "\n";
  }

  result_text += "\n";

  Serial.println(result_text);
}
