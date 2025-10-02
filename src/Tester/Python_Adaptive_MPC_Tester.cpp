#include "Python_Adaptive_MPC_Tester.hpp"

/* Create Reference */
struct ReferenceSequence {
  std::vector<float> x_sequence;
  std::vector<float> y_sequence;
  std::vector<float> theta_sequence;
  std::vector<float> r_sequence;
  std::vector<float> V_sequence;
};

ReferenceSequence create_reference(const std::vector<float> &time,
                                   float delta_time, float simulation_time) {

  constexpr float PI_LOCAL = 3.14159265358979323846F;
  const float vehicle_speed = 15.0F;
  const float curve_yaw_rate = PI_LOCAL / 5.0F;
  const float curve_timing = 2.0F;
  const float yaw_ref = PI_LOCAL;

  const size_t time_size = time.size();

  ReferenceSequence ref;
  ref.x_sequence.resize(time_size, 0.0F);
  ref.y_sequence.resize(time_size, 0.0F);
  ref.theta_sequence.resize(time_size, 0.0F);
  ref.r_sequence.resize(time_size, 0.0F);
  ref.V_sequence.resize(time_size, 0.0F);

  for (std::size_t i = 0; i < time_size; ++i) {
    if (time[i] < curve_timing) {
      if (i > 0) {
        ref.x_sequence[i] = ref.x_sequence[i - 1] + vehicle_speed * delta_time;
      } else {
        ref.x_sequence[i] = vehicle_speed * delta_time;
      }
      ref.y_sequence[i] = 0.0F;
      ref.theta_sequence[i] = 0.0F;
      ref.r_sequence[i] = 0.0F;
      ref.V_sequence[i] = vehicle_speed;

    } else if (time[i] > curve_timing &&
               (i == 0 || ref.theta_sequence[i - 1] < yaw_ref)) {

      float prev_theta = (i > 0) ? ref.theta_sequence[i - 1] : 0.0F;
      float prev_x = (i > 0) ? ref.x_sequence[i - 1] : 0.0F;
      float prev_y = (i > 0) ? ref.y_sequence[i - 1] : 0.0F;

      ref.x_sequence[i] =
          prev_x + vehicle_speed * delta_time * std::cos(prev_theta);
      ref.y_sequence[i] =
          prev_y + vehicle_speed * delta_time * std::sin(prev_theta);
      ref.theta_sequence[i] = prev_theta + curve_yaw_rate * delta_time;

      if (ref.theta_sequence[i] > yaw_ref) {
        ref.theta_sequence[i] = yaw_ref;
      }

      ref.r_sequence[i] = curve_yaw_rate;
      ref.V_sequence[i] = vehicle_speed;

    } else {
      float prev_theta = (i > 0) ? ref.theta_sequence[i - 1] : 0.0F;
      float prev_x = (i > 0) ? ref.x_sequence[i - 1] : 0.0F;
      float prev_y = (i > 0) ? ref.y_sequence[i - 1] : 0.0F;

      ref.x_sequence[i] =
          prev_x + vehicle_speed * delta_time * std::cos(prev_theta);
      ref.y_sequence[i] =
          prev_y + vehicle_speed * delta_time * std::sin(prev_theta);
      ref.theta_sequence[i] = prev_theta;

      ref.r_sequence[i] = 0.0F;
      ref.V_sequence[i] = vehicle_speed;
    }
  }

  return ref;
}

Python_Adaptive_MPC_Tester::Python_Adaptive_MPC_Tester() {

  this->_mpc = ada_mpc_namespace::make();
}

Python_Adaptive_MPC_Tester::~Python_Adaptive_MPC_Tester() {}

void Python_Adaptive_MPC_Tester::test_mpc(void) {
  /* Simulation Setting */
  constexpr float SIMULATION_TIME = 5.0F;
  constexpr float DELTA_TIME = 0.01F;
  constexpr std::size_t MAX_STEP =
      static_cast<std::size_t>(SIMULATION_TIME / DELTA_TIME);

  std::vector<float> time = std::vector<float>(MAX_STEP, 0.0F);
  for (std::size_t i = 0; i < MAX_STEP; ++i) {
    time[i] = i * DELTA_TIME;
  }

  /* Define MPC */
  constexpr std::size_t STATE_SIZE = ada_mpc_namespace::STATE_SIZE;
  constexpr std::size_t INPUT_SIZE = ada_mpc_namespace::INPUT_SIZE;
  constexpr std::size_t OUTPUT_SIZE = ada_mpc_namespace::OUTPUT_SIZE;

  Parameter_Type parameters;

  StateSpaceState_Type<float, STATE_SIZE> X;
  X.template set<0, 0>(static_cast<float>(0.0));
  X.template set<1, 0>(static_cast<float>(0.0));
  X.template set<2, 0>(static_cast<float>(0.0));
  X.template set<3, 0>(static_cast<float>(0.0));
  X.template set<4, 0>(static_cast<float>(0.0));
  X.template set<5, 0>(static_cast<float>(10.0));

  StateSpaceInput_Type<float, INPUT_SIZE> U;
  StateSpaceOutput_Type<float, OUTPUT_SIZE> Y;

  ada_mpc_namespace::Ref_Type ref;
  ReferenceSequence reference_sequence =
      create_reference(time, DELTA_TIME, SIMULATION_TIME);

  /* Simulation */
  unsigned long time_start[MAX_STEP] = {0};
  unsigned long time_end[MAX_STEP] = {0};

  std::array<PythonControl::StateSpaceOutput_Type<
                 float, Python_Adaptive_MPC_Tester::OUTPUT_SIZE>,
             MAX_STEP>
      y_array;
  std::array<PythonControl::StateSpaceInput_Type<
                 float, Python_Adaptive_MPC_Tester::INPUT_SIZE>,
             MAX_STEP>
      u_array;

  for (std::size_t sim_step = 0; sim_step < MAX_STEP; ++sim_step) {
    /* system response */
    X = state_function::function(X, U, parameters);
    Y = measurement_function::function(X, parameters);

    /* controller */
    ref(0, 0) = reference_sequence.x_sequence[sim_step];
    ref(1, 0) = reference_sequence.y_sequence[sim_step];
    ref(2, 0) = reference_sequence.theta_sequence[sim_step];
    ref(3, 0) = reference_sequence.r_sequence[sim_step];
    ref(4, 0) = reference_sequence.V_sequence[sim_step];

    time_start[sim_step] = micros(); // start measuring.

    U = this->_mpc.update_manipulation(ref, Y);

    time_end[sim_step] = micros(); // end measuring.

    /* store result */
    y_array[sim_step] = Y;
    u_array[sim_step] = U;
  }

  /* send result */
  Serial.begin(9600);

  delay(5000);

  Serial.println("Result: \n");
  String result_text;
  result_text.reserve(256);

  result_text += "Y[0], Y[1], U[0], U[1], Calculation time[us]\n";

  for (std::size_t i = 0; i < MAX_STEP; i++) {
    result_text += String(y_array[i](0, 0), 7);
    result_text += ", ";
    result_text += String(y_array[i](1, 0), 7);
    result_text += ", ";
    result_text += String(u_array[i](0, 0), 7);
    result_text += ", ";
    result_text += String(u_array[i](1, 0), 7);
    result_text += ", ";
    result_text += String(time_end[i] - time_start[i]);
    result_text += "\n";
  }

  result_text += "\n";

  Serial.println(result_text);
}
