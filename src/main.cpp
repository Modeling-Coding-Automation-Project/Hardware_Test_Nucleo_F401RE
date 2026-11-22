#ifdef HARDWARE_TEST_LTI_MPC
#include "Python_LTI_MPC_Tester.hpp"
static Python_LTI_MPC_Tester python_mpc_tester;
#elif defined(HARDWARE_TEST_LTI_MPC_INSTANT)
#include "Python_LTI_MPC_Instant_Tester.hpp"
static Python_LTI_MPC_Instant_Tester python_mpc_tester;
#elif defined(HARDWARE_TEST_LTV_MPC)
#include "Python_LTV_MPC_Tester.hpp"
static Python_LTV_MPC_Tester python_mpc_tester;
#elif defined(HARDWARE_TEST_ADAPTIVE_MPC)
#include "Python_Adaptive_MPC_Tester.hpp"
static Python_Adaptive_MPC_Tester python_mpc_tester;
#elif defined(HARDWARE_TEST_NONLINEAR_MPC_TWICE_DIFFERENTIABLE)
#include "Python_Nonlinear_MPC_Twice_Differentiable_Tester.hpp"
static Python_Nonlinear_MPC_Twice_Differentiable_Tester python_mpc_tester;
#elif defined(HARDWARE_TEST_NONLINEAR_MPC_OPTIMIZATION_ENGINE)
#include "Python_Nonlinear_MPC_Optimization_Engine_Tester.hpp"
static Python_Nonlinear_MPC_Optimization_Engine_Tester python_mpc_tester;
#else
#error "Select a hardware test in platformio.ini."
#endif

#include "main.hpp"

void setup() {
  python_mpc_tester.test_mpc();

  pinMode(LED_BUILTIN, OUTPUT);
  delay(100);
}

void loop() {

  digitalWrite(LED_BUILTIN, OUTPUT);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}
