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
