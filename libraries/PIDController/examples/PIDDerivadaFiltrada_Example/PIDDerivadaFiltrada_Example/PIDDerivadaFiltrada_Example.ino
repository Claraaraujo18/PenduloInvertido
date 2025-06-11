#include <PIDDerivadaFiltrada.h>

PIDDerivadaFiltrada pid(1.0, 0.5, 0.1, 0.1, 5.0);  // Kp, Ki, Kd, dt, N

float setpoint = 100.0;
float measured = 0.0;
float output = 0.0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  output = pid.compute(setpoint, measured);
  measured += output * 0.1;
  Serial.print("Output: ");
  Serial.print(output);
  Serial.print("\tMeasured: ");
  Serial.println(measured);
  delay(100);
}
