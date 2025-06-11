#include <PIDFiltroIIR.h>

PIDFiltroIIR pid(2.0, 1.0, 0.1, 0.1);  // Kp, Ki, Kd, dt

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

