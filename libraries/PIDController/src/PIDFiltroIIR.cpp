#include "PIDFiltroIIR.h"

PIDFiltroIIR::PIDFiltroIIR(float kp, float ki, float kd, float dt)
  : Kp(kp), Ki(ki), Kd(kd), dt(dt) {
  A0 = Kp + Ki * dt + Kd / dt;
  A1 = -Kp - 2.0 * Kd / dt;
  A2 = Kd / dt;
  error[0] = error[1] = error[2] = 0.0;
}

float PIDFiltroIIR::compute(float setpoint, float measured) {
  error[2] = error[1];
  error[1] = error[0];
  error[0] = setpoint - measured;
  return A0 * error[0] + A1 * error[1] + A2 * error[2];
}
