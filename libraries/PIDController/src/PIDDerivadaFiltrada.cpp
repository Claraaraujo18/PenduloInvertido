#include "PIDDerivadaFiltrada.h"

PIDDerivadaFiltrada::PIDDerivadaFiltrada(float kp, float ki, float kd, float dt, float N)
  : Kp(kp), Ki(ki), Kd(kd), dt(dt), N(N), d0(0.0), d1(0.0), fd0(0.0), fd1(0.0), output(0.0) {
  A0 = Kp + Ki * dt;
  A1 = -Kp;

  A0d = Kd / dt;
  A1d = -2.0 * Kd / dt;
  A2d = Kd / dt;

  float tau = Kd / (Kp * N);
  float alpha = dt / (2.0 * tau);
  alpha_1 = alpha / (alpha + 1.0);
  alpha_2 = (alpha - 1.0) / (alpha + 1.0);

  error[0] = error[1] = error[2] = 0.0;
}

float PIDDerivadaFiltrada::compute(float setpoint, float measured) {
  error[2] = error[1];
  error[1] = error[0];
  error[0] = setpoint - measured;

  output += A0 * error[0] + A1 * error[1];

  d1 = d0;
  d0 = A0d * error[0] + A1d * error[1] + A2d * error[2];

  fd1 = fd0;
  fd0 = alpha_1 * (d0 + d1) - alpha_2 * fd1;

  output += fd0;

  return output;
}
