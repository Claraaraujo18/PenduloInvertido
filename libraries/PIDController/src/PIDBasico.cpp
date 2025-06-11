#include "PIDBasico.h"

PIDBasico::PIDBasico(float kp, float ki, float kd, float dt)
  : Kp(kp), Ki(ki), Kd(kd), dt(dt), previous_error(0.0), integral(0.0) {}

float PIDBasico::compute(float setpoint, float measured) {
  float error = setpoint - measured;
  integral += error * dt;
  float derivative = (error - previous_error) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative;
  previous_error = error;
  return output;
}
