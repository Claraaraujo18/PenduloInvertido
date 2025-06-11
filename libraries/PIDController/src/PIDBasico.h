#ifndef PID_BASICO_H
#define PID_BASICO_H

class PIDBasico {
  public:
    PIDBasico(float kp, float ki, float kd, float dt);
    float compute(float setpoint, float measured);

  private:
    float Kp, Ki, Kd, dt;
    float previous_error;
    float integral;
};

#endif
