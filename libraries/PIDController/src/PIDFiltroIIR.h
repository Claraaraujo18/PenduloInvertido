#ifndef PID_FILTRO_IIR_H
#define PID_FILTRO_IIR_H

class PIDFiltroIIR {
  public:
    PIDFiltroIIR(float kp, float ki, float kd, float dt);
    float compute(float setpoint, float measured);

  private:
    float Kp, Ki, Kd, dt;
    float A0, A1, A2;
    float error[3];
};

#endif
