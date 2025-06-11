#ifndef PID_DERIVADA_FILTRADA_H
#define PID_DERIVADA_FILTRADA_H

class PIDDerivadaFiltrada {
  public:
    PIDDerivadaFiltrada(float kp, float ki, float kd, float dt, float N = 5.0);
    float compute(float setpoint, float measured);

  private:
    float Kp, Ki, Kd, dt, N;
    float A0, A1;
    float A0d, A1d, A2d;
    float alpha_1, alpha_2;
    float error[3];
    float d0, d1;
    float fd0, fd1;
    float output;
};

#endif
