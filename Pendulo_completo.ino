#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "PIDDerivadaFiltrada.h"

//Pines a los que se conecta el tb6612fng 
const int pinPWMA = 4;
const int pinAIN2 = 16;
const int pinAIN1 = 17;
const int pinBIN1 = 5;
const int pinBIN2 = 18;
const int pinPWMB = 19;

//Definimos el MPU-6050
MPU6050 sensor;

int ax, ay, az;
int gx, gy, gz;
float ang_x = 0.0, ang_x_prev = 0.0;
long tiempo_prev = 0;
float dt;

//Creamos el Controlador PID
float Kp = 30.0;     //Ya que no lo pude probar en el péndulo real los valores no son correctos
float Ki = 0.0;      //Ya que no lo pude probar en el péndulo real los valores no son correctos
float Kd = 1.5;      //Ya que no lo pude probar en el péndulo real los valores no son correctos
float N = 5.0;       
float setpoint = 0.0;
PIDDerivadaFiltrada pid(Kp, Ki, Kd, 0.01, N);


void setup() {
  Serial.begin(57600);
  Wire.begin(14, 12);  //Definimos los pines del ESP32 en los que va conectado el MPU-6050

  //Comprobamos que el MPU-6050 comienza a funcionar
  sensor.initialize();
  if (!sensor.testConnection()) {
    Serial.println("Error al conectar el MPU6050");
    while (1);
  } else {
    Serial.println("MPU6050 conectado.");
  }

  //Inicializamos motores
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinAIN2, OUTPUT);
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);
  pinMode(pinPWMB, OUTPUT);

}


void loop() {
  // 1. Leer sensores
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  // 2. Calcular ángulo con filtro complementario
  float accel_ang_x = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
  ang_x = 0.98 * (ang_x_prev + (gx / 131.0) * dt) + 0.02 * accel_ang_x;
  ang_x_prev = ang_x;

  // 3. Calcular PID
  float control = pid.compute(setpoint, ang_x);

  // 4. Limitar salida ya que los motores solo le podemos dar valor de velocidad entre 0 y 255
  control = constrain(control, -255, 255);

  // 5. Mover motores según control
  mover(control);

  // 6. Imprimir info para depurar el código
  Serial.print("Ángulo X: ");
  Serial.print(ang_x);
  Serial.print(" | Control: ");
  Serial.println(control);

  delay(10); 
}

// --------------------- Movimiento motores ---------------------
void mover(float control) {
  int velocidad = abs((int)control);
  velocidad = constrain(velocidad, 0, 255);  

  if (control > 0) {
    // Caído hacia atrás → empujar hacia adelante
    moverAdelante(velocidad);
  } else if (control < 0) {
    // Caído hacia adelante → empujar hacia atrás
    moverAtras(velocidad);
  }else {
    parar();
  }
}

void moverAdelante(int vel) {
  digitalWrite(pinAIN1, HIGH);
  digitalWrite(pinAIN2, LOW);
  analogWrite(pinPWMA, vel);

  digitalWrite(pinBIN1, HIGH);
  digitalWrite(pinBIN2, LOW);
  analogWrite(pinPWMB, vel);
}

void moverAtras(int vel) {
  digitalWrite(pinAIN1, LOW);
  digitalWrite(pinAIN2, HIGH);
  analogWrite(pinPWMA, vel);

  digitalWrite(pinBIN1, LOW);
  digitalWrite(pinBIN2, HIGH);
  analogWrite(pinPWMB, vel);
}

void parar() {
  analogWrite(pinPWMA, 0);
  analogWrite(pinPWMB, 0);
  digitalWrite(pinAIN1, LOW);
  digitalWrite(pinAIN2, LOW);
  digitalWrite(pinBIN1, LOW);
  digitalWrite(pinBIN2, LOW);
}
