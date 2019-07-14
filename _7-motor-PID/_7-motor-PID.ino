#include <Ultrasonic.h>
#include <SimpleKalmanFilter.h>

#include <FastPID.h>

#define PIN_INPUT     A0
#define PIN_SETPOINT  A1
#define PIN_OUTPUT    5

float Kp = 0.2, Ki = 0, Kd = 0, Hz = 100;
int output_bits = 9;
bool output_signed = true;

FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);

Ultrasonic ultrasonic(8, 9);
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);

int IN1 = 6;
int IN2 = 5;

void setup()
{
  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
}

void loop()
{
  int a = analogRead(A0);
  int distance_unfiltered = ultrasonic.read();
  float distance = simpleKalmanFilter.updateEstimate(distance_unfiltered);

  int setpoint = 10;
  int feedback = distance;
  int output = myPID.step(setpoint, feedback);

  if (output < 0)
  {
    output = -output;
    output += 130;
    analogWrite(IN1, output);
    analogWrite(IN2, LOW);
  }
  else
  {
    output += 130;
    analogWrite(IN1, LOW);
    analogWrite(IN2, output);
  }
  
  Serial.print(" sp: ");
  Serial.print(setpoint);
  Serial.print(" fb: ");
  Serial.print(feedback);
  Serial.print(" out: ");
  Serial.println(output);
}
