#include <Ultrasonic.h>

#define TIME_SINCE(x) (millis() - (x))
float Kp = 10;
float Ki = 0.01;
float Kd = 0.1;

float error_last = 0;
float error = 0;
float integral = 0;
float derivative = 0;
float output = 0;

int setpoint = 10;

float distance_last = 0;
float distance_f = 0;
int   distance = 0;

int motor_offset = 130;

Ultrasonic ultrasonic(8, 9);

int IN1 = 6;
int IN2 = 5;

float frequency = 1000;
float period    = 1/frequency;
float period_ms = 1000/frequency;


void update_ultrasonic()
{
  int a = ultrasonic.read();
  
  distance_f = 0.1*a + 0.9*distance_last;
  
  distance_last = distance_f;
  distance = round(distance_f);
}

void PID()
{
  update_ultrasonic();
  
  error      = setpoint - distance_f;
  integral   = integral + error * period;
  derivative = (error - error_last) / period;
  output     = Kp * error + Ki * integral + Kd * derivative;
  error_last = error;
}

void setup()
{
  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

}

void loop()
{
  uint32_t timer1 = millis();

  setpoint = analogRead(A0);
  setpoint = map(setpoint, 0, 1023, 5, 20);
  PID();
  
  int out= round(output);
  out = constrain(out, -255, 255);

  if (out == 0)
  {
    analogWrite(IN1, LOW);
    analogWrite(IN2, LOW);
  }
  else if (out < 0)
  {
    out = (-out) + motor_offset;
    out = constrain(out, -255, 255);
    analogWrite(IN1, out);
    analogWrite(IN2, LOW);
  }
  else
  {
    out += motor_offset;
    out =  constrain(out, -255, 255);
    analogWrite(IN1, LOW);
    analogWrite(IN2, out);
  }

  Serial.print(setpoint);
  Serial.print('\t');
  Serial.print(distance_f);
  Serial.print('\t');
  Serial.print(out);
  Serial.print('\t');
  Serial.print(output);
  Serial.println("");
  
  while (TIME_SINCE(timer1) < period_ms)
  {
  }
  

}
