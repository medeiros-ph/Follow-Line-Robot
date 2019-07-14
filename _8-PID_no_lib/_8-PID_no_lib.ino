#include <Ultrasonic.h>
#include <TimerOne.h>

float Kp = 0.5;
float Ki = 0.0;
float Kd = 0;
float Hz = 100;

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

  //Timer1.initialize(period_ms * 1000);
  //Timer1.attachInterrupt(PID);
}

void loop()
{
  uint32_t timer1 = millis();

  PID();
  
  int out= round(output);

  if (out == 0)
  {
    analogWrite(IN1, LOW);
    analogWrite(IN2, LOW);
  }
  else if (out < 0)
  {
    analogWrite(IN1, (-out) + motor_offset);
    analogWrite(IN2, LOW);
  }
  else
  {
    analogWrite(IN1, LOW);
    analogWrite(IN2, out + motor_offset);
  }

  Serial.print(distance_f);
  Serial.print('\t');
  Serial.print(out);
  Serial.print('\t');
  Serial.print(output);
  Serial.println("");
  
  while ((millis() - timer1) < period_ms)
  {
  }
  

}
