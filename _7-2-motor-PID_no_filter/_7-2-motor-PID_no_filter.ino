#include <Ultrasonic.h>
#include <SimpleKalmanFilter.h>

#include <FastPID.h>

#define PIN_INPUT     A0
#define PIN_SETPOINT  A1
#define PIN_OUTPUT    5

float Kp = 10, Ki = 0, Kd = 0.1, Hz = 100;
int output_bits = 9;
bool output_signed = true;

FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);

Ultrasonic ultrasonic(8, 9);

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
  
  static float distance_last = 0;;
  int a = ultrasonic.read();
  float distance_f = 0.1*a + 0.9*distance_last;
  distance_last = distance_f;
  int distance = round(distance_f);

  int setpoint = 10;
  int feedback = distance;
  int output = myPID.step(setpoint, feedback);

  char str[40] = "";

  Serial.print(distance_f);
  Serial.print('\t');
  Serial.print(setpoint - feedback);
  Serial.print('\t');
  
  if (output < 0)
  {
    Serial.print(output);
    
    output = -output;
    output += 150;
    
    analogWrite(IN1, output);
    analogWrite(IN2, LOW);
  }
  else
  {
    Serial.print(output);
    
    output += 150;
    analogWrite(IN1, LOW);
    analogWrite(IN2, output);
  }

  Serial.println("");
}
