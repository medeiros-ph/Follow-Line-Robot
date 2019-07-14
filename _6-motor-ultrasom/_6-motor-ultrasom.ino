#include <Ultrasonic.h>
#include <SimpleKalmanFilter.h>

Ultrasonic ultrasonic(8, 9);
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);

int IN1 = 5;
int IN2 = 4;
int IN3 = 6;
int IN4 = 7;

void setup()
{
  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop()
{
  int a = analogRead(A0);
  int distance_unfiltered = ultrasonic.read();
  float distance = simpleKalmanFilter.updateEstimate(distance_unfiltered);
  
  Serial.println(distance);
  
  analogWrite(IN1, a/4);
  digitalWrite(IN2, LOW);
  analogWrite(IN3, a/4);
  digitalWrite(IN4, LOW);
}
