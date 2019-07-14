#include <Ultrasonic.h>
#include <SimpleKalmanFilter.h>

Ultrasonic ultrasonic(8, 9);
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);

void setup() 
{
  Serial.begin(9600);
}

void loop() 
{
  
  int a = ultrasonic.read();
  float distance = simpleKalmanFilter.updateEstimate(a);
  
  Serial.print(distance);
  Serial.print("\t");
  Serial.println(a);
}
