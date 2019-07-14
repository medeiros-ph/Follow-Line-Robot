#include <Ultrasonic.h>

Ultrasonic ultrasonic(8, 9);

void setup() 
{
  Serial.begin(9600);
}

void loop() 
{
  static float distance_last = 0;
  float distance;

  int a = ultrasonic.read();
  distance = 0.1*a + 0.9*distance_last;
  distance_last = distance;
  
  Serial.print(distance);
  Serial.print("\t");
  Serial.print(a);
  Serial.println("");
}
