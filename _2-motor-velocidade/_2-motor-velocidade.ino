// Programa : Controle 2 motores DC usando Ponte H L298N
//Autor : FILIPEFLOP

//Definicoes pinos Arduino ligados a entrada da Ponte H
int IN1 = 6;
int IN2 = 5;

void setup()
{
  //Define os pinos como saida
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  Serial.begin(9600);
}

void loop()
{
  int a = analogRead(A0) / 4;
  
  //Gira o Motor A no sentido horario
  analogWrite(IN1, a);
  digitalWrite(IN2, LOW);

  Serial.println(a);
}
