#define echo 8
#define trig 9

void trigPuls(); //Função que gera o pulso de trigger

float pulse;    // Variável que armazena o tempo de duração do echo
float dist_cm;  // Variável que armazena o valor da distância em centimetros

void setup() {
  pinMode(trig, OUTPUT);  // Pino de Trgger sera saida digital
  pinMode(echo, INPUT);   // Pino de Echo sera entrada digital

  digitalWrite(trig, LOW);  //Saida trigger inicia em nivel baixo

  Serial.begin(9600);  // Inicia comunicacao serial
  
}

void loop() {
  trigPulse();  //Aciona o trigger do modulo ultrassonico

  pulse = pulseIn(echo, HIGH);  //Mede o tempo em que o pino de echo fica em nivel alto

  dist_cm = pulse/58.82;    // Valor da distancia em centimetros

  // 340m/s
  // 34000cm/s

  /*
      1000000 us - (34000cm/s)/2  -> /2, pois é tempo ida e volta do echo
          x   us - 1cm
          x = 1E6/17E3 = 58.82 
  */


  Serial.println(dist_cm);    // Imprime o valor na serial
  delay(500);                 // Taxa de atualizacao
 
}

void trigPulse(){
  digitalWrite(trig, HIGH);   // Pulso de trigger em nivel alto
  delayMicroseconds(10);      // Duracao de 10 micro segundos
  digitalWrite(trig, LOW);    // Pulso de trigger em nivel baixo
    
}
