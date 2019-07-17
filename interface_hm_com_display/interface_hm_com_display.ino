#include <LiquidCrystal.h>

#define buttom1 6 //botao 1 no pino digital 6
#define buttom2 7 //

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

int b1=0, b2=0;

void setup() {
  pinMode(buttom1, INPUT);
  pinMode(buttom2, INPUT);

  lcd.begin(16, 2);
  lcd.setCursor(1,0);
  lcd.print("Pressione:");
}

void loop() {
  b1 = digitalRead(buttom1);
  b2 = digitalRead(buttom2);

  if(b1 == LOW)
  {
    lcd.setCursor(2,1);
    lcd.print("Opção 1");
  }
  
  if(b2 == LOW)
  {
    lcd.setCursor(2,1);
    lcd.print("Opção 2");
  }  
}
