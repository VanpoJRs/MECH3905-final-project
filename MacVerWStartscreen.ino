//don't change anything in IDE code

#include <LiquidCrystal.h>

const int rs=3, en=4, d4=5, d5=6, d6=7, d7=8;
LiquidCrystal lcd(rs,en,d4,d5,d6,d7);

const byte joystickPinX = A0;
const byte joystickPinY = A1;
const int buttonPin=2;

int data_from_MATLAB;
int i = 1;

bool gameStarted = false;

void setup()
{
  Serial.begin(115200);
  delay(500);

  lcd.begin(16, 2);
  pinMode(buttonPin, INPUT_PULLUP);

  lcd.setCursor(0,0);
  lcd.print("Rush hour");
  lcd.setCursor(0,1);
  lcd.print("Press to start");
}

void loop()
{
  int buttonState = digitalRead(buttonPin);
  int buttonPressed = (buttonState == LOW);

  if (buttonPressed && !gameStarted) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Game Start");
    gameStarted = true;
  }


if (Serial.available() > 0)
{
  String msg = Serial.readStringUntil('\n');

  if (msg == "START") {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Game Running");
  }

  if (msg == "WIN") {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("You reached!");
    lcd.setCursor(0,1);
    lcd.print("Press restart");
  }

  if (msg == "RESET") {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Press to start");
  }
}
