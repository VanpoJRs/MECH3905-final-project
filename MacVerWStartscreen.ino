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

  if (Serial.available() > 2)
  {
    data_from_MATLAB = Serial.parseInt();

    int joystickValueX = analogRead(joystickPinX);
    int joystickValueY = analogRead(joystickPinY);

    Serial.print(String(i) + "," + String(joystickValueX) + "," + String(joystickValueY) + "," + String(buttonPressed));
    Serial.write(13);
    Serial.write(10);
    Serial.flush();

    i += 1;
  }
}
