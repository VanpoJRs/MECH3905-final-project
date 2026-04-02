
const byte joystickPinX = A0;
const byte joystickPinY = A1;
const int buttonPin = 2;

int data_from_MATLAB;
int i = 1;

void setup()
{
  Serial.begin(115200);
  delay(500);

  pinMode(buttonPin, INPUT_PULLUP);  // button uses pull-up
}

void loop()
{
  int buttonState = digitalRead(buttonPin);
  int buttonPressed = (buttonState == LOW);  // pressed = 1

  if (Serial.available() > 0)
  {
    data_from_MATLAB = Serial.parseInt();  // read loop index

    int joystickValueX = analogRead(joystickPinX);
    int joystickValueY = analogRead(joystickPinY);

    Serial.print(String(i) + "," + 
    String(joystickValueX) + "," + 
    String(joystickValueY) + "," + 
    String(buttonPressed));

    Serial.write(13); // CR
    Serial.write(10); // LF

    Serial.flush();

    i += 1;
  }
}