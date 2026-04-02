
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
  // 1. Read button state CONSTANTLY, not just when serial is available
  int buttonState = digitalRead(buttonPin);
  int buttonPressed = (buttonState == LOW) ? 1 : 0; 

  if (Serial.available() > 0)
  {
    // 2. Read the index sent by MATLAB
    int incomingIndex = Serial.parseInt(); 

    // 3. Read Joysticks
    int joystickValueX = analogRead(joystickPinX);
    int joystickValueY = analogRead(joystickPinY);

    // 4. Send back the EXACT index MATLAB is expecting (incomingIndex)
    Serial.print(incomingIndex); 
    Serial.print(",");
    Serial.print(joystickValueX);
    Serial.print(",");
    Serial.print(joystickValueY);
    Serial.print(",");
    Serial.println(buttonPressed); // println handles the CR/LF automatically

    Serial.flush();
  }
}
