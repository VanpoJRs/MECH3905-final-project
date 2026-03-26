const byte joystickPinX = A0; // analog input pin connected to joystick X-axis
const byte joystickPinY = A1; // analog input pin connected to joystick Y-axis
const int buttonPin=2;

int data_from_MATLAB; // stores the step index received from MATLAB
int i = 1; // local counter for transmitted samples

void setup() // runs once at startup
{
  Serial.begin(115200); // initialize serial communication at 115200 baud
  delay(500); // allow serial connection to stabilize
}

void loop() // runs continuously after setup
{
  if (Serial.available() > 2) // check if incoming data from MATLAB is available
  {
    data_from_MATLAB = Serial.parseInt(); // read integer step index sent from MATLAB

    int joystickValueX = analogRead(joystickPinX); // read joystick X-axis ADC value (0–1023)
    int joystickValueY = analogRead(joystickPinY); // read joystick Y-axis ADC value (0–1023)

    int buttonState = digitalRead(buttonPin);
    // Convert to 1 when pressed, 0 when not
    int buttonPressed = (buttonState == LOW) ? 1 : 0;

    // send formatted response: counter , X value , Y value and button value
    Serial.print(String(i) + "," + String(joystickValueX) + "," + String(joystickValueY)+","+String(buttonPressed));

    Serial.write(13); // send carriage return (CR)
    Serial.write(10); // send line feed (LF)
    Serial.flush(); // wait until transmission is complete

    i += 1; // increment sample counter
  }
}
