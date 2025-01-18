// For sending commands from arduino nano to pc.

// Joystick 1
const int J1_X_PIN = A0;
const int J1_Y_PIN = A1;
const int J1_SW_PIN = 16;

// Joystick 2
const int J2_X_PIN = A3;
const int J2_Y_PIN = A4;
const int J2_SW_PIN = 19;


// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
  pinMode(J1_SW_PIN, INPUT_PULLUP);
  pinMode(J2_SW_PIN, INPUT_PULLUP);
}

// the loop function runs over and over again forever
void loop() {

  // Joystick 1
  int J1_xValue = analogRead(J1_X_PIN);
  int J1_yValue = analogRead(J1_Y_PIN);
  int J1_buttonState = digitalRead(J1_SW_PIN);

  // Joystick 2
  int J2_xValue = analogRead(J2_X_PIN);
  int J2_yValue = analogRead(J2_Y_PIN);
  int J2_buttonState = digitalRead(J2_SW_PIN);

  // Joystick 1
  Serial.print(J1_xValue);
  Serial.print("@");
  Serial.print(J1_yValue);
  Serial.print("@");

  // Check if the button is pressed
  if (J1_buttonState == LOW) {
    Serial.print(1);
    Serial.print("@");
  } else {
    Serial.print(0);
    Serial.print("@");
  }

  // Joystick 2
  Serial.print(J2_xValue);
  Serial.print("@");
  Serial.print(J2_yValue);
  Serial.print("@");

  // Check if the button is pressed
  if (J2_buttonState == LOW) {
    Serial.println(1);
  } else {
    Serial.println(0);
  }

  delay(100);            
}


