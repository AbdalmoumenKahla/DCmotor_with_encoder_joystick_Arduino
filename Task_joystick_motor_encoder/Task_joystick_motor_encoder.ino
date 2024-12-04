#include <LiquidCrystal.h>

const int rs = 8, en = 13, d4 = 9, d5 = 10, d6 = 11, d7 = 12;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


// Define motor driver pins
const int motorPin1 = 5;  // IN1 on motor driver
const int motorPin2 = 4;  // IN2 on motor driver
const int enablePin = 6;  // ENA on motor driver (PWM control)

// Define Encoder pins
const int EncoderPin1 = 2; // Encoder A
const int EncoderPin2 = 3; // Encoder B

// Define joystick pins
const int joystickY = A1;  // Y-axis to control speed and direction

// Variables for joystick and motor control
int joystickYValue = 0;    // To store joystick Y value
int motorSpeed = 0;        // Motor speed (0-255 for PWM)
int motorDirection = 0;    // Motor direction (1 for clockwise, -1 for counterclockwise, 0 for state)
volatile int encoderPos = 0; // Encoder position (counts)
int lastEncoderPos = 0;    // For tracking encoder position changes

const int degpertick = 360 / 66;

// Interrupt function to track encoder position
void encoderISR()
{
    int val = digitalRead(EncoderPin2);
    if(val == 0){
      encoderPos += 1;
    }
    else {
       encoderPos -= 1;
    }


}

void setup() {

  lcd.begin(16, 2); // Initialize LCD with 16x2 dimensions

  // Set motor control pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  // Set encoder pins as inputs
  pinMode(EncoderPin1, INPUT_PULLUP);
  pinMode(EncoderPin2, INPUT_PULLUP);

  // Attach interrupt to encoder pin to detect changes
  attachInterrupt(digitalPinToInterrupt(EncoderPin1), encoderISR, RISING);
  

  // Start serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Read the Y-axis value from the joystick (range: 0-1023)
  int cycle = encoderPos/360;
  joystickYValue = analogRead(joystickY);
  
  // Map joystickYValue to a PWM value (0-255)
  motorSpeed = map(abs(joystickYValue - 512), 0, 512, 0, 255);  // Speed control based on distance from neutral
  
  // Set motor direction
  if (joystickYValue > 512) {
    // Moving joystick up -> clockwise rotation
    motorDirection = 1;
    analogWrite(enablePin, motorSpeed);
  } else if (joystickYValue < 512) {
    // Moving joystick down -> counterclockwise rotation
    motorDirection = -1;
    analogWrite(enablePin, motorSpeed);
  } else {
    // Joystick in neutral position -> stop the motor
    motorDirection = 0;
    analogWrite(enablePin, 0);  // No speed (motor stops)
  }

  // Set motor direction based on joystick movement
  if (motorDirection == 1) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  } else if (motorDirection == -1) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  }

  // Print motor status and encoder position for debugging
  Serial.print("Joystick Y: ");
  Serial.print(joystickYValue);
  Serial.print("\tMotor Speed: ");
  Serial.print(motorSpeed);
  Serial.print("\tDirection: ");
  Serial.print(motorDirection == 1 ? "Clockwise" : (motorDirection == -1 ? "Counterclockwise" : "Stopped"));
  Serial.print("\tEncoder Position: ");
  Serial.print(encoderPos);
  Serial.print("\tEncoder cycle: ");
  Serial.println(cycle);

   // Display on LCD
  lcd.setCursor(0, 0); // Set to first row
  lcd.print("Y:");
  lcd.print(joystickYValue);
  lcd.print(" Spd:");
  lcd.print(motorSpeed);

  lcd.setCursor(0, 1); // Set to second row
  lcd.print(motorDirection == 1 ? "CW " : (motorDirection == -1 ? "CCW" : "Stop"));
  lcd.print("  pos:");
  lcd.print(encoderPos);


  // Small delay to avoid too fast serial output
  delay(300);
}
