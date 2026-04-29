/*
  KAA109 Engineering Problem Solving and Data Analysis
  Assignment 3: Arduino Autonomous Vehicle
  Team Number: 
  Team Members: 
  
  This programme implements the following funcitonality:
   * 
   * 
   * 
*/

// Enable debugging:
// If defined then output will be printed through Serial.print
#define DEBUG

// Tinkercad or physical system selection:
// If TINKERCAD is defined, then the RGB LEDs will use different
// values for enabling and setting their red, green and blue outputs.
//#define TINKERCAD

#ifdef DEBUG
#define log(x) Serial.print(x)
#define logln(x) Serial.println(x)
#else
#define log(x)
#define logln(x)
#endif

// Drive pin definitions
#define DRIVE_RIGHT_PIN_1 9   // PWM/analogWrite(0-255)
#define DRIVE_RIGHT_PIN_2 8
#define DRIVE_LEFT_PIN_1  10  // PWM/analogWrite(0-255)
#define DRIVE_LEFT_PIN_2  12

// Encoder pin definitions:
#define LEFT_ENCODER_PIN  2
#define RIGHT_ENCODER_PIN 3

// Ultrasonic sensor pin and value definitions:
#define LEFT_TRIG_PIN     A1
#define LEFT_ECHO_PIN     A0

#define RIGHT_TRIG_PIN    A2
#define RIGHT_ECHO_PIN    A3

// RGB LED pin definitions:
#define RED_LED_PIN           5
#define GREEN_LED_PIN         11
#define BLUE_LED_PIN          6
#define LED_LEFT_EN_PIN       4
#define LED_RIGHT_EN_PIN      7

// 'On' brightness of RGB LEDs (max 255):
#ifdef TINKERCAD
#define BRIGHT_PWM        255
#else
#define BRIGHT_PWM        16
#endif

// Left and right enable output values:
// Hint: Make sure you enable the left and right LEDs
//       if you want them to be on.
// Hint: To set red, green and blue outputs use the
//       LED_PWM_VALUE(x) macro to get their brightness.
#ifdef TINKERCAD
#define LED_LEFT_ENABLED   LOW
#define LED_RIGHT_ENABLED  LOW
#define LED_LEFT_DISABLED  HIGH
#define LED_RIGHT_DISABLED HIGH
#define LED_PWM_VALUE(pwm) pwm
#else
#define LED_LEFT_ENABLED   LOW
#define LED_RIGHT_ENABLED  HIGH
#define LED_LEFT_DISABLED  HIGH
#define LED_RIGHT_DISABLED LOW
#define LED_PWM_VALUE(pwm) (255-pwm)
#endif

#define ENCODER_DISTANCE 0.001

unsigned long totalTime = 0;
unsigned long lastPrintTime = 0;
volatile unsigned long previousLeftTime = 0;
volatile unsigned long previousRightTime = 0;
volatile float leftSpeed = 0;
volatile float rightSpeed = 0;
volatile long leftPulses = 0;
volatile long rightPulses = 0;
float leftDistance = 0;
float rightDistance = 0;
float prevLeftSpeed = 0;
float error = 0;
int slavePower = 241;
int kp = 50;
unsigned long lastCorrection = 0;
/*
Drive forwards
*/
void driveForwards()
{
  digitalWrite(DRIVE_LEFT_PIN_1, LOW); //Master motor - no pwm variability
  analogWrite(DRIVE_LEFT_PIN_2, 241); //Slave motor 
  digitalWrite(DRIVE_RIGHT_PIN_1, LOW);
  digitalWrite(DRIVE_RIGHT_PIN_2, HIGH);

  analogWrite(BLUE_LED_PIN, LED_PWM_VALUE(BRIGHT_PWM)); //high is off 
}

void getLeftSpeed() {
  totalTime = micros();
  leftSpeed = 1/(((totalTime-previousLeftTime)*40.0)/1000000.0)*60.0;
  previousLeftTime = totalTime;
  leftPulses++;
}

void getRightSpeed() {
  totalTime = micros();
  rightSpeed = 1/(((totalTime-previousRightTime)*40.0)/1000000.0)*60.0;
  previousRightTime = totalTime;
  rightPulses++;
}

void setup()
{
  pinMode(DRIVE_LEFT_PIN_1, OUTPUT);
  pinMode(DRIVE_LEFT_PIN_2, OUTPUT);
  pinMode(DRIVE_RIGHT_PIN_1, OUTPUT);
  pinMode(DRIVE_RIGHT_PIN_2, OUTPUT);
  pinMode(LEFT_ENCODER_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT);
  pinMode(LED_LEFT_EN_PIN, OUTPUT);
  pinMode(LED_RIGHT_EN_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), getLeftSpeed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), getRightSpeed, CHANGE);
  Serial.begin(9600);

  Serial.println("~~~new test~~");
}

/**
 * 
 */
void loop() 
{
  driveForwards();

  if (millis() - lastPrintTime >= 5000) {
    
    Serial.println(leftSpeed);
    Serial.println(rightSpeed);
    Serial.println(slavePower);
    lastPrintTime = millis();

    error = rightSpeed - leftSpeed;
    Serial.println(error);
    //slavePower += error / kp;
  }
}
//INTERRUPTS











