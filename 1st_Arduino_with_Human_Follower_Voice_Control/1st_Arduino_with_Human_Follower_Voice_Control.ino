#include <SoftwareSerial.h>

// Motor Pin Definitions
#define ENA 3  // Speed control for Motor 1
#define ENB 6  // Speed control for Motor 2
#define IN1 2  // Motor 1 direction control
#define IN2 4  // Motor 1 direction control
#define IN3 5  // Motor 2 direction control
#define IN4 7  // Motor 2 direction control

#define MAX_SPEED 140
#define MIN_SPEED 80

#define MIN_DISTANCE 10
#define MAX_DISTANCE 30

//IR Sensors
#define IR_SENSOR_RIGHT 8
#define IR_SENSOR_LEFT 9

//LEDS
#define PIN_LED_LEFT 12   //Left LED
#define PIN_LED_RIGHT 13  //Right LED

char command, value;              // Variable to store received Bluetooth command
bool lineTrackingActive = false;  // Variable to track line tracking state
bool bodyTrackingActive = false;  // Variable to track line tracking state
float distance;

//Flashing Leds Variables
bool isFlashingLeft = false;
bool isFlashingRight = false;
unsigned long flashStartTime = 0;
int flashCount = 0;
const unsigned long flashInterval = 200;  // Interval for LED flashing

//Speeds
float FA = 81, FB = 110, BA = 70, BB = 105, R = 80;

SoftwareSerial mySerial(10, 11);  // RX, TX


void setup() {
  // Motor pin setup
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);


  // IR sensors pins setup
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);

  pinMode(PIN_LED_LEFT, OUTPUT);
  pinMode(PIN_LED_RIGHT, OUTPUT);

  // Set LEDs off initially
  digitalWrite(PIN_LED_LEFT, LOW);
  digitalWrite(PIN_LED_RIGHT, LOW);

  Serial.begin(9600);  // Initialize Serial communication
  mySerial.begin(9600);
  Stop();  // Initialize with all motors off
}

void loop() {
  handleObstacle();
  if (mySerial.available()) {
    String receivedData = mySerial.readStringUntil('\n');  // Read the incoming distance
    distance = receivedData.toFloat();                     // Convert the received string to a float

    Serial.print("Distance received (float): ");
    Serial.println(distance);
  }

  // Only run line tracking if it's active
  if (lineTrackingActive) {
    lineTracker();
  }

  if (bodyTrackingActive) {
    bodyTracking();
  }

  //Otherwise, keep processing commands
  processCommand();

  // Handle non-blocking LED flashing
  handleLEDFlashing();

  //Voice Control
  voicecontrol();
}

void lineTracker() {

  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);

  //If none of the sensors detects black line, then go straight
  if (rightIRSensorValue == LOW && leftIRSensorValue == LOW) {
    forward();
    Serial.println("FORWARD");
  }
  //If right sensor detects black line, then turn right
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW) {
    right();
    Serial.println("RIGHT");
  }
  //If left sensor detects black line, then turn left
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH) {
    left();
    Serial.println("LEFT");
  }
  //If both the sensors detect black line, then stop
  else {
    Stop();
    Serial.println("STOP");
  }
}
void bodyTracking() {
  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);

  //If right sensor detects hand, then turn right.
  if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH) {
    right();
  }
  //If left sensor detects hand, then turn left. We increase right motor speed and decrease the left motor speed to turn towards left
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW) {
    left();
  }
  //If distance is between min and max then go straight
  else if (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE && rightIRSensorValue == HIGH && leftIRSensorValue == HIGH) {
    forward();
  }
  //stop the motors
  else {
    Stop();
  }
}
void processCommand() {
  // Clear the serial buffer
  if (Serial.available() > 0) {
    command = Serial.read();  // Read the latest incoming command
    Serial.println(command);
    switch (command) {
      case 'f': forward(); break;
      case 'b': back(); break;
      case 'l': left(); break;
      case 'r': right(); break;
      case 's': Stop(); break;

      case 't':  // Start line tracking letter
        lineTrackingActive = true;
        Serial.println("Line Tracking Started");
        break;
      case 'm':  // Stop line tracking letter
        lineTrackingActive = false;
        Stop();  // Ensure motors stop
        Serial.println("Line Tracking Stopped");
        break;

      case 'h':  //Start Body Tracking
        bodyTrackingActive = true;
        Serial.println("Body Tracking Started");
        break;
      case 'c':  //Stop Body Tracking
        bodyTrackingActive = false;
        Stop();  // Ensure motors stop
        Serial.println("Body Tracking Stopped");
        break;

      case 'i':  //Increase Speed
        increaseSpeed();
        break;
      case 'd':  //Decrease Speed
        decreaseSpeed();
        break;

      default: break;  // Ignore invalid commands
    }
  }
}

void handleObstacle() {
  if (distance <= 15.0) {
    Stop();
    // Clear the serial buffer to ignore commands sent during the obstacle
    while (Serial.available() > 0) {
      Serial.read();
    }
  } else {
    if (lineTrackingActive) {
      lineTracker();
    } else {
      processCommand();
    }
  }
  delay(50);  // Short delay to stabilize readings
}

void increaseSpeed() {
  // Increase all speeds by 1%
  FA *= 1.01;
  FB *= 1.01;
  BA *= 1.01;
  BB *= 1.01;
  R *= 1.01;

  // Ensure none of the speeds exceed MAX_SPEED
  if (FA > MAX_SPEED) FA = MAX_SPEED;
  if (FB > MAX_SPEED) FB = MAX_SPEED;
  if (BA > MAX_SPEED) BA = MAX_SPEED;
  if (BB > MAX_SPEED) BB = MAX_SPEED;
  if (R > MAX_SPEED) R = MAX_SPEED;
}

void decreaseSpeed() {
  // Decrease all speeds by 1%
  FA *= 0.99;
  FB *= 0.99;
  BA *= 0.99;
  BB *= 0.99;
  R *= 0.99;

  // Ensure none of the speeds go below MIN_SPEED
  if (FA < MIN_SPEED) FA = MIN_SPEED;
  if (FB < MIN_SPEED) FB = MIN_SPEED;
  if (BA < MIN_SPEED) BA = MIN_SPEED;
  if (BB < MIN_SPEED) BB = MIN_SPEED;
  if (R < MIN_SPEED) R = MIN_SPEED;
}

void handleLEDFlashing() {
  if (isFlashingLeft || isFlashingRight) {
    unsigned long currentTime = millis();

    // Check if it's time to toggle the LED state
    if (currentTime - flashStartTime >= flashInterval) {
      flashStartTime = currentTime;

      if (isFlashingLeft) {
        digitalWrite(PIN_LED_LEFT, !digitalRead(PIN_LED_LEFT));  // Toggle left LED
      } else if (isFlashingRight) {
        digitalWrite(PIN_LED_RIGHT, !digitalRead(PIN_LED_RIGHT));  // Toggle right LED
      }

      flashCount++;
    }

    // Stop flashing after two blinks (4 state changes)
    if (flashCount >= 4) {
      isFlashingLeft = false;
      isFlashingRight = false;
      digitalWrite(PIN_LED_LEFT, LOW);
      digitalWrite(PIN_LED_RIGHT, LOW);
    }
  }
}

void voicecontrol() {
  if (Serial.available() > 0) {
    value = Serial.read();
    Serial.println(value);
    //If the char value is "^", the car moves forward.
    if (value == '^') {
      forward();
    }
    //If the char value is "-", the car moves backward.
    else if (value == '-') {
      backward();
    }
    //If the char value is "<", the car moves left.
    else if (value == '<') {
      left();
    }
    //If the char value is ">", the car moves right.
    else if (value == '>') {
      right();
    }
    //If the char value is "*", the car is stopped.
    else if (value == '*') {
      Stop();
    }
  }
}

// Motor Control Functions
void forward() {
  digitalWrite(IN1, HIGH);  // Left motor forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);  // Right motor forward
  digitalWrite(IN4, LOW);
  analogWrite(ENA, FA);  // Set speed
  analogWrite(ENB, FB);  //past 115
}

void back() {
  digitalWrite(IN1, LOW);  // Left motor backward
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  // Right motor backward
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, BA);  // Set speed
  analogWrite(ENB, BB);
}

void left() {
  digitalWrite(IN1, LOW);  // Left motor stop or backward
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);  // Right motor forward
  digitalWrite(IN4, LOW);
  analogWrite(ENA, R);  // Set speed
  analogWrite(ENB, R);

  // Start non-blocking LED flashing
  isFlashingLeft = true;
  isFlashingRight = false;
  flashStartTime = millis();
  flashCount = 0;
}

void right() {
  digitalWrite(IN1, HIGH);  // Left motor forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  // Right motor stop or backward
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, R);  // Set speed
  analogWrite(ENB, R);

  // Start non-blocking LED flashing
  isFlashingRight = true;
  isFlashingLeft = false;
  flashStartTime = millis();
  flashCount = 0;
}

void Stop() {
  // Turn off motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
