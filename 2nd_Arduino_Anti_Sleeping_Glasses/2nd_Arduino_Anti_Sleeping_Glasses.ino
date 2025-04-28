#include <SoftwareSerial.h>
#include <Servo.h>
//Sensor pins
#define trigPin 8
#define echoPin 9

//Buzzer Pin
#define buzzerPin 4
#define servo 6

//Flame Sensor Pins
#define flame 2
#define flameLed 3

#define MAX_DISTANCE 25       // Maximum distance to detect (in cm)
#define OBSTACLE_DISTANCE 10  // Distance in cm to trigger constant tone

#define IR_Sensor 5

float duration;
float distance;
bool flameActive=false;
SoftwareSerial mySerial(10, 11);  // RX, TX


void setup() {
  // Buzzer pin setup
  pinMode(buzzerPin, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(servo, OUTPUT);

  pinMode(IR_Sensor, INPUT);

  pinMode(flame, INPUT);  // Set Flame sensor pin as input
  pinMode(flameLed, OUTPUT);

  Serial.begin(9600);  // Initialize Serial communication
  mySerial.begin(9600);

  // Move servo through initial angles

  for (int angle = 90; angle <= 140; angle += 5) {
    servoPulse(servo, angle);
  }
  for (int angle = 140; angle >= 90; angle -= 5) {
    servoPulse(servo, angle);
  }
  for (int angle = 90; angle >= 0; angle -= 5) {
    servoPulse(servo, angle);
  }
  for (int angle = 0; angle <= 90; angle += 5) {
    servoPulse(servo, angle);
  }
  delay(50);

  distance = ultraSonic();
  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  ultraSonic();
  handleObstacle();
  flameSensor();
}



void flameSensor() {
  Serial.println(digitalRead(flame));
  if (digitalRead(flame) == LOW && !flameActive) {  // Flame detected (active LOW sensor)
    Serial.println("Flame detected!");
    flashLED();  // Call function to flash the LED
    //alertBuzzer();
    flameActive=true;
  } else {
    digitalWrite(flameLed, LOW);  // Turn off LED when no flame is detected
    Serial.println("No flame detected.");
  }

  delay(200);
}
void alertBuzzer() {
  for (int i = 0; i < 5; i++) {     // Sound the buzzer 5 times
    digitalWrite(buzzerPin, HIGH);  // Turn buzzer on
    delay(300);                     // Wait 300ms
    digitalWrite(buzzerPin, LOW);   // Turn buzzer off
    delay(300);                     // Wait 300ms
  }
  noTone(buzzerPin);
  flameActive=true;
}
void flashLED() {
  delay(8000);
  for (int i = 0; i < 5; i++) {    // Flash LED 5 times
    digitalWrite(flameLed, HIGH);  // Turn LED on
    delay(200);                    // Wait 200ms
    digitalWrite(flameLed, LOW);   // Turn LED off
    delay(200);                    // Wait 200ms
  }
  flameActive=true;
}
void antiSleeping() {
  int IR_SensorValue = digitalRead(IR_Sensor);
  static unsigned long lastTime = 0;  // To store the last time the IR sensor was triggered
  static bool triggered = false;      // To track if the IR sensor has been triggered

  if (IR_SensorValue == LOW && !triggered) {
    triggered = true;
    lastTime = millis();  // Record the current time
  }

  if (IR_SensorValue == HIGH) {
    triggered = false;  // Reset if the IR sensor is no longer triggered
    noTone(buzzerPin);  // Turn off buzzer
  }

  if (triggered && (millis() - lastTime >= 2000)) {
    tone(buzzerPin, 1000);  // Trigger buzzer after 2 seconds of no signal
    distance = 0;
    mySerial.println(distance);
  }
}

void servoPulse(int pin, int angle) {
  int pwm = (angle * 11) + 500;  // Convert angle to microseconds
  digitalWrite(pin, HIGH);
  delayMicroseconds(pwm);
  digitalWrite(pin, LOW);
  delay(50);  // Refresh cycle of servo
}

long ultraSonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration * .0343) / 2;
  Serial.print("Distance: ");
  Serial.println(distance);
  mySerial.println(distance);
  delay(100);

  return distance;
}

void handleObstacle() {
  // Check if the distance is less than the set threshold

  if (distance <= 15) {
    // Constant tone for very close obstacles
    tone(buzzerPin, 1000);  // Steady 1000 Hz tone
  } else {
    noTone(buzzerPin);  // Stop the tone if no obstacle is detected
  }
  delay(50);  // Short delay to stabilize readings
}
