// Motor Pin Definitions
#define ENA 3 // Speed control for Motor 1
#define ENB 6 // Speed control for Motor 2
#define IN1 2 // Motor 1 direction control
#define IN2 4 // Motor 1 direction control
#define IN3 5 // Motor 2 direction control
#define IN4 7 // Motor 2 direction control

char command; // Variable to store received Bluetooth command

void setup() {
  // Motor pin setup
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(9600); // Initialize Serial communication
  Stop(); // Initialize with all motors off
}

void loop() {
  if (Serial.available() > 0) {
    command = Serial.read(); // Read the incoming Bluetooth command

    Stop(); // Turn off motors before processing a new command

    switch (command) {
      case 'f': // Forward
        forward();
        break;
      case 'b': // Backward
        back();
        break;
      case 'l': // Left
        left();
        break;
      case 'r': // Right
        right();
        break;
      case 's': // Stop
        Stop();
        break;
      default:
        // Handle invalid commands
        break;
    }
  }
}

// Motor Control Functions
void forward() {
  digitalWrite(IN1, HIGH); // Left motor forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); // Right motor forward
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 80); // Set speed
  analogWrite(ENB, 80);
}

void back() {
  digitalWrite(IN1, LOW); // Left motor backward
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); // Right motor backward
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 80); // Set speed
  analogWrite(ENB, 80);
}

void left() {
  digitalWrite(IN1, LOW); // Left motor stop or backward
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); // Right motor forward
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 80); // Set speed
  analogWrite(ENB, 80);
}

void right() {
  digitalWrite(IN1, HIGH); // Left motor forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); // Right motor stop or backward
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 80); // Set speed
  analogWrite(ENB, 80);
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
