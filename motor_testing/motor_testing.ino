#define ENA 3 // Speed control for Motor 1
#define ENB 6 // Speed control for Motor 2
#define IN1 2 // Motor 1 direction control
#define IN2 4 // Motor 1 direction control
#define IN3 5 // Motor 2 direction control
#define IN4 7 // Motor 2 direction control

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Test Right Turn
  digitalWrite(IN1, HIGH); // Left motor forward
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, LOW);  // Right motor backward or stop
  digitalWrite(IN4, HIGH); 
  analogWrite(ENA, 80); // Set speed
  analogWrite(ENB, 80);
  delay(1500);

  // Stop
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(500);

  // Test Left Turn
  digitalWrite(IN1, LOW);  // Left motor backward or stop
  digitalWrite(IN2, HIGH); 
  digitalWrite(IN3, HIGH); // Right motor forward
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 80); // Set speed
  analogWrite(ENB, 80);
  delay(1500);

  // Stop
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(500);
}

void loop() {
  // Empty loop
}
