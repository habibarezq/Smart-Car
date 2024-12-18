// char command; // Variable to store received Bluetooth command

// void setup() {
//   pinMode(8, OUTPUT);        // Set pin 8 as output for the LED
//   digitalWrite(8, LOW);      // Turn LED off initially
//   Serial.begin(9600);        // Initialize Serial communication
// }

// void loop() {
//   // Check if there is any data available to read
//   while (Serial.available()) {
//     command = Serial.read(); // Read the incoming byte

//     // Debug: Print the received command
//     Serial.print("Received Command: ");
//     Serial.println(command);

//     // Act based on the command received
//     if (command == '1') {
//       digitalWrite(8, HIGH); // Turn LED ON
//       Serial.println("LED ON");
//     } else if (command == '0') {
//       digitalWrite(8, LOW);  // Turn LED OFF
//       Serial.println("LED OFF");
//     }

//     // Clear out any residual characters
//     while (Serial.available()) {
//       Serial.read();
//     }
//   }
// }


char command;

const int ledF = 2; // LED for "Forward"
const int ledB = 3; // LED for "Backward"
const int ledL = 4; // LED for "Left"
const int ledR = 5; // LED for "Right"

void setup()
{
  //HC05.begin(9600);  // Set the baud rate to your Bluetooth module
  
  // Configure LED pins as OUTPUT
  pinMode(ledF, OUTPUT);
  pinMode(ledB, OUTPUT);
  pinMode(ledL, OUTPUT);
  pinMode(ledR, OUTPUT);
  Serial.begin(9600);

  Stop(); // Initialize with all LEDs off
}

void loop() {
  if (Serial.available() > 0) {
    command = Serial.read();

    Stop(); // Turn off all LEDs before processing a new command
    
    switch (command) {
      case 'f':
        forward();
        break;
      case 'b':
        back();
        break;
      case 'l':
        left();
        break;
      case 'r':
        right();
        break;
      case 's':
        Stop();
      default:
        // Handle invalid commands if necessary
        break;
    }
  }
}

void forward()
{
  digitalWrite(ledF, HIGH); // Turn on the "Forward" LED
  
}

void back()
{
  digitalWrite(ledB, HIGH); // Turn on the "Backward" LE
}

void left()
{
  digitalWrite(ledL, HIGH); // Turn on the "Left" LED

}

void right()
{
  digitalWrite(ledR, HIGH); // Turn on the "Right" LED

}

void Stop()
{
  // Turn off all LEDs
  digitalWrite(ledF, LOW);
  digitalWrite(ledB, LOW);
  digitalWrite(ledL, LOW);
  digitalWrite(ledR, LOW);
}

