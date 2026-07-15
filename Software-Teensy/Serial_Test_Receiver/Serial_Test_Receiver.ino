// Serial_Test_Receiver.ino
// Teensy 4.0 (olfactometer)
//
// Receives commands over Serial3 and prints acknowledgments over USB.
// Wire: pin 1 (Serial1 TX) on sender --> pin 15 (Serial3 RX / BNC3) here

static String rxBuffer = "";

void setup() {
  Serial.begin(115200);   // USB: for printing acknowledgments
  Serial3.begin(115200);  // RX on pin 15 (BNC3)
  Serial.println("Receiver ready.");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {

  while (Serial3.available()) {
    char c = Serial3.read();
    if (c == '\n' || c == ';') {
      rxBuffer.trim();
      if (rxBuffer.length() > 0) {
        acknowledge(rxBuffer);
        rxBuffer = "";
      }
    } else {
      rxBuffer += c;
    }
  }
}

void acknowledge(String msg) {
  char cmd = msg[0];
  int arg  = msg.substring(1).toInt();

  Serial.print("ACK | cmd='");
  Serial.print(cmd);
  Serial.print("' arg=");
  Serial.print(arg);
  Serial.print(" -> ");

  switch (cmd) {
    case 'o': case 'O':
      Serial.print("Open odor valve ");
      Serial.println(arg);
      break;
    case 'c': case 'C':
      Serial.print("Close odor valve ");
      Serial.println(arg);
      break;
    case 'a': case 'A':
      Serial.print("Open aux valve ");
      Serial.println(arg);
      break;
    case 'x': case 'X':
      Serial.print("Close aux valve ");
      Serial.println(arg);
      break;
    default:
      Serial.print("Unknown command: ");
      Serial.println(msg);
  }
}
