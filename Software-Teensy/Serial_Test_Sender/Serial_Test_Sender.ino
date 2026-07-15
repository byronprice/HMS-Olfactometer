// Serial_Test_Sender.ino
// Teensy 4.1 (maze controller)
//
// Sends a sample olfactometer command string over Serial1 every 10 seconds.
// Wire: pin 1 (Serial1 TX) --> pin 15 (Serial3 RX / BNC3) on olfactometer Teensy 4.0

void setup() {
  // Serial.begin(115200);   // USB: for monitoring what we send
  Serial1.begin(115200);  // TX on pin 1
}

void loop() {
  const char* cmd = "c1;c11;c17;o2;o12;o18\n";
  Serial1.print(cmd);
  // Serial.print("Sent: ");
  // Serial.print(cmd);
  delay(5000);

}
