
const bool DEBUG = false;

#include "Olfactometer_header.h"

// Define Carrier/Odor MFC numbers
// NOTE: The PCB connector (MFC-1..4) determines the MFC number
// const int CARRIER_MFC = 2;
const int ODOR_MFC = 1;


void setup() {
  if (DEBUG) {
    Serial.begin(115200);
    while (!Serial) {
      delay(100);
    }
  }

  // Set up MFCs.
  // Run for each MFC connected to the board.
  // NOTE: The PCB connector (MFC-1..4) determines the MFC number
  // setupMFC(CARRIER_MFC);
  setupMFC(ODOR_MFC);

  // Initialize valves.
  // This command will determine which IO-expander chips are wired up.
  InitializeValves();

  // Set up built-in LED as a status indicator.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Set up Serial3 on BNC3 (pin 15, RX) and BNC4 (pin 14, TX)
  // for hardware serial communication with the maze Teensy 4.1.
  Serial3.begin(115200);
}


void loop() {

  // Act on USB commands (debug only):
  if (DEBUG) readFromUSB();

  // Act on hardware serial commands from maze Teensy:
  readFromSerial3();

}



void blinkLED(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    if (i < n-1) delay(200);
  }
}

void readFromUSB() {
  static String usbMessage = "";
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if ((inByte == '\n') || (inByte == ';')){
      interpretMessage(usbMessage);
      usbMessage = "";
    } else {
      usbMessage = usbMessage + inByte;
    }
  }
}

void readFromSerial3() {
  static String s3Message = "";
  if (Serial3.available() > 0) {
    char inByte = Serial3.read();
    if ((inByte == '\n') || (inByte == ';')){
      interpretMessage(s3Message);
      s3Message = "";
    } else {
      s3Message = s3Message + inByte;
    }
  }
}

void interpretMessage(String message) {
  message.trim();
  int len = message.length();
  if (len==0) {
    if (DEBUG) Serial.println("#"); // "#" means error
    return;
  }
  char command = message[0];
  String parameters = message.substring(1);
  parameters.trim();

  String intString = "";
  while ((parameters.length() > 0) && (isDigit(parameters[0]))) {
    intString += parameters[0];
    parameters.remove(0,1);
  }
  long arg1 = intString.toInt();

  parameters.trim();
  intString = "";
  while ((parameters.length() > 0) && (isDigit(parameters[0]))) {
    intString += parameters[0];
    parameters.remove(0,1);
  }

  switch (command) {

    // D: changes o[D]or stream flow rate (in mLPM)
    case 'D':
    case 'd':
      setMFCFlowRate(ODOR_MFC, arg1/1000.0);
      delay(250);
      blinkLED((int)round(arg1/1000.0));
      if (DEBUG) {
        Serial.print("Setting ODOR flow to: ");
        Serial.print(arg1/1000.0);
        Serial.println(" lpm");
        Serial.print("Actual ODOR flow: ");
        Serial.print(getMFCFlowRate(ODOR_MFC));
        Serial.println(" lpm");
      }
      break;

    // A: opens [A]ux valve
    case 'A':
    case 'a':
      activateAuxValve(arg1);
      break;

    // X: close au[X] valve
    case 'X':
    case 'x':
      deactivateAuxValve(arg1);
      break;

    // O: opens odor valve
    case 'O':
    case 'o':
      activateOdorValve(arg1);
      break;

    // C: closes odor valve
    case 'C':
    case 'c':
      deactivateOdorValve(arg1);
      break;

    // B: pulse [B]NC1 high for 1 second
    case 'B':
    case 'b':
      digitalWrite(BNC1_pin, HIGH);
      delay(1000);
      digitalWrite(BNC1_pin, LOW);
      if (DEBUG) Serial.println("BNC1 pulsed high for 1 second");
      break;

    // otherwise: error
    default:
      if (DEBUG) Serial.println("#"); // "#" means error
  }
}
