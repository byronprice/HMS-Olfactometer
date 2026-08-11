//check Serialn

#include "Olfactometer_header.h"

const bool DEBUG = false;

const int CARRIER_MFC = 2;
const int ODOR_MFC = 1;

// Bank definitions: index 0=zone0, 1=zone1, 2=zone2
const int bankOdor[3]  = {17, 1, 9};
const int bankBlank[3] = {18, 2, 10};

const float odor_flow_rate = 3.0;

int activeBankNumber = -1;
unsigned long previousValveTimer = 0;
bool isValveCurrentlyOpen = false;
const unsigned long valveOnMin  = 1000;  // ms
const unsigned long valveOnMax  = 3000;  // ms
// OFF is always ON/4, preserving the 4:1 ratio

unsigned long valveOnDuration  = 2000;
unsigned long valveOffDuration = 500;

bool          controlMode     = false;
int           blinkCount      = 0;
bool          ledOn           = false;
unsigned long lastLEDMillis   = 0;
const unsigned long blinkInterval = 250;  // ms per on/off phase


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  delay(500);  // allow I2C peripherals to power up before scanning
  randomSeed(micros());

  Serial3.begin(115200);

  setupMFC(ODOR_MFC);
  InitializeValves();

  pinMode(BNC1_pin, OUTPUT);
  digitalWrite(BNC1_pin, LOW);

  // Ensure known state: all valves closed on boot
  for (int i = 0; i < 3; i++) {
    deactivateOdorValve(bankOdor[i]);
    deactivateOdorValve(bankBlank[i]);
  }

  if (DEBUG) {
    Serial.begin(115200);
    Serial.println("Olfactometer Receiver Ready.");
  }

}


void loop() {
  readFromSender();
  handleValvePulsing();
  handleLEDBlink();
}


void handleLEDBlink() {
  if (blinkCount == 0) return;
  unsigned long now = millis();
  if (now - lastLEDMillis < blinkInterval) return;
  lastLEDMillis = now;
  if (ledOn) {
    digitalWrite(LED_BUILTIN, LOW);
    ledOn = false;
    blinkCount--;
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
    ledOn = true;
  }
}


void pulseBNC() {
  digitalWrite(BNC1_pin, HIGH);
  delay(10);
  digitalWrite(BNC1_pin, LOW);
}


void handleValvePulsing() {
  if (activeBankNumber == -1) return;

  unsigned long currentMillis = millis();

  if (isValveCurrentlyOpen) {
    // Odor is ON — check if time to switch to blank
    if (currentMillis - previousValveTimer >= valveOnDuration) {
      activateOdorValve(bankBlank[activeBankNumber]);
      deactivateOdorValve(bankOdor[activeBankNumber]);
      isValveCurrentlyOpen = false;
      previousValveTimer = currentMillis;
      if (DEBUG) Serial.println("Pulse OFF (blank open)");
    }
  } else {
    // Blank is ON — check if time to switch to odor
    if (currentMillis - previousValveTimer >= valveOffDuration) {
      valveOnDuration  = random(valveOnMin, valveOnMax + 1);
      valveOffDuration = valveOnDuration / 4;
      activateOdorValve(bankOdor[activeBankNumber]);
      deactivateOdorValve(bankBlank[activeBankNumber]);
      isValveCurrentlyOpen = true;
      previousValveTimer = currentMillis;
      if (DEBUG) {
        Serial.print("Pulse ON (odor open) — ON=");
        Serial.print(valveOnDuration);
        Serial.print("ms OFF=");
        Serial.print(valveOffDuration);
        Serial.println("ms");
      }
    }
  }
}


void readFromSender() {
  static char buf[64];
  static int idx = 0;

  while (Serial3.available() > 0) {
    char inByte = Serial3.read();
    if (inByte == '\n' || inByte == ';') {
      buf[idx] = '\0';
      interpretMessage(String(buf));
      idx = 0;
    } else if (idx < 63) {
      buf[idx++] = inByte;
    }
  }
}


void interpretMessage(String message) {
  message.trim();
  int len = message.length();
  if (len==0) {
    if (DEBUG) Serial.println("#");
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

    case 'Z':
    case 'z': {
      int targetBank = controlMode ? (int)random(0, 3) : (int)arg1;
      if (targetBank < 0 || targetBank > 2) break;

      // Revert any currently active bank back to its blank
      if (activeBankNumber != -1 && activeBankNumber != targetBank) {
        if (isValveCurrentlyOpen) {
          activateOdorValve(bankBlank[activeBankNumber]);
          deactivateOdorValve(bankOdor[activeBankNumber]);
        }
        activeBankNumber = -1;
        isValveCurrentlyOpen = false;
      }

      // Activate target bank: open odor, close blank
      activateOdorValve(bankOdor[targetBank]);
      deactivateOdorValve(bankBlank[targetBank]);

      activeBankNumber = targetBank;
      isValveCurrentlyOpen = true;
      previousValveTimer = millis();

      if (DEBUG) {
        Serial.print("Zone ");
        Serial.print(targetBank);
        Serial.print(" activated: odor ");
        Serial.print(bankOdor[targetBank]);
        Serial.print(" open, blank ");
        Serial.print(bankBlank[targetBank]);
        Serial.println(" closed. Pulsing 2s ON / 0.5s OFF.");
      }
      break;
    }

    case 'M':
    case 'm': {
      // Mineral oil: revert any active bank back to blank, no odor flowing
      if (activeBankNumber != -1) {
        if (isValveCurrentlyOpen) {
          activateOdorValve(bankBlank[activeBankNumber]);
          deactivateOdorValve(bankOdor[activeBankNumber]);
        }
        activeBankNumber = -1;
        isValveCurrentlyOpen = false;
      }

      if (DEBUG) Serial.println("MINERAL OIL: reverted to blank.");
      break;
    }

    case 'D':
    case 'd':
      setMFCFlowRate(ODOR_MFC, arg1/1000.0);
      delay(250);
      if (DEBUG) {
        Serial.print("Setting ODOR flow to: ");
        Serial.print(arg1/1000.0);
        Serial.println(" lpm");
      }
      break;

    case 'R':
    case 'r':
      setMFCFlowRate(CARRIER_MFC, arg1/1000.0);
      delay(250);
      if (DEBUG) {
        Serial.print("Setting CARRIER flow to: ");
        Serial.print(arg1/1000.0);
        Serial.println(" lpm");
      }
      break;

    case 'A':
    case 'a':
      activateAuxValve(arg1);
      pulseBNC();
      break;

    case 'X':
    case 'x':
      deactivateAuxValve(arg1);
      pulseBNC();
      break;

    case 'O':
    case 'o':
      activateOdorValve(arg1);
      pulseBNC();
      break;

    case 'C':
    case 'c':
      deactivateOdorValve(arg1);
      pulseBNC();

      if (activeBankNumber != -1 && bankOdor[activeBankNumber] == arg1) {
        activeBankNumber = -1;
        isValveCurrentlyOpen = false;
      }
      if (DEBUG) {
        Serial.print("Closed odor valve: ");
        Serial.println(arg1);
      }
      break;

    case 'G':
    case 'g':
      // Start: open all blank valves, close all odor valves
      for (int i = 0; i < 3; i++) {
        activateOdorValve(bankBlank[i]);
        deactivateOdorValve(bankOdor[i]);
      }
      controlMode = (arg1 == 1);
      if (DEBUG) {
        Serial.print("START: all blank valves open. Control mode: ");
        Serial.println(controlMode ? "ON" : "OFF");
      }
      activeBankNumber = -1;
      isValveCurrentlyOpen = false;
      setMFCFlowRate(ODOR_MFC, odor_flow_rate);
      delay(250);
      // pulseBNC();
      if (DEBUG) Serial.println("START: all blank valves open.");
      // Blink LED to visually confirm actual MFC flow rate
      {
        float measuredFlow = getMFCFlowRate(ODOR_MFC);
        if (measuredFlow > 0) {
          blinkCount    = (int)round(measuredFlow);
          ledOn         = false;
          lastLEDMillis = millis();
        }
        if (DEBUG) {
          Serial.print("Measured flow rate: ");
          Serial.print(measuredFlow);
          Serial.println(" LPM");
        }
      }
      break;

    case 'S':
    case 's':
      // Stop: close all odor and blank valves, halt pulsing
      setMFCFlowRate(ODOR_MFC, 0.0);
      for (int i = 0; i < 3; i++) {
        deactivateOdorValve(bankOdor[i]);
        deactivateOdorValve(bankBlank[i]);
      }
      activeBankNumber = -1;
      isValveCurrentlyOpen = false;
      // pulseBNC();
      if (DEBUG) Serial.println("STOP: all valves closed.");
      break;

    case 'B':
    case 'b':
      digitalWrite(BNC1_pin, HIGH);
      delay(1000);
      digitalWrite(BNC1_pin, LOW);
      if (DEBUG) Serial.println("BNC1 pulsed high for 1 second");
      break;

    default:
      if (DEBUG) Serial.println("#");
  }
}
