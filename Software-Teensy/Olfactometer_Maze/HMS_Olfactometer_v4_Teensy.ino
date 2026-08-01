//check Serialn

#include "Olfactometer_header.h"

const bool DEBUG = false;

const int CARRIER_MFC = 2;
const int ODOR_MFC = 1;

int activeOdorValveNumber = -1; 
unsigned long previousValveTimer = 0;
bool isValveCurrentlyOpen = false;
const unsigned long valveOnDuration = 2000;  // 2 seconds ON
const unsigned long valveOffDuration = 500;  // 0.5 seconds OFF


void setup() {
  Serial.begin(115200);  
  Serial3.begin(115200);
  
  while (!Serial) {
      delay(100);
  }

  setupMFC(ODOR_MFC);
  InitializeValves();

  pinMode(BNC1_pin, OUTPUT);
  digitalWrite(BNC1_pin, LOW);
  
  if (DEBUG) {
    Serial.println("Olfactometer Receiver Ready.");
  }
}


void loop() {
  readFromSender();
  handleValvePulsing();
}


void pulseBNC() {
  digitalWrite(BNC1_pin, HIGH);
  delay(10);
  digitalWrite(BNC1_pin, LOW);
}


void handleValvePulsing() {
  if (activeOdorValveNumber == -1) {
    return;
  }

  unsigned long currentMillis = millis();

  if (isValveCurrentlyOpen) {
    // If it's ON, check if it is time to turn it OFF
    if (currentMillis - previousValveTimer >= valveOnDuration) {
      deactivateOdorValve(activeOdorValveNumber);
      isValveCurrentlyOpen = false;
      previousValveTimer = currentMillis;
      
      pulseBNC();
      if (DEBUG) Serial.println("Pulse OFF");
    }
  } else {
    // If it's OFF, check if it is time to turn it ON
    if (currentMillis - previousValveTimer >= valveOffDuration) {
      activateOdorValve(activeOdorValveNumber);
      isValveCurrentlyOpen = true;
      previousValveTimer = currentMillis;
      
      pulseBNC();
      if (DEBUG) Serial.println("Pulse ON");
    }
  }
}


void readFromSender() {
  static String incomingMessage = ""; 
  
  if (Serial3.available() > 0) {
    char inByte = Serial3.read();
    
    if ((inByte == '\n') || (inByte == ';')){
      interpretMessage(incomingMessage);
      incomingMessage = ""; 
    } else {
      incomingMessage = incomingMessage + inByte;
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
      int targetValve = -1;
      
      if (arg1 == 0) targetValve = 17;
      else if (arg1 == 1) targetValve = 1;
      else if (arg1 == 2) targetValve = 9;

      if (targetValve != -1) {
        if (activeOdorValveNumber != -1 && activeOdorValveNumber != targetValve) {
          deactivateOdorValve(activeOdorValveNumber);
        }

        activeOdorValveNumber = targetValve;
        activateOdorValve(activeOdorValveNumber);
        isValveCurrentlyOpen = true;
        previousValveTimer = millis();
        
        pulseBNC();

        if (DEBUG) {
          Serial.print("Zone ");
          Serial.print(arg1);
          Serial.println(" activated (Pulsing 2s ON / 0.5s OFF)");
        }
      }
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
      
      if (activeOdorValveNumber == arg1) {
        activeOdorValveNumber = -1; 
        isValveCurrentlyOpen = false;
      }
      if (DEBUG) {
        Serial.print("Closed odor valve: ");
        Serial.println(arg1);
      }
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