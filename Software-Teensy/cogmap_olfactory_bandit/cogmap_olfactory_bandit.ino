/*
read data from a photo interrupter (beam break) and use it to 
 control a solenoid that allows water to flow (for a water reward)
*/

// constants won't change:
const int total_setups = 6;
const int waterPins[total_setups] = {33,34,35,36,37,38};
const int photoPins[total_setups] = {13,14,15,16,17,18};
const int camera_trigger_pin = 23;
const int sync_trigger_pin = 22;
const int n_patches = total_setups / (int) 2;

// tube clearing variables, to start the day, prime the tubes, clear bubbles
const unsigned long tube_clear_release_time = 2000; // 1 second

// Variables will change:

// for reward contingency swap
struct Pair {
  float first;
  float second;
};

const int n_reward_pairs = 3;
Pair rewardContPairs[n_reward_pairs] = {
  {0.1,0.1},
  {0.1,0.1},
  {0.9,0.9}
};
// Pair rewardContPairs[n_reward_pairs] = {
//   {0.3,0.3},
//   {0.5,0.3},
//   {0.8,0.4},
//   {0.5,0.5},
//   {0.6,0.8},
//   {0.8,0.8}
// };

// beam break variables
bool beam_state[total_setups] = {false,false,false,false,false,false};
unsigned long time_broken[total_setups] = {0,0,0,0,0,0}; // measure how long beam has actually been broken
unsigned long beam_break_time = 40; // milliseconds, how long beam must be broken to trigger water

// water reward variables
bool give_water[total_setups] = {false,false,false,false,false,false}; // signal to provide water when beam break happens for more than 
    // beam_break_time milliseconds
unsigned long water_release_time[total_setups] = {30,30,30,30,30,30}; // 30 milliseconds, how long water is released for, equivalent to 5 uL volume
unsigned long water_valve_time[total_setups];
const int max_rewards = 275;
int reward_count = 0;
int trial_count = 0;

// debounce, to prevent repeated reward delivery within some window
unsigned long timeout[total_setups];
unsigned long max_timeout = 500; // 5 seconds
bool check_debounce[total_setups] = {true,true,true,true,true,true}; 

// state variables
unsigned long check_time;
unsigned long serial_wait_time = 500; // 500 milliseconds between checks on the serial port
unsigned long prev_serial_check;
int session_state = 0; // 0 is waiting to start experimental session, 1 is active

// reward port probabilities
int reward_swap = 10; // change reward contingency every 10 trials
int reward_swap_min = 10;
int last_swap_trial_count = 0; // trial_count value at the most recent swap
float reward_swap_geo_mean = 1.0/5.0; // reward swap happens every 10 + geometric(5) trials
float reward_prob[total_setups] = {0.0,0.0,0.0,0.0,0.0,0.0};
bool reward_given[total_setups] = {false,false,false,false,false,false};

// int reward_partners[total_setups] = {1,0,3,2,5,4};
int prev_reward_port = -1;
float prev_reward_value = 0.9;

// reward contingency delay
bool contingency_pause = false;
unsigned long contingency_pause_start = 0;
const unsigned long contingency_pause_duration = 15000;
const unsigned long mineral_oil_duration = 13000; // how long to sit on mineral oil before revealing new high-reward patch
bool patch_signal_sent = false;
int pending_patch_signal = -1;

// camera parameters
unsigned long prev_camera_time;
unsigned long camera_on_time = 10;
unsigned long camera_off_time = 20;
int camera_state = 0;

// sync pulse parameters
unsigned long prev_sync_time;
unsigned long sync_on_time = 530;
unsigned long sync_off_time = 470;
unsigned long sync_off_params[2] = {500,1500};
int sync_state = 0;

void setup() {
  // set the digital pins as input/output:
  for (int i = 0; i < total_setups; i = i+1) {
    pinMode(waterPins[i], OUTPUT);
    pinMode(photoPins[i], INPUT);
    digitalWrite(waterPins[i], LOW);
    time_broken[i] = millis();
    timeout[i] = millis() + max_timeout;
  }
  pinMode(camera_trigger_pin, OUTPUT);
  pinMode(sync_trigger_pin, OUTPUT);

  digitalWrite(camera_trigger_pin, LOW);
  digitalWrite(sync_trigger_pin, LOW);

  prev_serial_check = millis();
  prev_camera_time = millis();
  prev_sync_time = millis();

  Serial.begin(115200);
  Serial5.begin(115200);
}

void loop() {
  check_time = millis();

  sync_pulse(session_state, check_time);
  if (session_state == 1) {
    trigger_camera();
  }
  
  if (session_state == 1) {
    check_beam_break();
    emit_reward();
    debounce();
    enforce_contingency_pause();
    
  } else if (session_state == 2) {
    beam_break_test();
  }
  
  if (check_time - prev_serial_check >= serial_wait_time) {
    prev_serial_check = check_time;
    check_USB();
  }
}

void beam_break_test() {
  unsigned long total_time_broken;
  for (int i = 0; i < total_setups; i = i+1) {
    int read_beam_break = digitalRead(photoPins[i]);
    check_time = millis();
    if (read_beam_break == LOW) {
      total_time_broken = check_time - time_broken[i];
    } else {
      time_broken[i] = millis();
      total_time_broken = 0;
      beam_state[i] = false;
    }

    if ((total_time_broken >= beam_break_time) && (beam_state[i] == false)) {
      digitalWrite(waterPins[i], HIGH);
      delay(50);
      digitalWrite(waterPins[i], LOW);

      beam_state[i] = true;
    }
  }
}

void check_beam_break() {
  unsigned long total_time_broken;
  for (int i = 0; i < total_setups; i = i+1) {
    int read_beam_break = digitalRead(photoPins[i]);
    check_time = millis();
    if (read_beam_break == LOW) {
      total_time_broken = check_time - time_broken[i];
    } else {
      time_broken[i] = millis();
      total_time_broken = 0;
      beam_state[i] = false;
    }

    if ((total_time_broken >= beam_break_time) && (beam_state[i] == false)) {
      if ((check_debounce[i] == true) && (contingency_pause == false)) {

        if (prev_reward_port < 0) {
          give_water[i] = true;
        } else if (prev_reward_port != i) {
          reward_prob[prev_reward_port] = prev_reward_value;
          give_water[i] = true; 
        }
      }
      
      print_to_serial(check_time,i,-1,-1,0,0);
      beam_state[i] = true;
      time_broken[i] = millis();
    }
  }
}

void emit_reward() {
  for (int i = 0; i < total_setups; i = i+1) {
    if ((give_water[i] == true) && (check_debounce[i] == true)) {
      check_time = millis();
      check_debounce[i] = false;
      float randomU = (float) random(0,100000) / 100000;
      water_valve_time[i] = millis();

      if ((randomU <= reward_prob[i]) && (reward_count < max_rewards)) {
        digitalWrite(waterPins[i], HIGH);
        reward_given[i] = true;
        reward_count = reward_count+1;
      } 
      timeout[i] = millis();

      trial_count = trial_count + 1;
      change_reward_contingency();
      prev_reward_value = (float) reward_prob[i];
      prev_reward_port = (int) i;

      reward_prob[i] = 0.0;

    } else if ((give_water[i] == true) && (check_debounce[i] == false)) {
      check_time = millis();
      if ((check_time - water_valve_time[i]) >= water_release_time[i]) {
        digitalWrite(waterPins[i], LOW);
        give_water[i] = false;

        if (reward_given[i] == true) {
          //reward
          print_to_serial(check_time,-1,i,-1,0,0);
        } else {
          //catch
          print_to_serial(check_time,-1,-1,i,0,0);
        }
        reward_given[i] = false;
      }
    } 
  }
}

void change_reward_contingency() {
  int trials_since_swap = trial_count - last_swap_trial_count;

  if (trials_since_swap >= reward_swap) {
    last_swap_trial_count = trial_count;

    Serial.print("rc,");
    Serial.print(check_time);
    Serial.print(",");

    contingency_pause = true;
    contingency_pause_start = millis();
    patch_signal_sent = false;
    pending_patch_signal = -1;
    Serial5.println("M");

    int indices[n_reward_pairs];
    for (int i=0; i<n_reward_pairs;i = i+1) {
      indices[i] = i;
    }
    for (int i=n_reward_pairs-1; i > 0; i=i-1) {
      int j = random(i+1);
      int temp = indices[i];
      indices[i] = indices[j];
      indices[j] = temp;
    }

    int reward_prob_index = 0;
    for (int i=0; i < n_patches; i=i+1) {
      Pair p = rewardContPairs[indices[i]];

      bool coin_flip = random(2);

      if (coin_flip) {
        float temp = p.first;
        p.first = p.second;
        p.second = temp;
      }

      if (indices[i] == 2) {
        pending_patch_signal = i;
      }

      reward_prob[reward_prob_index] = p.first;
      Serial.print(reward_prob[reward_prob_index]);
      Serial.print(",");
      reward_prob_index = reward_prob_index + 1;

      reward_prob[reward_prob_index] = p.second;
      Serial.print(reward_prob[reward_prob_index]);
      Serial.print(",");
      reward_prob_index = reward_prob_index + 1;
    }
    Serial.println();
    float u = random(1, 100000) / 100000.0;
    int x = ceil(log(1 - u) / log(1 - reward_swap_geo_mean));
    reward_swap = reward_swap_min + min(x, 30);
  }
}

void enforce_contingency_pause(){
  if (contingency_pause) {
    unsigned long elapsed = millis() - contingency_pause_start;

    if ((patch_signal_sent == false) && (elapsed >= mineral_oil_duration) && (pending_patch_signal >= 0)) {
      Serial5.print("z");
      Serial5.println(pending_patch_signal);
      patch_signal_sent = true;
    }

    if (elapsed >= contingency_pause_duration) {
        contingency_pause = false;
    } else {
        return;
    }
  }
}

void debounce() {
  for (int i = 0; i < total_setups; i = i+1) {
    if (check_time - timeout[i] >= max_timeout) {
      check_debounce[i] = true;
    }
  }
}

void prime_ports() {
  for (int i=0; i < total_setups; i = i+1) {
    digitalWrite(waterPins[i], HIGH);
    delay(water_release_time[i]);
    digitalWrite(waterPins[i], LOW);
  }
}

void trigger_camera() {
  check_time = millis();
  if ((check_time-prev_camera_time >= camera_off_time) && (camera_state == 0)) {
    camera_state = 1;
    digitalWrite(camera_trigger_pin, HIGH);
    prev_camera_time = check_time;
    print_to_serial(check_time,-1,-1,-1,1,0);
  } else if ((check_time-prev_camera_time >= camera_on_time) && (camera_state == 1)) {
    camera_state = 0;
    digitalWrite(camera_trigger_pin, LOW);
    prev_camera_time = check_time;
  }
}

void sync_pulse(int send_serial, unsigned long current_time){
  if ((current_time-prev_sync_time >= sync_off_time) && (sync_state == 0)) {
    sync_state = 1;
    digitalWrite(sync_trigger_pin, HIGH);
    prev_sync_time = current_time;

    if (send_serial == 1) {
      print_to_serial(check_time,-1,-1,-1,0,1);
    }
    sync_off_time = (unsigned long) random(sync_off_params[0], sync_off_params[1]);
  } else if ((current_time-prev_sync_time >= sync_on_time) && (sync_state == 1)) {
    sync_state = 0;
    digitalWrite(sync_trigger_pin, LOW);
    prev_sync_time = current_time;
  }
}

void print_to_serial(unsigned long timestamp, int beam_break_setup, int reward_setup, int catch_setup, int camera_frame, int sync) {
  Serial.print(timestamp);
  Serial.print(",");
  Serial.print(camera_frame);
  Serial.print(",");
  Serial.print(sync);
  Serial.print(",");
  for (int i = 0; i < total_setups; i = i+1) {
    if (beam_break_setup == i) {
      Serial.print(1);
    } else {
      Serial.print(0);
    }
    Serial.print(",");
    if (reward_setup == i) {
      Serial.print(1);
    } else {
      Serial.print(0);
    }
    Serial.print(",");
    if (catch_setup == i) {
      Serial.print(1);
    } else {
      Serial.print(0);
    }
    if (i != (total_setups-1)) {
      Serial.print(",");
    }
  }
  Serial.println();
}

void check_USB() {
  // read from USB, if available
  static String usbMessage = "";  // initialize usbMessage to empty string,
                    // happens once at start of program
  while (Serial.available() > 0) {
    // keep reading the next char, while some are still available
    char inByte = Serial.read();
    if ((inByte == '\n') || (inByte == '\r'))  {
      // the new-line character ('\n') or carriage return ('\r')
      // indicates a complete message,
      // so interpret the message and then clear buffer
      if (usbMessage.length() > 0) {
        interpretCommand(usbMessage);
        usbMessage = ""; // clear message buffer
        break; // exit while loop after interpreting a command
      }
    } else {
      // append character to message buffer
      usbMessage = usbMessage + inByte;
    }
  }
}

void check_Serial5() {
  // read from USB, if available
  static String usbMessage = "";  // initialize usbMessage to empty string,
                    // happens once at start of program
  while (Serial5.available() > 0) {
    // keep reading the next char, while some are still available
    char inByte = Serial5.read();
    if ((inByte == '\n') || (inByte == '\r'))  {
      // the new-line character ('\n') or carriage return ('\r')
      // indicates a complete message,
      // so interpret the message and then clear buffer
      if (usbMessage.length() > 0) {
        interpretCommand(usbMessage);
        usbMessage = ""; // clear message buffer
        break; // exit while loop after interpreting a command
      }
    } else {
      // append character to message buffer
      usbMessage = usbMessage + inByte;
    }
  }
}


void interpretCommand(String message) {
  message.trim(); // remove leading and trailing white space
  int len = message.length();
  if (len==0) {
    Serial.println("#"); // "#" means error
    return;
  }
  char command = message[0]; // the command is the first char of a message

  // Extract characters 2–N of the message (to parse parameters)
  message.remove(0,1);

  // parse parameters as integers, separated by underscores
  String intString = "";
  unsigned long args[total_setups+1];
  int count = 0;
  while (message.length() > 0) {
    if (isDigit(message[0])) {
      intString += message[0];
      message.remove(0,1);
    } else if (message[0] == '_') {
      args[count] = intString.toInt();
      message.remove(0,1);
      intString = "";
      count = count + 1;
    } else {
      break;
    }
  }

  // S: start session
  if ((command == 'S') || (command == 's')) {
    session_state = 1;
    Serial.print("timestamp,");
    Serial.print("camera_frame,");
    Serial.print("sync_on,");

    Serial5.print("G");
    Serial5.println(args[total_setups]);

    for (int i = 0; i < total_setups; i = i+1) {
      water_release_time[i] = args[i];
      Serial.print("break");
      Serial.print(i+1);
      Serial.print(",reward");
      Serial.print(i+1);
      Serial.print(",catch");
      Serial.print(i+1);
      if (i != (total_setups-1)) {
        Serial.print(",");
      }
    }
    Serial.println();

    prime_ports();

    reward_count = 0;
    camera_state = 0;
    trial_count = 0;
    last_swap_trial_count = -reward_swap;
    prev_camera_time = millis();
    prev_sync_time = millis();

    print_to_serial(prev_camera_time,-1,-1,-1,0,0);

    change_reward_contingency();
  
  } else if ((command == 'T' ) || (command == 't')) {
    // drain water from tubes, to prime the system and remove bubbles
    for (int i=0; i < total_setups; i = i+1) {
      digitalWrite(waterPins[i], HIGH);
    }
    delay(tube_clear_release_time);
    for (int i=0; i < total_setups; i = i+1) {
      digitalWrite(waterPins[i], LOW);
    }

  } else if ((command == 'C') || (command == 'c')) {
    // calibrate water release
    int port = (int) args[0];
    int n_iters = (int) args[2];

    int count = 0;
    while (count < n_iters) {
      digitalWrite(waterPins[port], HIGH);
      delay(args[1]);
      digitalWrite(waterPins[port], LOW);
      delay(10);
      count = count + 1;
    }

  } else if ((command == 'B' ) || (command == 'b')) {
    session_state = 2;
  } else if ((command == 'D' ) || (command == 'd')) {
    session_state = 0;
  } else if ((command == 'E') || (command == 'e')) {
    // E: end session
    session_state = 0;
    camera_state = 0;
    digitalWrite(camera_trigger_pin, LOW);
    prev_reward_port = -1;
    unsigned long current_time = millis();
    print_to_serial(current_time,-1,-1,-1,0,0);

    for (int i = 0; i < total_setups; i = i+1) {
      digitalWrite(waterPins[i], LOW);
    }

    Serial5.println("S");
  }
}