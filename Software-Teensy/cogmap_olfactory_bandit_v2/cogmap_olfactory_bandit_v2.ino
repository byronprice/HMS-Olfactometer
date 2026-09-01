/*
read data from a photo interrupter (beam break) and use it to
 control a solenoid that allows water to flow (for a water reward)

 v2: occasionally double the reward, but only while the mouse is exploiting the
 hot patch as intended. The valve is held open water_multiplier times as long,
 which delivers roughly twice the water (see water_multiplier -- the solenoid is
 not proportional to open time).

 Eligibility is a VARIABLE-RATIO schedule: it takes poke_threshold consecutive
 pokes in the hot patch, where poke_threshold is itself redrawn at random after
 every double and whenever the hot patch switches. See the poke_threshold block
 for why a fixed threshold is the wrong choice here.

 patch_poke_count resets when the mouse leaves the patch, after each double, and
 on a hot-patch switch -- but NOT on a block transition that happens to leave the
 hot patch where it was.

 Two columns are appended to the serial log; reward1..6 / catch1..6 keep their
 existing 0/1 meaning:
   pokecount  consecutive pokes in the current patch as of this trial, so every
              trial carries its exploitation depth. Controls for a doubled trial
              are non-doubled trials at matched pokecount.
   doubled    1 when this delivery's valve time was actually doubled.
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
  {0.0,0.0},
  {0.0,0.0},
  {0.95,0.95}
};
// Pair rewardContPairs[n_reward_pairs] = {
//   {0.3,0.3},
//   {0.5,0.3},
//   {0.8,0.4},
//   {0.5,0.5},
//   {0.6,0.8},
//   {0.8,0.8}
// };

// index into rewardContPairs of the "hot" (high reward rate) pair. this is the
// same pair whose patch gets the odor cue revealed over Serial5. NOTE: only
// meaningful while exactly one pair is the high-rate one -- if the 6-pair
// asymmetric config above is restored, the reward doubling gate needs revisiting.
const int hot_pair_index = n_reward_pairs - 1;

// change_reward_contingency() shuffles all n_reward_pairs but its assignment
// loop only reads indices[0 .. n_patches-1]. If there are more pairs than
// patches, hot_pair_index is dealt to a patch only n_patches/n_reward_pairs of
// the time; on the other blocks the `indices[i] == hot_pair_index` test never
// fires, so pending_patch_signal stays -1 (no odor reveal is sent, the
// olfactometer sits on mineral oil for the whole block) and hot_patch silently
// keeps its value from the PREVIOUS block, pointing the reward-doubling gate at
// the wrong patch. Both failures are invisible in the logs. This is currently
// safe (3 pairs, 3 patches) and this assert keeps it that way -- it trips if the
// 6-pair config above is uncommented, or if total_setups changes (n_patches is
// derived from it, so a 4-port maze would give 2 patches and break this too).
static_assert(n_reward_pairs <= n_patches,
              "more reward pairs than patches: hot_pair_index would not be dealt "
              "every block, leaving hot_patch stale and suppressing the odor "
              "reveal. Derive the hot patch from the assigned probabilities "
              "instead of from a fixed index -- see the note above.");

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

// reward doubling variables
//
// VARIABLE-RATIO schedule: the number of consecutive hot-patch pokes required
// is itself random, redrawn after every double and whenever the hot patch
// changes. A fixed threshold would be a fixed-ratio schedule -- the most
// learnable kind -- and the mouse could farm it by lingering in the patch,
// which would contaminate exactly the patch-residence measures the task exists
// to measure. A random threshold leaves nothing to count.
//
// Calibrated against 43 real sessions of experiment14 behavior. Empirically the
// mean consecutive hot-patch run is only 2.23 pokes (median 2), so thresholds
// must live in the 3-8 range: a fixed threshold of 10 fires 0.74x per session
// with 77% of sessions getting none at all. T = 2 + min(geom(1/2), 6) gives
// ~11 doubles/session (median 7), with 7% of sessions getting none -- those
// are the short sessions (~65 trials vs ~330).
const int poke_threshold_min = 3;
const float poke_threshold_geo = 1.0/2.0;
const int poke_threshold_cap = 10;   // T in {3..8}
int poke_threshold = 4;             // redrawn by draw_poke_threshold()
// NOT 2.0. The solenoid is affine, not proportional, in open time: it has a
// fixed opening dead time t0, so V(t) ~ (t - t0) and 2x the open time gives
// well over 2x the water. Measured in /n/groups/datta/jgrossi/olfactory_maze/
// hardware/solenoid_test: t0 = 8.7-10.6 ms across valves/days while the flow
// rate itself varies 1.36x. The multiplier needed for 2x volume is
// m = 2 - t0/t, which depends on t0 but NOT on flow rate -- so one constant
// transfers across valves. At the 26-34 ms open times these ports calibrate
// to for 4 uL, m = 1.7 delivers 1.99-2.04x the volume (1.8 -> ~2.15x,
// 2.0 -> ~2.45x). Revisit if the calibrated release times move far out of
// that range, since m depends on the baseline t.
const float water_multiplier = 1.7;
int hot_patch = -1;              // patch currently holding rewardContPairs[hot_pair_index]
int patch_poke_count = 0;        // consecutive pokes in the current patch
bool double_water[total_setups] = {false,false,false,false,false,false}; // this delivery is doubled
int logged_poke_count[total_setups] = {0,0,0,0,0,0}; // count at trial registration, for the log

// debounce, to prevent repeated reward delivery within some window
unsigned long timeout[total_setups];
unsigned long max_timeout = 500; // 500 milliseconds
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
const unsigned long contingency_pause_duration = 3000;
const unsigned long mineral_oil_duration = 2000; // how long to sit on mineral oil before revealing new high-reward patch
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

      print_to_serial(check_time,i,-1,-1,0,0,0,0);
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

      // count consecutive pokes inside a single patch. a poke in a different
      // patch restarts the count at 1. a same-patch poke is necessarily at the
      // other port of the pair, since a repeat of prev_reward_port never
      // registers a trial. evaluated BEFORE prev_reward_port is overwritten and
      // BEFORE change_reward_contingency(), so it sees this block's hot patch
      // and the block swap's reset lands afterward.
      int cur_patch = i / 2;
      int prev_patch = (prev_reward_port >= 0) ? (prev_reward_port / 2) : -1;
      patch_poke_count = (cur_patch == prev_patch) ? patch_poke_count + 1 : 1;
      logged_poke_count[i] = patch_poke_count;  // capture before any reset below

      bool double_this_trial = false;
      // strictly less-than: random(0,100000) can return 0, so randomU can be
      // exactly 0.0 and "<=" would pay out 1 trial in 100000 at a port whose
      // probability is 0.0. with "<" a 0.0 pair is truly never rewarded, and a
      // 0.95 pair is exactly 0.95 rather than 0.95001.
      if ((randomU < reward_prob[i]) && (reward_count < max_rewards)) {
        // water IS being delivered on this trial, so now decide whether to
        // double it. deciding here rather than above means the threshold is
        // only ever spent on a trial that actually delivers water -- an
        // unlucky catch no longer burns a run the mouse earned, and it also
        // stops the schedule idling once the daily water cap is reached.
        if ((cur_patch == hot_patch) && (patch_poke_count >= poke_threshold)) {
          double_this_trial = true;
          patch_poke_count = 0;
          draw_poke_threshold();
        }
        digitalWrite(waterPins[i], HIGH);
        reward_given[i] = true;
        double_water[i] = double_this_trial;
        // a doubled reward counts twice, so max_rewards stays a water volume cap
        reward_count = reward_count + (double_this_trial ? 2 : 1);
      }
      timeout[i] = millis();

      trial_count = trial_count + 1;
      change_reward_contingency();
      prev_reward_value = (float) reward_prob[i];
      prev_reward_port = (int) i;

      reward_prob[i] = 0.0;

    } else if ((give_water[i] == true) && (check_debounce[i] == false)) {
      check_time = millis();

      unsigned long release = water_release_time[i];
      if (double_water[i] == true) {
        // water_multiplier is calibrated to double the VOLUME, not the time
        release = (unsigned long) (release * water_multiplier + 0.5);
      }
      // the valve can only be closed while check_debounce[i] is false, which
      // expires max_timeout after the valve opened. if the open time ever
      // exceeded that, give_water[i] would still be true when the debounce
      // cleared and the branch above would re-fire, re-registering a trial and
      // re-opening the valve. clamp so that is impossible.
      if (release > (max_timeout - 100)) {
        release = max_timeout - 100;
      }

      if ((check_time - water_valve_time[i]) >= release) {
        digitalWrite(waterPins[i], LOW);
        give_water[i] = false;

        int double_flag = (double_water[i] == true) ? 1 : 0;

        if (reward_given[i] == true) {
          //reward
          print_to_serial(check_time,-1,i,-1,0,0,logged_poke_count[i],double_flag);
        } else {
          //catch
          print_to_serial(check_time,-1,-1,i,0,0,logged_poke_count[i],double_flag);
        }
        reward_given[i] = false;
        double_water[i] = false;
        logged_poke_count[i] = 0;
      }
    }
  }
}

// consecutive hot-patch pokes required for the next double. same geometric
// idiom the block length uses, so T = poke_threshold_min + min(geom, cap).
void draw_poke_threshold() {
  float u = random(1, 100000) / 100000.0;
  int x = ceil(log(1 - u) / log(1 - poke_threshold_geo));
  poke_threshold = poke_threshold_min + min(x, poke_threshold_cap);
}

void change_reward_contingency() {
  int trials_since_swap = trial_count - last_swap_trial_count;

  if (trials_since_swap >= reward_swap) {
    last_swap_trial_count = trial_count;

    check_time = millis();
    Serial.print("rc,");
    Serial.print(check_time);
    Serial.print(",");

    // NB: the pause and the olfactometer handoff are NOT started here -- we do
    // not yet know which patch the shuffle will hand the hot pair to. Both are
    // decided after the assignment loop below, once new_hot is known.
    int prev_hot = hot_patch;
    int new_hot = -1;

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

      if (indices[i] == hot_pair_index) {
        new_hot = i;
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
    hot_patch = new_hot;

    // Everything that interrupts the session happens ONLY when the hot patch
    // actually moved. The shuffle hands it back to the same patch ~1/3 of blocks
    // (36% observed); with the current pair config -- {0,0},{0,0},{0.95,0.95} --
    // the two cold pairs are identical, so a same-patch swap leaves every port's
    // probability exactly as it was. There is nothing for the mouse to relearn
    // and nothing new to smell, so pausing and re-cueing the odor would be pure
    // dead time. The run also continues uninterrupted, so the doubling counter
    // and threshold carry over rather than resetting.
    //
    // WARNING: this shortcut assumes the non-hot pairs are interchangeable. If
    // the two cold pairs are ever given DIFFERENT values, a same-hot-patch swap
    // still reshuffles them between the cold patches -- a real contingency
    // change that this would skip the pause and the odor re-cue for.
    if (new_hot != prev_hot) {
      patch_poke_count = 0;
      draw_poke_threshold();

      contingency_pause = true;
      contingency_pause_start = millis();
      patch_signal_sent = false;
      pending_patch_signal = new_hot;
      Serial5.println("M");
    }

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
    print_to_serial(check_time,-1,-1,-1,1,0,0,0);
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
      print_to_serial(check_time,-1,-1,-1,0,1,0,0);
    }
    sync_off_time = (unsigned long) random(sync_off_params[0], sync_off_params[1]);
  } else if ((current_time-prev_sync_time >= sync_on_time) && (sync_state == 1)) {
    sync_state = 0;
    digitalWrite(sync_trigger_pin, LOW);
    prev_sync_time = current_time;
  }
}

void print_to_serial(unsigned long timestamp, int beam_break_setup, int reward_setup, int catch_setup, int camera_frame, int sync, int pokecount, int doubled) {
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
  Serial.print(",");
  Serial.print(pokecount);
  Serial.print(",");
  Serial.print(doubled);
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
    // seed from the microsecond clock at the moment the experimenter starts the
    // session. without this the Teensy replays one fixed random stream from
    // every power-up/reflash, so the hot-patch sequence, the reward draws and
    // the doubling thresholds repeat across sessions and sessions are not
    // independent samples. must happen before draw_poke_threshold() and the
    // first change_reward_contingency() below, which both consume random().
    randomSeed(micros());

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
    Serial.print(",pokecount,doubled");
    Serial.println();

    prime_ports();

    reward_count = 0;
    camera_state = 0;
    trial_count = 0;
    last_swap_trial_count = -reward_swap;
    prev_camera_time = millis();
    prev_sync_time = millis();

    // no draw_poke_threshold() here on purpose. hot_patch = -1 makes the first
    // block count as a hot-patch switch, so change_reward_contingency() below
    // (which always fires, since last_swap_trial_count = -reward_swap) draws the
    // threshold itself. drawing here too would be dead -- immediately
    // overwritten -- and would burn a random() call, shifting the stream.
    patch_poke_count = 0;
    hot_patch = -1;
    for (int i = 0; i < total_setups; i = i+1) {
      double_water[i] = false;
      logged_poke_count[i] = 0;
    }

    print_to_serial(prev_camera_time,-1,-1,-1,0,0,0,0);

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
      delay(100);
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
    print_to_serial(current_time,-1,-1,-1,0,0,0,0);

    for (int i = 0; i < total_setups; i = i+1) {
      digitalWrite(waterPins[i], LOW);
      double_water[i] = false;
      logged_poke_count[i] = 0;
    }
    patch_poke_count = 0;

    Serial5.println("S");
  }
}
