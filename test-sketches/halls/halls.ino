int PB1_LED_EN = 0; // on the shift registers
int PB2_LED_EN = 1; // on the shift registers
int PB3_LED_EN = 2; // on the shift registers

int PB1_SIG_PIN = 0; // on the mux - left blinker
int PB2_SIG_PIN = 1; // on the mux - right blinker
int PB3_SIG_PIN = 2; // on the mux - hazards

int HALL_SIG_PIN = 18; // on the arduino digital pins - an interrupt pin
int HALR_SIG_PIN = 19; // on the arduino digital pins - an interrupt pin



#define PRESSTIME 100

#define NUMTOGGLEBUTTONSLEDS 3 

#define LEFTBLINKSTATE 0 // the location of the left blinker's state in toggle_states
#define RIGHTBLINKSTATE 1
#define HAZARDBLINKSTATE 2

int toggle_buttons[NUMTOGGLEBUTTONSLEDS] = { PB1_SIG_PIN, PB2_SIG_PIN, PB3_SIG_PIN };
int toggle_button_LEDs[NUMTOGGLEBUTTONSLEDS] = { PB1_LED_EN, PB2_LED_EN, PB3_LED_EN }; 

bool toggle_states[NUMTOGGLEBUTTONSLEDS] = { false };
bool toggle_newpresses[NUMTOGGLEBUTTONSLEDS] = { true };
unsigned long toggle_downtimes[NUMTOGGLEBUTTONSLEDS] = { 0 };
unsigned long toggle_pressstarted[NUMTOGGLEBUTTONSLEDS] = { false };

bool left_turn_signal_state = 0;
bool right_turn_signal_state = 0;

#define number_of_74hc595s 2 // Number of Shift Registers
#define numOfRegisterPins number_of_74hc595s * 8
boolean registers[numOfRegisterPins];

// Shift Register Control Pins
int SREG_SER_PIN = 26; // on the arduino digital pins
int SREG_RCLK_PIN = 27; // on the arduino digital pins
int SREG_SRCLK_PIN = 28; // on the arduino digital pins
int SREG_SRCLR_PIN = 29; // on the arduino digital pins
int SREG_OE_PIN = 30; // on the arduino digital pins

// Mux control pins
int MUX_SIG_PIN = 12; // on the arduino analog pins
int MUX_S3_PIN = 31; // on the arduino digital pins
int MUX_S2_PIN = 32; // on the arduino digital pins
int MUX_S1_PIN = 33; // on the arduino digital pins
int MUX_S0_PIN = 34; // on the arduino digital pins
int MUX_EN_PIN = 35; // on the arduino digital pins

#define NUMBLINKVARS 4

#define BLINKSLOW 0
#define BLINKMEDIUM 1
#define BLINKFAST 2
#define BLINKFASTEST 3

int blink_increments[NUMBLINKVARS] = { 800, 700, 200, 50 }; 
unsigned long blink_times[NUMBLINKVARS] = { 0 };
bool blink_states[NUMBLINKVARS] = { false };

bool waiting_for_left = true;
bool waiting_for_right = true;
unsigned long left_time = 0;
unsigned long right_time = 0;


void setup() {

  Serial.begin(9600);
  
  pinMode(HALL_SIG_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_SIG_PIN), hall_isr, RISING);

  pinMode(HALR_SIG_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALR_SIG_PIN), halr_isr, RISING);

  pinMode(SREG_SER_PIN, OUTPUT);
  pinMode(SREG_RCLK_PIN, OUTPUT);
  pinMode(SREG_SRCLK_PIN, OUTPUT);
  pinMode(SREG_SRCLR_PIN, OUTPUT);
  pinMode(SREG_OE_PIN, OUTPUT);
  
  pinMode(MUX_S3_PIN, OUTPUT);
  pinMode(MUX_S2_PIN, OUTPUT);
  pinMode(MUX_S1_PIN, OUTPUT);
  pinMode(MUX_S0_PIN, OUTPUT);
  pinMode(MUX_EN_PIN, OUTPUT);

  digitalWrite(SREG_OE_PIN, LOW);
  digitalWrite(SREG_SRCLR_PIN, HIGH);

  //reset all register pins
  clearRegisters();
  writeRegisters();
}

void loop() {

  // set the state of the global blink variables
  for (int i = 0; i < NUMBLINKVARS; i++) {
    if (millis() - blink_times[i] > blink_increments[i]) {
      blink_states[i] = !blink_states[i];
      blink_times[i] = millis();
    }
  }
  
  // look for new presses on the buttons
  for (int i = 0; i < NUMTOGGLEBUTTONSLEDS; i++) {
    if (read_mux_button(toggle_buttons[i])) {
      if (toggle_pressstarted[i] == false) {
        toggle_pressstarted[i] = true;
        toggle_downtimes[i] = millis();
      }
      if (millis() - toggle_downtimes[i] > PRESSTIME) {
        if (toggle_newpresses[i]) {
          toggle_states[i] = !toggle_states[i];
          toggle_newpresses[i] = false;
        }
      }
    }
    else {
      toggle_pressstarted[i] = false;
      toggle_newpresses[i] = true;
    }
  }
  
  left_turn_signal_state = (toggle_states[LEFTBLINKSTATE] || toggle_states[HAZARDBLINKSTATE]) && blink_states[BLINKFAST]; 
  right_turn_signal_state = (toggle_states[RIGHTBLINKSTATE] || toggle_states[HAZARDBLINKSTATE]) && blink_states[BLINKFAST];
  
  setAndWrite(toggle_button_LEDs[LEFTBLINKSTATE], left_turn_signal_state);
  setAndWrite(toggle_button_LEDs[RIGHTBLINKSTATE], right_turn_signal_state);
  setAndWrite(toggle_button_LEDs[HAZARDBLINKSTATE], toggle_states[HAZARDBLINKSTATE] && blink_states[BLINKSLOW]);

}

void new_hal_edge(){

  if ((waiting_for_left == false) && (waiting_for_right == false)) {
    if (right_time < left_time) { // A right turn
      if (toggle_states[LEFTBLINKSTATE]){
        toggle_states[LEFTBLINKSTATE] = false;
      }
    }
    else if (right_time > left_time) { // A left turn
      if (toggle_states[RIGHTBLINKSTATE]){
        toggle_states[RIGHTBLINKSTATE] = false;
      }
    }
        
    waiting_for_left = true;
    waiting_for_right = true;
     
  }
}

void hall_isr() {
  left_time = millis();
  waiting_for_left = false;
  new_hal_edge();
}

void halr_isr() {
  right_time = millis();
  waiting_for_right = false;
  new_hal_edge();
}

void setAndWrite(int index, int value) {
  setRegisterPin(index, value);
  writeRegisters();
}

void clearRegisters() {
  for (int i = numOfRegisterPins - 1; i >= 0; i--) {
    registers[i] = LOW;
  }
}

void writeRegisters() {

  digitalWrite(SREG_RCLK_PIN, LOW);

  for (int i = numOfRegisterPins - 1; i >= 0; i--) {

    digitalWrite(SREG_SRCLK_PIN, LOW);

    int val = registers[i];

    digitalWrite(SREG_SER_PIN, val);
    digitalWrite(SREG_SRCLK_PIN, HIGH);
  }
  digitalWrite(SREG_RCLK_PIN, HIGH);
}

void setRegisterPin(int index, int value) {
  //set an individual pin HIGH or LOW
  registers[index] = value;
}

bool ar_to_dr(int ar_value) {
  // turns an analog read to a binary digital value

  if (ar_value > 512) {
    return true;
  }
  else {
    return false;
  }
}

int read_mux(int channel) {

  int controlPin[] = { MUX_S0_PIN, MUX_S1_PIN, MUX_S2_PIN, MUX_S3_PIN };

  int muxChannel[16][4] = {
    { 0,0,0,0 }, //channel 0
    { 1,0,0,0 }, //channel 1
    { 0,1,0,0 }, //channel 2
    { 1,1,0,0 }, //channel 3
    { 0,0,1,0 }, //channel 4
    { 1,0,1,0 }, //channel 5
    { 0,1,1,0 }, //channel 6
    { 1,1,1,0 }, //channel 7
    { 0,0,0,1 }, //channel 8
    { 1,0,0,1 }, //channel 9
    { 0,1,0,1 }, //channel 10
    { 1,1,0,1 }, //channel 11
    { 0,0,1,1 }, //channel 12
    { 1,0,1,1 }, //channel 13
    { 0,1,1,1 }, //channel 14
    { 1,1,1,1 }  //channel 15
  };

  //loop through the 4 sig
  for (int i = 0; i < 4; i++) {
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  int val = analogRead(MUX_SIG_PIN);

  return val;
}

int read_mux_button(int channel) {
  return ar_to_dr(read_mux(channel));
}


