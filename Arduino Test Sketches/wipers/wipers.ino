#define WIPEROFF 0
#define WIPERLOW 1
#define WIPERHIGH 2

int wiperstate = WIPEROFF; 
unsigned long previous_wiperbutton_time = 0;
bool looking_for_wiper_button = true;

int PB4_SIG_PIN = 3; // on the mux
int PB4_LED_EN = 3; // on the shift registers

int WIPEO_EN_PIN = 9; // on the arduino digital pins
int WIPEL_EN_PIN = 10; // on the arduino digital pins
int WIPEH_EN_PIN = 11; // on the arduino digital pins

// Shift Register Control Pins
int SREG_SER_PIN = 26; // on the arduino digital pins
int SREG_RCLK_PIN = 27; // on the arduino digital pins
int SREG_SRCLK_PIN = 28; // on the arduino digital pins
int SREG_SRCLR_PIN = 29; // on the arduino digital pins
int SREG_OE_PIN = 30; // on the arduino digital pins

int MUX_SIG_PIN = 12; // on the arduino analog pins
int MUX_S3_PIN = 31; // on the arduino digital pins
int MUX_S2_PIN = 32; // on the arduino digital pins
int MUX_S1_PIN = 33; // on the arduino digital pins
int MUX_S0_PIN = 34; // on the arduino digital pins
int MUX_EN_PIN = 35; // on the arduino digital pins

#define number_of_74hc595s 2 // Number of Shift Registers
#define numOfRegisterPins number_of_74hc595s * 8
boolean registers[numOfRegisterPins];

#define NUMBLINKVARS 4

#define BLINKSLOW 0
#define BLINKMEDIUM 1
#define BLINKFAST 2
#define BLINKFASTEST 3

int blink_increments[NUMBLINKVARS] = { 800, 700, 200, 50 }; 
unsigned long blink_times[NUMBLINKVARS] = { 0 };
bool blink_states[NUMBLINKVARS] = { false };

#define PRESSTIME 30

int count = 0;

void setup() {
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

  pinMode(WIPEO_EN_PIN, OUTPUT);
  pinMode(WIPEL_EN_PIN, OUTPUT);
  pinMode(WIPEH_EN_PIN, OUTPUT);

  digitalWrite(SREG_OE_PIN, LOW);
  digitalWrite(SREG_SRCLR_PIN, HIGH);

  digitalWrite(WIPEO_EN_PIN, LOW);
  digitalWrite(WIPEL_EN_PIN, LOW);
  digitalWrite(WIPEH_EN_PIN, LOW);


  Serial.begin(9600);

}

void loop() {

  Serial.println(count++);

  for (int i = 0; i < NUMBLINKVARS; i++) {
    if (millis() - blink_times[i] > blink_increments[i]) {
      blink_states[i] = !blink_states[i];
      blink_times[i] = millis();
    }
  }

  
  if (read_mux_button(PB4_SIG_PIN)) {
    debug_print("new press", true);
    if (looking_for_wiper_button) {
      looking_for_wiper_button = false;
      if (millis() - previous_wiperbutton_time > PRESSTIME) {
        debug_print("incrementing", true);
        previous_wiperbutton_time = millis();
        wiperstate++;
        if (wiperstate > WIPERHIGH) {
          wiperstate = WIPEROFF;
        }
      }
    }
  }
  else {
    looking_for_wiper_button = true;
  }


  switch (wiperstate) {

    case WIPEROFF:
    {
      delay(100);
      digitalWrite(WIPEL_EN_PIN, LOW);
      delay(100);
      digitalWrite(WIPEH_EN_PIN, LOW);
      delay(100);
      digitalWrite(WIPEO_EN_PIN, HIGH);
      
      setAndWrite(PB4_LED_EN, LOW);
    }
    break;

    case WIPERLOW:
    {
      delay(100);
      digitalWrite(WIPEO_EN_PIN, LOW);
      delay(100);
      digitalWrite(WIPEH_EN_PIN, LOW);
      delay(100);
      digitalWrite(WIPEL_EN_PIN, HIGH);
      delay(100);
      
      setAndWrite(PB4_LED_EN, blink_states[BLINKFAST]);
    }
    break;

    case WIPERHIGH:
    {
      delay(100);
      digitalWrite(WIPEO_EN_PIN, LOW);
      delay(100);
      digitalWrite(WIPEL_EN_PIN, LOW);
      delay(100);
      digitalWrite(WIPEH_EN_PIN, HIGH);
  
      setAndWrite(PB4_LED_EN, blink_states[BLINKFASTEST]);
    }
    break;
    
  }
}

/*
  Shift Register / Multiplexer Functions
*/

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


void debug_print(const String &s, bool newline) {
#if DEBUGMODE
  if (newline) {
    Serial.println(s);
  }
  else {
    Serial.print(s);
  }
#endif // DEBUGMODE
}

void debug_print(int s, bool newline) {
#if DEBUGMODE
  if (newline) {
    Serial.println(s);
  }
  else {
    Serial.print(s);
  }
#endif // DEBUGMODE
}

void debug_print(float s, bool newline) {
#if DEBUGMODE
  if (newline) {
    Serial.println(s);
  }
  else {
    Serial.print(s);
  }
#endif // DEBUGMODE
}

void debug_print(byte s, bool newline) {
#if DEBUGMODE
  if (newline) {
    Serial.println(s);
  }
  else {
    Serial.print(s);
  }
#endif // DEBUGMODE
}

void debug_print(unsigned long s, bool newline) {
#if DEBUGMODE
  if (newline) {
    Serial.println(s);
  }
  else {
    Serial.print(s);
  }
#endif // DEBUGMODE
}

