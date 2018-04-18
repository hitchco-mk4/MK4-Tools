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
int MUX_S3_PIN = 31; // on the arduino digital pins
int MUX_S2_PIN = 32; // on the arduino digital pins
int MUX_S1_PIN = 33; // on the arduino digital pins
int MUX_S0_PIN = 34; // on the arduino digital pins
int MUX_EN_PIN = 35; // on the arduino digital pins
int MUX_SIG_PIN = 12; // on the arduino analog pins

#define num_buttons 15

/*
  Shift Register Mapping
*/
int PB1_LED_EN = 0; // on the shift registers
int PB2_LED_EN = 1; // on the shift registers
int PB3_LED_EN = 2; // on the shift registers
int PB4_LED_EN = 3; // on the shift registers
int PB6_LED_EN = 4; // on the shift registers
int PB7_LED_EN = 5; // on the shift registers
int PB9_LED_EN = 6; // on the shift registers
int PB10_LED_EN = 7; // on the shift registers
int PB11_LED_EN = 8; // on the shift registers
int PB12_LED_EN = 9; // on the shift registers
int PB13_LED_EN = 10; // on the shift registers
int PB14_LED_EN = 11; // on the shift registers
int PB15_LED_EN = 12; // on the shift registers
int PB16_LED_EN = 13; // on the shift registers
int PB17_LED_EN = 14; // on the shift registers

int LEDs[15] = {PB1_LED_EN, PB2_LED_EN, PB3_LED_EN, PB4_LED_EN, PB6_LED_EN, PB7_LED_EN, PB9_LED_EN, PB10_LED_EN, PB11_LED_EN, PB12_LED_EN, PB13_LED_EN, PB14_LED_EN, PB15_LED_EN, PB16_LED_EN, PB17_LED_EN};

/*
  Multiplexer Mapping
*/
int PB1_SIG_PIN = 0; // on the mux
int PB2_SIG_PIN = 1; // on the mux
int PB3_SIG_PIN = 2; // on the mux
int PB4_SIG_PIN = 3; // on the mux
int PB6_SIG_PIN = 4; // on the mux
int PB7_SIG_PIN = 5; // on the mux
int PB9_SIG_PIN = 6; // on the mux
int PB10_SIG_PIN = 7; // on the mux
int PB11_SIG_PIN = 8; // on the mux
int PB12_SIG_PIN = 9; // on the mux
int PB13_SIG_PIN = 10; // on the mux
int PB14_SIG_PIN = 11; // on the mux
int PB15_SIG_PIN = 12; // on the mux
int PB16_SIG_PIN = 13; // on the mux
int PB17_SIG_PIN = 14; // on the mux

int buttons[15] = { PB1_SIG_PIN, PB2_SIG_PIN, PB3_SIG_PIN, PB4_SIG_PIN, PB6_SIG_PIN, PB7_SIG_PIN, PB9_SIG_PIN, PB10_SIG_PIN, PB11_SIG_PIN, PB12_SIG_PIN, PB13_SIG_PIN, PB14_SIG_PIN, PB15_SIG_PIN, PB16_SIG_PIN, PB17_SIG_PIN };

// Starter button pins
int PB8_SIG_PIN = 43; // on the arduino digital pins
int PB8_BLED_PIN = 44; // on the arduino digital pins
int PB8_GLED_PIN = 45; // on the arduino digital pins
int PB8_RLED_PIN = 46; // on the arduino digital pins


void setup() {
  // put your setup code here, to run once:

  pinMode(PB8_SIG_PIN, INPUT);
  pinMode(PB8_BLED_PIN, OUTPUT);
  pinMode(PB8_GLED_PIN, OUTPUT);
  pinMode(PB8_RLED_PIN, OUTPUT);

  pinMode(SREG_SER_PIN, OUTPUT);
  pinMode(SREG_RCLK_PIN, OUTPUT);
  pinMode(SREG_SRCLK_PIN, OUTPUT);
  pinMode(SREG_SRCLR_PIN, OUTPUT);
  pinMode(SREG_OE_PIN, OUTPUT);

  pinMode(MUX_S3_PIN, OUTPUT);
  pinMode(MUX_S2_PIN, OUTPUT);
  pinMode(MUX_S1_PIN, OUTPUT);
  pinMode(MUX_S0_PIN, OUTPUT);
  
  digitalWrite(SREG_OE_PIN, LOW);
  digitalWrite(SREG_SRCLR_PIN, HIGH);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:



  if (digitalRead(PB8_SIG_PIN)) {
    digitalWrite(PB8_BLED_PIN, LOW);
    delay(100);
    digitalWrite(PB8_GLED_PIN, LOW);
    delay(100);
    digitalWrite(PB8_RLED_PIN, LOW);
    delay(100);
    analogWrite(PB8_RLED_PIN, 128);
    analogWrite(PB8_GLED_PIN, 128);
    analogWrite(PB8_RLED_PIN, 255);
    
  } else {
    digitalWrite(PB8_BLED_PIN, HIGH);
    digitalWrite(PB8_GLED_PIN, HIGH);
    digitalWrite(PB8_RLED_PIN, HIGH);
  }
  
  for (int i = 0; i < num_buttons; i++) {
    if (readMux(i) == HIGH) {
      setAndWrite(i, HIGH);
      Serial.println(i);
    } else {
      setAndWrite(i, LOW);
    }
  }
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

int readMux(int channel) {
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

  if (val > 512) {
    return 1;
  } else {
    return 0;
  }
  
}

