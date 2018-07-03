int SREG_SER_PIN =      26; // on the arduino digital pins
int SREG_RCLK_PIN =     27; // on the arduino digital pins
int SREG_SRCLK_PIN =    28; // on the arduino digital pins
int SREG_SRCLR_PIN =    29; // on the arduino digital pins
int SREG_OE_PIN =       30; // on the arduino digital pins

void setup() {
  pinMode(SREG_SER_PIN, OUTPUT);
  pinMode(SREG_RCLK_PIN, OUTPUT);
  pinMode(SREG_SRCLK_PIN, OUTPUT);
  pinMode(SREG_SRCLR_PIN, OUTPUT);
  pinMode(SREG_OE_PIN, OUTPUT);

}

void loop() {
    
  digitalWrite(SREG_SER_PIN, HIGH);
  digitalWrite(SREG_RCLK_PIN, HIGH);
  digitalWrite(SREG_SRCLK_PIN, HIGH);
  digitalWrite(SREG_SRCLR_PIN, HIGH);
  digitalWrite(SREG_OE_PIN, HIGH);

  delay(1000);

  digitalWrite(SREG_SER_PIN, LOW);
  digitalWrite(SREG_RCLK_PIN, LOW);
  digitalWrite(SREG_SRCLK_PIN, LOW);
  digitalWrite(SREG_SRCLR_PIN, LOW);
  digitalWrite(SREG_OE_PIN, LOW);

  delay(1000);
  
}
