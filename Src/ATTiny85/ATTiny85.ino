#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <SoftwareSerial.h>

// ATTiny Pins
#define ANEMOMETER_PIN PCINT4
#define RAIN_BUCKET_PIN PCINT3
#define ESP_READING_REQUEST_PIN PB2
#define TX_PIN PB1
#define ESP_RESET_PIN PB0

// Serial comns
SoftwareSerial Serial(-1, TX_PIN);

// Sample Settings
#define SAMPLES_PER_READING 150
#define SAMPLE_INTERVAL 10000

// Sample variables
volatile uint8_t previousInterruptPins = 0;
volatile unsigned long lastInterruptAnemometer = 0;
volatile unsigned long lastInterruptRainBucket = 0;

volatile unsigned char anemometerSampleCount = 0;
volatile unsigned char rainBucketSampleCount = 0;

volatile bool requestedSamples = false;
volatile unsigned long ledOnMillis = 0;

// Other variables
unsigned long lastTickMillis = 0;


// 1 / 10.2677201193868 = samples per 1mph
#define ANEMOMETER_CALIBRATION_COEF 0.09739260404185239

void printBin(int pbyte) {
  for (int i=0;i<8; i++) {
    Serial.print((pbyte>>i)&1);
  }
  Serial.println();
}

ISR(PCINT0_vect) {
  // Get what chaged
  uint8_t changedBits;
  changedBits = PINB ^ previousInterruptPins;

  // If the pin is changed and it was previously low
  if (((changedBits>>ANEMOMETER_PIN)&1) && !((previousInterruptPins>>ANEMOMETER_PIN)&1)) {
    if (millis()-lastInterruptAnemometer > 2) {
      lastInterruptAnemometer = millis();
      anemometerSampleCount++;
      PORTB |= 1<<ESP_RESET_PIN;
      ledOnMillis = millis();
    }
  }

  if (((changedBits>>RAIN_BUCKET_PIN)&1) && !((previousInterruptPins>>RAIN_BUCKET_PIN)&1)) {
    if (millis()-lastInterruptRainBucket > 2) {
      lastInterruptRainBucket = millis();
      rainBucketSampleCount++;
      PORTB |= 1<<ESP_RESET_PIN;
      ledOnMillis = millis();
    }
  }

  if (!requestedSamples && ((changedBits>>ESP_READING_REQUEST_PIN)&1) && !((previousInterruptPins>>ESP_READING_REQUEST_PIN)&1)) {
    requestedSamples = true;
    PORTB |= 1<<ESP_RESET_PIN;
    ledOnMillis = millis();
  }

  previousInterruptPins = PINB;
}

static inline void initInterrupt(void) {
  // PCIE: Pin Change Interrupt Enable
  GIMSK |= (1 << PCIE);

  // Set pins to input
  DDRB |= (1 << ANEMOMETER_PIN);
  DDRB |= (1 << RAIN_BUCKET_PIN);
  DDRB |= (1 << ESP_READING_REQUEST_PIN);

  // Enable interrupts for pins
  PCMSK |= (1 << ANEMOMETER_PIN);
  PCMSK |= (1 << RAIN_BUCKET_PIN);
  PCMSK |= (1 << ESP_READING_REQUEST_PIN);

  // Save the start state
  previousInterruptPins = PINB;

  sei(); // Enable interrupts (Sets I flag (Bit 7) in SREG - AVR Status Register)
}

void setup() {
  Serial.begin(2400);

  Serial.println();
  Serial.println("Initilizing ATtiny85");

  // Setup interrupts
  initInterrupt();

  // Set the reset pin as out and low
  DDRB |= 1<<ESP_RESET_PIN;
  PORTB &= ~(1<<ESP_RESET_PIN);

  // Request pin as input
  DDRB &= ~(1<<ESP_READING_REQUEST_PIN);
}

void loop() {
  // if (millis()-lastTickMillis > 1000) {
  //   lastTickMillis = millis();
  //   Serial.print("Tick; Wind: ");
  //   Serial.print(anemometerSampleCount);
  //   Serial.print(" Rain: ");
  //   Serial.println(rainBucketSampleCount);
  //   Serial.print("OnTime: ");
  //   Serial.println(ledOnMillis);
  // }

  // Turn off the led after 100ms
  if (ledOnMillis && (millis() > ledOnMillis+100)) {
    ledOnMillis = 0;
    PORTB &= ~(1<<ESP_RESET_PIN);
  }

  // If the request pin is high
  if (requestedSamples) {
    cli();
    requestedSamples = false;
    unsigned char samples[2] = {anemometerSampleCount, rainBucketSampleCount};
    rainBucketSampleCount = 0;
    anemometerSampleCount = 0;
    sei();

    Serial.write("~");
    Serial.write(samples[0]);
    Serial.write("!");
    Serial.write(samples[1]);
    Serial.write("\n");
  }
}
