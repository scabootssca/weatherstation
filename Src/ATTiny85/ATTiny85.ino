#ifndef F_CPU
#define F_CPU 1000000UL
#endif

//#ifndef ICACHE_RAM_ATTR
//#define ICACHE_RAM_ATTR
//#endif


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <TinyWireM.h>
#include <SoftwareSerial.h>

#include "sampleTypes.h"
#include "TinyMCP23008.h"


#define TX_PIN PB3
#define RX_PIN PB5 // Reset pin but we don't use it
SoftwareSerial Serial(RX_PIN, TX_PIN);

// Pin Defines
#define INTERRUPT_PIN PCINT4

// Mcp pins
#define ONEWIRE_RX_PIN 0
#define ONEWIRE_TX_PIN 1

#define ANEMOMETER_PIN_MCP 6
#define LED_PIN_MCP 7

// Defines
#define SAMPLES_PER_READING 150
#define SAMPLE_INTERVAL 2000

// Anemometer
#define ANEMOMETER_CALIBRATION_COEF 0.09739260404185239 // 10.2677201193868 = samples per 1mph


// Variables
SampleAccumulator sampleAccumulator;
unsigned short numSamples = 0;

unsigned int anemometerPulseCount = 0;

//unsigned long lastInterruptMillis = 0;
unsigned long lastSampleMillis = 0;
unsigned long lastAnemometerReadingMillis = 0;

volatile unsigned long lastInterruptMillis = 0;
volatile bool hasInterrupt = false;

int i2cRequestCount = 0;
byte recvByte = 0;

// IO Expander
TinyMCP23008 mcp;
#include "McpComn.h"

void espWake() {
  //PORTB ^= (1 << ESP_RESET_PIN);
  mcp.digitalWrite(LED_PIN_MCP, HIGH);
}

void espSleep() {
  //PORTB ^= (1 << ESP_RESET_PIN);
  mcp.digitalWrite(LED_PIN_MCP, LOW);
}

void espGetSample() {
  //unsigned int response = TinyWireM.requestFrom(ESP_I2C_ADDR, 8);
  //someByte = TinyWireM.recieve()
  //TinyWireM.available()
}

void espSubmitReadings() {
  // We'll send it our accumulated sample with a submit command
  // The esp will write the reading to the eeprom if the submission fails
}

void averageSampleAccumulator() {

}

float readAnemometer() {
  int millisNow = millis();

  // Get how many samples we had
  cli();
  int numTimelyAnemometerSamples = anemometerPulseCount;
  anemometerPulseCount = 0;
  sei();

	// mph = revolutions * measurementDuration * calibrationData
	float anemometerMph = (numTimelyAnemometerSamples * 0.5) * (60000.0 / (millisNow-lastAnemometerReadingMillis)) * ANEMOMETER_CALIBRATION_COEF;
  lastAnemometerReadingMillis = millisNow;

	return anemometerMph;
}

ISR(PCINT0_vect) {
  // If the current state of the pin is high then it's an interrupt since we only want rising edges
  if ((PINB>>INTERRUPT_PIN)&1) {
    hasInterrupt = true;
  }
}

static inline void initInterrupt(void) {
  // We'll handle interrupts on INTERRUPT_PIN

  // PCIE: Pin Change Interrupt Enable
  GIMSK |= (1 << PCIE);

  // Enable interrupts for INTERRUPT_PIN
  PCMSK |= (1 << INTERRUPT_PIN);

  sei(); // Enable interrupts (Sets I flag (Bit 7) in SREG - AVR Status Register)
}

void pinMode(int pin, int dir) {
  if (dir) {
    DDRB |= (1 << pin); // Enable reset pin as output
  } else {
    DDRB &= ~(1 << pin); // Enable reset pin as input
  }
}

int digitalRead(int pin) {
  return (PORTB>>pin)&1;
}

int digitalWrite(int pin, int dir) {
  if (dir) {
    PORTB |= (1 << pin); // Enable pullup (write high)
  } else {
    PORTB &= ~(1 << pin);
  }
}

void setup() {
  ADCSRA &= ~(1<<ADEN); // Disable ADC

  /*
  ACSR = (1<<ACD); //Disable the analog comparator
  DIDR0 = 0x3F; //Disable digital input buffers on all ADC0-ADC5 pins.
  */

  // Init serial
  Serial.begin(2400);

  // // Init i2c
  TinyWireM.begin();

  // mcp23008 begin
  mcp.begin(1);      // use address 1

  // Interrupts for mcp23008
  mcp.setInterruptOutPinMode(MCP23008_INT_OUT_HIGH);
  initInterrupt(); // Setup interrupt registers on the attiny

  // Esp communication pins
  mcp.pinMode(ONEWIRE_TX_PIN, OUTPUT);
  mcp.pinMode(ONEWIRE_RX_PIN, INPUT);

  mcp.digitalWrite(ONEWIRE_TX_PIN, LOW);
  mcp.enableInterrupt(ONEWIRE_RX_PIN, FALLING); // This'll interrupt when the pin goes low for a request

  // Anemometer pin
  mcp.pinMode(ANEMOMETER_PIN_MCP, INPUT);
  mcp.enableInterrupt(ANEMOMETER_PIN_MCP, RISING);

  // Led status
  mcp.pinMode(LED_PIN_MCP, OUTPUT);
}

void loop() {
  if (hasInterrupt) {
    cli();
    uint8_t interruptMask = mcp.readInterrupts();
    hasInterrupt = false;
    sei();

    if ((interruptMask>>ANEMOMETER_PIN_MCP)&1) {
      if (millis()-lastInterruptMillis > 100) {
        lastInterruptMillis = millis();
        anemometerPulseCount++;
        Serial.print("Pulse #: ");
        Serial.println(anemometerPulseCount);
        //Serial.println("Handled Interrupt: ANEMOMETER_PIN_MCP");
      }
    }

    if ((interruptMask>>ONEWIRE_RX_PIN)&1) {
      sendResponse(anemometerPulseCount);
      anemometerPulseCount = 0;
      //Serial.println("Handled Interrupt: ONEWIRE_RX_PIN");
    }

    // Serial.print("Interrupts: ");
    //
    // for (int i=7;i>=0;i--) {
    //   Serial.print( (interruptMask>>i)&1 );
    // }
    //
    // Serial.println();

  // } else if ((PINB>>INTERRUPT_PIN)&1) {
  //   Serial.println("Detected Stall; Resetting");
  //   mcp.readInterrupts();
  // }
  }

  if (millis()-lastSampleMillis > SAMPLE_INTERVAL) {
    lastSampleMillis = millis();

    //PORTB ^= (1 << PB3);
    Serial.print("ATtiny Tick: (");
    Serial.print(anemometerPulseCount);
    Serial.println(")");
  }

  //   espWake(); // Drive ESP_RESET_PIN low
  //   // The ESP will wake and then take a sample and wait for this as the i2c master
  //   // It'll send the data and we'll store it in the accumulator
  //
  //   espGetSample();
  //   float windSpeed = readAnemometer();
  //   //float rainVolumnMl = readRainBucket();
  //
  //   // Debug out
  //   Serial.print(windSpeed);
  //   Serial.println(" MPH");
  //
  //   // If the samples are full then average the accumulator
  //   if (numSamples == SAMPLES_PER_READING-1) {
  //     averageSampleAccumulator();
  //     espSubmitReadings(); // This'll send the esp the reading(s) and have it submit it(them)
  //   }
  //
  //   espSleep();
  // }
}
