// #ifndef F_CPU
// #define F_CPU 1000000UL
// #endif

//#ifndef ICACHE_RAM_ATTR
//#define ICACHE_RAM_ATTR
//#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// #include <TinyWireM.h>
// #include <TinyWireS.h>
#include <SoftwareSerial.h>

//#include "TinyMCP23008.h"
//#include "RTClib.h"

// ATTiny Pins
//SDA_PIN is PB0
//#define ESP_AWAKE_PIN PB1
//SCL_PIN is PB2
#define ANEMOMETER_PIN PCINT4
#define RAIN_BUCKET_PIN PCINT3
#define RX_PIN PB2
#define TX_PIN PB1
#define ESP_RESET_PIN PB0

// // MCP23008 and pins
// TinyMCP23008 mcp;
// #define ESP_RESET_PIN 0
// #define PULSE_PIN 5
// #define RAIN_PIN 6
// #define ANEMOMETER_PIN 7

// Serial comns
SoftwareSerial Serial(RX_PIN, TX_PIN);

// // I2C
// // We'll start as a slave with the esp awake
// #define I2C_SLAVE_ADDR 0x57
// #define I2C_MASTER 0
// #define I2C_SLAVE 1
//
// bool i2c_mode = I2C_SLAVE;

// Globals
// volatile bool espAwake = true;
// volatile bool hasInterrupt;

unsigned long lastTickMillis = 0;

// Sample Settings
#define SAMPLES_PER_READING 150
#define SAMPLE_INTERVAL 10000

volatile unsigned int anemometerSampleCount = 0;
volatile unsigned int rainBucketSampleCount = 0;

volatile unsigned long lastInterrupt = 0;

volatile uint8_t previousInterruptPins = 0;

volatile unsigned long ledOff = 0;

// 1 / 10.2677201193868 = samples per 1mph
#define ANEMOMETER_CALIBRATION_COEF 0.09739260404185239

void printBin(int pbyte) {
  for (int i=0;i<8; i++) {
    Serial.print((pbyte>>i)&1);
  }
  Serial.println();
}

ISR(PCINT0_vect) {
  if (millis()-lastInterrupt < 50) {
    return;
  }

  lastInterrupt = millis();

  // Get what chaged
  uint8_t changedBits;
  changedBits = PINB ^ previousInterruptPins;

  // If the pin is changed and it was previously low
  if ((changedBits>>ANEMOMETER_PIN)&1 && !((previousInterruptPins>>ANEMOMETER_PIN)&1)) {
    anemometerSampleCount++;
    ledOff = millis()+50;
    PORTB |= 1<<ESP_RESET_PIN;
  }

  if ((changedBits>>RAIN_BUCKET_PIN)&1 && !((previousInterruptPins>>RAIN_BUCKET_PIN)&1)) {
    rainBucketSampleCount++;
    ledOff = millis()+50;
    PORTB |= 1<<ESP_RESET_PIN;
  }

  previousInterruptPins = PINB;
}

static inline void initInterrupt(void) {
  // PCIE: Pin Change Interrupt Enable
  GIMSK |= (1 << PCIE);

  // Set pins to input
  DDRB |= (1 << ANEMOMETER_PIN);
  DDRB |= (1 << RAIN_BUCKET_PIN);

  // Enable interrupts for pins
  PCMSK |= (1 << ANEMOMETER_PIN);
  PCMSK |= (1 << RAIN_BUCKET_PIN);

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
}

void loop() {
  if (millis()-lastTickMillis > 1000) {
    lastTickMillis = millis();
    Serial.print("Wind: ");
    Serial.print(anemometerSampleCount);
    Serial.print("Rain: ");
    Serial.println(rainBucketSampleCount);
  }

  if (millis()>ledOff) {
    ledOff += 1000;
    PORTB &= ~(1<<ESP_RESET_PIN);
  }

}

// // Variables
// SampleAccumulator sampleAccumulator;
// unsigned short numSamples = 0;
//
// // Pulse counts
// uint8_t anemometerPulseCount = 0;
// uint8_t rainPulseCount = 0;
//
// // Interrupt and reading timings
// unsigned long lastInterruptMillis = 0;
// unsigned long lastInterruptMillis1 = 0;
// unsigned long lastTickMillis = 0;
// unsigned long lastSampleMillis = 0;
// unsigned long lastAnemometerReadingMillis = 0;
//
// volatile bool hasInterrupt = false;
// bool espResetting = false;
//

// void espReset() {
//   return;
//   //PORTB ^= (1 << ESP_RESET_PIN);
//   mcp.pinMode(ESP_RESET_PIN, OUTPUT);
//
//   espResetting = true;
//   mcp.digitalWrite(ESP_RESET_PIN, LOW);
//   delay(10);
//   mcp.digitalWrite(ESP_RESET_PIN, HIGH);
//   delay(10);
//   mcp.digitalWrite(ESP_RESET_PIN, LOW);
//   espResetting = false;
//
//   mcp.pinMode(ESP_RESET_PIN, INPUT);
// }
//
// void espGetSample() {
//   //unsigned int response = TinyWireM.requestFrom(ESP_I2C_ADDR, 8);
//   //someByte = TinyWireM.recieve()
//   //TinyWireM.available()
// }
//
// void espSubmitReadings() {
//   // We'll send it our accumulated sample with a submit command
//   // The esp will write the reading to the eeprom if the submission fails
// }
//
// void averageSampleAccumulator() {
//
// }
//
// float readAnemometer() {
//   int millisNow = millis();
//
//   // Get how many samples we had
//   cli();
//   int numTimelyAnemometerSamples = anemometerPulseCount;
//   anemometerPulseCount = 0;
//   sei();
//
// 	// mph = revolutions * measurementDuration * calibrationData
// 	float anemometerMph = (numTimelyAnemometerSamples * 0.5) * (60000.0 / (millisNow-lastAnemometerReadingMillis)) * ANEMOMETER_CALIBRATION_COEF;
//   lastAnemometerReadingMillis = millisNow;
//
// 	return anemometerMph;
// }
//
// ISR(PCINT0_vect) {
//   // If the current state of the pin is high then it's an interrupt since we only want rising edges
//   //if ((PINB>>INTERRUPT_PIN)&1) {
//   //hasInterrupt = true;
//   //}
//
//   if ((PINB>>INTERRUPT_PIN)&1) {
//     hasInterrupt = true;
//   }
//
//   espAwake = ((PINB>>ESP_AWAKE_PIN)&1);
// }

// void setup() {
//   //ADCSRA &= ~(1<<ADEN); // Disable ADC
//
//   /*
//   ACSR = (1<<ACD); //Disable the analog comparator
//   DIDR0 = 0x3F; //Disable digital input buffers on all ADC0-ADC5 pins.
//   */
//
//   // Init serial
//   Serial.begin(1200);
//
//   Serial.println();
//   Serial.println("Initilizing ATtiny85");
//
//   // Store if the esp is awake
//   espAwake = ((PINB>>ESP_AWAKE_PIN)&1);
//
//   // Setup interrupt registers on the attiny
//   //initInterrupt();
//
//   // Set to master if ESP_AWAKE is low else slave
//   //set_i2c_mode(((PINB>>ESP_AWAKE_PIN)&1));
//   //DDRB |= 0<<ESP_AWAKE_PIN; // Set it as input
//   //ESP_AWAKE_PIN
// }
//
// void loop() {
//   Serial.println("Tick");
//   delay(1000);
//   return;
//   // // If it's awake and we're a master then set to slave
//   // if (espAwake && i2c_mode == I2C_MASTER) {
//   //   set_i2c_mode(I2C_SLAVE);
//   // // If it's sleeping and we're a slave then set to master
//   // } else if (!espAwake && i2c_mode == I2C_SLAVE) {
//   //   set_i2c_mode(I2C_MASTER);
//   // }
//   //
//   // if (espAwake) {
//   //   return;
//   // }
//
//   // if (millis()-lastSampleMillis > 1) {
//   //   lastSampleMillis = millis();
//   //   mcp.digitalWrite(ESP_AWAKE_DEBUG_PIN, !mcp.digitalRead(ESP_AWAKE_PIN));
//   // }
//
//   // delay(1);
//   // if (mcp.digitalRead(ESP_AWAKE_PIN) == HIGH) {
//   //   mcp.digitalWrite(ESP_AWAKE_DEBUG_PIN, !mcp.digitalRead(ESP_AWAKE_PIN));
//   //   return;
//   // }
//
//   // if (!hasInterrupt && (PINB>>INTERRUPT_PIN)&1) {
//   //   // Seems it got stuck high
//   //   //Serial.println("Interrupt stuck, clearing");
//   //   hasInterrupt = true;
//   // }
//
//   if (millis()-lastTickMillis > 1000) {
//     mcp.digitalWrite(PULSE_PIN, HIGH);
//     lastTickMillis = millis();
//
//     //PORTB ^= (1 << PB3);
//     Serial.print("ATtiny Tick: ");
//     //Serial.println(RTC.now().unixtime());
//     delay(5);
//     mcp.digitalWrite(PULSE_PIN, LOW);
//     // Serial.print(anemometerPulseCount);
//     // Serial.println(")");
//
//     //espReset();
//   }
//
//   // If it's high or has interrupt
//   //if (hasInterrupt) {
//   if ((PINB>>INTERRUPT_PIN)&1) {
//     cli();
//     hasInterrupt = false;
//     sei();
//
//     uint8_t interruptMask = mcp.readInterrupts();
//
//     if ((interruptMask>>ANEMOMETER_PIN)&1) {
//       if (millis()-lastInterruptMillis > 100) {
//         lastInterruptMillis = millis();
//         anemometerPulseCount++;
//         Serial.print("Wind Pulse #: ");
//         Serial.println(anemometerPulseCount);
//         //Serial.println("Handled Interrupt: ANEMOMETER_PIN");
//       }
//     }
//
//     if ((interruptMask>>RAIN_PIN)&1) {
//       if (millis()-lastInterruptMillis1 > 100) {
//         lastInterruptMillis1 = millis();
//         rainPulseCount++;
//         Serial.print("Rain Pulse #: ");
//         Serial.println(rainPulseCount);
//         //Serial.println("Handled Interrupt: ANEMOMETER_PIN");
//       }
//     }
//
//     // if (!espResetting) {
//     //   // if ((interruptMask>>ONEWIRE_RX_PIN)&1) {
//     //   //   // response = [rainPulseCount][anemometerPulseCount]
//     //   //   uint16_t response = rainPulseCount;
//     //   //   response <<= 8;
//     //   //   response |= anemometerPulseCount;
//     //   //
//     //   //   sendResponse(response);
//     //   //
//     //   //   Serial.print("Sent Rain: ");
//     //   //   Serial.println(rainPulseCount);
//     //   //   Serial.print("Sent Wind: ");
//     //   //   Serial.println(anemometerPulseCount);
//     //   //
//     //   //   anemometerPulseCount = 0;
//     //   //   rainPulseCount = 0;
//     //   //   //Serial.println("Handled Interrupt: ONEWIRE_RX_PIN");
//     //   // }
//     // }
//
//     // Serial.print("Interrupts: ");
//     //
//     // for (int i=7;i>=0;i--) {
//     //   Serial.print( (interruptMask>>i)&1 );
//     // }
//     //
//     // Serial.println();
//
//   // } else if ((PINB>>INTERRUPT_PIN)&1) {
//   //   Serial.println("Detected Stall; Resetting");
//   //   mcp.readInterrupts();
//   // }
//   }
//
//
//   // // Try Serial commands
//   // bool toWrite = false;
//   // while (Serial.available()) {
//   //   char rxByte = Serial.read();
//   //
//   //   if (toWrite) {
//   //     toWrite = false;
//   //     Serial.write("PRINT>");
//   //     Serial.write(rxByte);
//   //   }
//   //
//   //   else if (strcmp(&rxByte, "p") == 0) {
//   //     toWrite = true;
//   //   }
//   // }
//
//
//
//   // if (millis()-lastSampleMillis > 20000) {
//   //   lastSampleMillis = millis();
//   //   Serial.println("Resetting ESP");
//   //   espReset();
//   // }
//
//   //   // The ESP will wake and then take a sample and wait for this as the i2c master
//   //   // It'll send the data and we'll store it in the accumulator
//   //
//   //   espGetSample();
//   //   float windSpeed = readAnemometer();
//   //   //float rainVolumnMl = readRainBucket();
//   //
//   //   // Debug out
//   //   Serial.print(windSpeed);
//   //   Serial.println(" MPH");
//   //
//   //   // If the samples are full then average the accumulator
//   //   if (numSamples == SAMPLES_PER_READING-1) {
//   //     averageSampleAccumulator();
//   //     espSubmitReadings(); // This'll send the esp the reading(s) and have it submit it(them)
//   //   }
//   //
//   //   espSleep();
// }
//
// // void pinMode(int pin, int dir) {
// //   if (dir) {
// //     DDRB |= (1 << pin); // Enable reset pin as output
// //   } else {
// //     DDRB &= ~(1 << pin); // Enable reset pin as input
// //   }
// // }
//
// // int digitalRead(int pin) {
// //   return (PORTB>>pin)&1;
// // }
// //
// // int digitalWrite(int pin, int dir) {
// //   if (dir) {
// //     PORTB |= (1 << pin); // Enable pullup (write high)
// //   } else {
// //     PORTB &= ~(1 << pin);
// //   }
// // }
