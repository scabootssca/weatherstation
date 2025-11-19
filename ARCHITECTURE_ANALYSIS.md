# ATmega Weather Logger - Architecture Analysis

## Executive Summary

This document provides a comprehensive technical analysis of the ATmega Weather Logger codebase for its intended use as a budget-friendly PLC-esque automation system for greenhouses, climate automation, and gardens.

---

## 1. System Architecture Overview

### Dual-Processor Design
```
[Sensors] --> [ATmega328p] --> [SoftwareSerial @ 4800 baud] --> [ESP8266] --> [WiFi] --> [Server]
```

| Component | Role | Communication |
|-----------|------|---------------|
| ATmega328p | Main controller - sensor reading, data processing, storage | I2C, SPI, SoftwareSerial |
| ESP8266 | WiFi module - HTTP requests to server | SoftwareSerial, WiFi |

### Hardware Components

**Sensors:**
- BME280 (I2C @ 0x76) - Temperature, Humidity, Barometric Pressure
- BH1750 (I2C @ 0x23) - Lux/Light level
- DS1307 (I2C @ 0x68) - Real-time clock
- Wind vane - Analog input (ADC)
- Anemometer - Digital interrupt (pulse counting)
- Rain gauge - Digital interrupt (tipping bucket)
- Battery voltage - Analog input with voltage divider

**Peripheral ICs:**
- MCP23008 (I2C @ 0x20) - GPIO expander for ESP control
- 23A1024 (SPI) - External 1Mbit SRAM for data storage

---

## 2. Major Sections and Key Functions Analysis

### 2.1 Main Controller (`ATmega328p.ino`)

#### Initialization (`setup()`)
**Location:** Lines 328-536

**Purpose:** System initialization including watchdog, peripherals, and state restoration.

**Key Operations:**
- Enables 8-second watchdog timer
- Initializes I2C bus with timeout and recovery
- Configures SPI for external SRAM
- Sets up GPIO expander (MCP23008) for ESP control
- Configures interrupts for anemometer and rain gauge
- Restores state from external SRAM or initializes fresh
- Sends boot debug message to server

#### Main Loop (`loop()`)
**Location:** Lines 743-790

**Purpose:** Core processing loop with periodic sampling and reading generation.

**Key Operations:**
1. Pet watchdog timer
2. Handle ESP state machine
3. Take samples every 2 seconds
4. Create weather readings every 5 minutes (150 samples)
5. Trigger submission every 20 minutes (4 readings)

#### ESP State Machine (`handle_esp()`)
**Location:** Lines 539-630

**Purpose:** Manages ESP8266 lifecycle for reliable communication.

**States:**
| State | Description |
|-------|-------------|
| `ESP_STATE_SLEEP` | ESP in deep sleep, no power consumption |
| `ESP_STATE_RESETTING` | ESP being reset, waiting for ready signals |
| `ESP_STATE_IDLE` | ESP awake and ready for commands |
| `ESP_STATE_AWAITING_RESULT` | Request sent, waiting for response |
| `ESP_STATE_AWAITING_DEBUG_RESULT` | Debug request sent |

#### Anemometer ISR (`anemometerISR()`)
**Location:** Lines 219-258

**Purpose:** Interrupt-driven wind speed measurement with debouncing.

**Key Features:**
- 50ms debounce (max ~58.4 MPH measurable)
- Accumulates time deltas for averaging
- Tracks minimum delta for gust detection
- Handles millis() rollover

#### Data Sampling (`take_sample()`)
**Location:** Lines 1182-1258

**Purpose:** Collects all sensor data and accumulates for averaging.

**Sampled Data:**
- Battery voltage (every 25th sample)
- Wind speed and direction
- Rain bucket pulses
- Temperature, pressure, humidity (BME280)
- Light level (BH1750)

### 2.2 Data Storage Module (`MCP_23A1024.h`)

#### Memory Layout
```
[0-4]     BLANK
[5-9]     populated key (0xFAD00BA6)
[10-12]   readIndex
[13-19]   BLANK
[20-22]   writeIndex
[23-31]   BLANK
[32-end]  readingAccumulator + readings circular buffer
```

**Maximum Readings:** Calculated as `(131072 - SRAM_ADDR_READINGS) / sizeof(WeatherReading)`

#### Key Functions

| Function | Purpose |
|----------|---------|
| `sram_init()` | Initialize SPI settings, set sequential mode |
| `sram_get_populated()` | Check/set magic key for data persistence |
| `sram_read_accumulator()` / `sram_write_accumulator()` | Persist accumulator state |
| `sram_read_reading()` / `sram_write_reading()` | Read/write from circular buffer |

### 2.3 Weather Data Module (`weather_readings.h`)

#### Data Structures

**WeatherReading** - Final averaged reading (40 bytes):
```c
struct WeatherReading {
    uint32_t timestamp;
    float temperature, humidity, pressure;
    float batteryMv, windGust, windSpeed, windDirection, lux;
    uint32_t rain;
};
```

**WeatherReadingAccumulator** - Sample accumulation (58 bytes):
```c
struct WeatherReadingAccumulator {
    uint32_t timestamp;
    int32_t temperature;
    uint32_t humidity, batteryMv, windGust, windSpeed, rain;
    uint64_t pressure, lux;
    float windDirectionX, windDirectionY;  // For vector averaging
    uint8_t numSamples, numBatterySamples;
};
```

#### Vector Wind Direction Averaging (`store_accumulator()`)
**Location:** Lines 109-166

Uses Cartesian coordinate averaging to properly handle circular mean:
```c
float avgWindX = src.windDirectionX / float(src.numSamples);
float avgWindY = src.windDirectionY / float(src.numSamples);
float angleRadians = atan(avgWindY / avgWindX);
// Quadrant correction for full 360 degrees
```

### 2.4 WiFi Module (`ESP8266.ino`)

#### Serial Protocol
**Format:** `[Command Byte][Length Byte][Data...]`

**Commands:**
| Code | Command | Action |
|------|---------|--------|
| 0 | PING | Echo test |
| 1 | REQUEST | Send HTTP GET |
| 2 | SLEEP | Enter deep sleep |
| 6 | IDLE | Reset result pins |

#### HTTP Communication (`send_request()`)
**Location:** Lines 294-358

- Connects to configured WiFi
- Sends HTTP GET request with weather data as URL parameters
- Parses response for "Success" keyword
- Sets GPIO pins to signal result to ATmega

---

## 3. Potential Bugs and Issues

### 3.1 Critical Bugs

#### Bug 1: Integer Overflow in Wind Speed Accumulation
**Location:** `ATmega328p.ino:1049`
```c
sampleAccumulator.windSpeed += int32_t(anemometerMph*100);
```
**Issue:** With 150 samples at high wind speeds, this can overflow. At 50 MPH average:
`150 * 50 * 100 = 750,000` (safe for int32_t, but marginal)

**Risk:** Medium - Only affects sustained high winds

#### Bug 2: Division by Zero in Wind Direction
**Location:** `weather_readings.h:138`
```c
float angleRadians = atan(avgWindY/avgWindX);
```
**Issue:** If `avgWindX == 0`, this causes undefined behavior.

**Fix Required:** Add check for `avgWindX == 0` before division.

#### Bug 3: Race Condition in Anemometer Variables
**Location:** `ATmega328p.ino:1036-1038`
```c
cli();
float averagePulseTDelta = anemometerTDeltaAccumulator / float(anemometerPulses);
anemometerPulses = 0;
anemometerTDeltaAccumulator = 0;
sei();
```
**Issue:** `anemometerTDeltaAccumulator` is `uint64_t` which requires multiple instructions to read on 8-bit AVR. While `cli()` is called, the ISR could have updated it atomically during the division if the division is optimized.

**Risk:** Low - The `cli()` prevents the issue, but code is fragile.

#### Bug 4: Buffer Overflow in ESP Serial
**Location:** `ESP8266.ino:243-246`
```c
if (ATmegaSerialIndex >= ATMEGA_BUFFER_SIZE-1) {
    Serial.println("Serial buffer overflow!");
    ATmegaSerialIndex = 0;
}
```
**Issue:** This check happens AFTER writing to the buffer, potentially allowing one byte overflow.

#### Bug 5: Uninitialized lastAnemometerPulseMillis After Reset
**Location:** `ATmega328p.ino:221-225`
```c
if (lastAnemometerPulseMillis == 0) {
    lastAnemometerPulseMillis = millis();
    return;
}
```
**Issue:** If the first pulse happens at millis() = 0 (just after reset), it will be skipped and the next tDelta will be very large.

### 3.2 Security Vulnerabilities

#### Vulnerability 1: No HTTPS - Data in Plaintext
**Location:** `ESP8266.ino:320-322`
```c
client.print(String("GET ") + url + " HTTP/1.1\r\n" + ...);
```
**Risk:** HIGH - All sensor data transmitted unencrypted
**Impact:** Data interception, potential for injection attacks

#### Vulnerability 2: Hardcoded Credentials in Config
**Location:** `config.h` (user-created)
```c
#define WIFI_PASS "*******************"
```
**Risk:** Medium - Credentials stored in plaintext
**Mitigation:** File is gitignored, but should use secure storage

#### Vulnerability 3: No Server Authentication
**Issue:** Server response is only checked for "Success" string - no verification of server identity
**Risk:** Medium - Man-in-the-middle attacks possible

#### Vulnerability 4: URL Injection Potential
**Location:** `ATmega328p.ino:814-865`
**Issue:** Sensor values are concatenated directly into URL without sanitization
**Risk:** Low - Controlled input from sensors, but no validation

### 3.3 Performance Bottlenecks

#### Bottleneck 1: String Concatenation in URL Generation
**Location:** `ATmega328p.ino:814-865`
```c
String outputUrl = "/report.php?";
outputUrl += "temp=";
outputUrl += weatherReading.temperature;
// ... repeated 15+ times
```
**Issue:** Each `+=` causes memory reallocation, fragmenting heap
**Impact:** ~300+ bytes allocated/deallocated per submission

#### Bottleneck 2: Blocking Delays in Battery Reading
**Location:** `ATmega328p.ino:1119-1141`
```c
mcp.digitalWrite(MCP_SOLAR_ENABLE_PIN, LOW);
delay(50);
mcp.digitalWrite(MCP_BAT_DIV_ENABLE_PIN, HIGH);
delay(50);
```
**Issue:** 100ms+ blocking delay during every 25th sample
**Impact:** Missed anemometer pulses during battery reading

#### Bottleneck 3: Slow SoftwareSerial Baud Rate
**Location:** `config.h`
```c
#define ESP_ATMEGA_BAUD_RATE 4800
```
**Issue:** At 4800 baud, a 200-byte URL takes ~400ms to transmit
**Impact:** Extended communication time, increased power consumption

#### Bottleneck 4: pow() Function for ADC Oversampling
**Location:** `ATmega328p.ino:731`
```c
short numSamples = (int)pow(4.0, oversampleBits);
```
**Issue:** Floating-point pow() is expensive; result is always 16 (4^2)
**Fix:** Use `(1 << (2 * oversampleBits))` or pre-computed constant

---

## 4. Code Quality Evaluation

### 4.1 Readability

**Score: 6/10**

**Strengths:**
- Consistent naming convention (snake_case for functions, camelCase for variables)
- Good use of `#define` for configuration constants
- Helpful comments explaining WMO standards compliance
- Clear separation of concerns between files

**Weaknesses:**
- Many commented-out code blocks (e.g., lines 667-707, 709-720)
- Magic numbers in some places (e.g., 50ms debounce, line 240)
- Long functions (setup() is 200+ lines)
- Inconsistent debug print formatting

### 4.2 Maintainability

**Score: 5/10**

**Strengths:**
- Modular design with separate header files
- Configuration centralized in config.h
- Debug levels (0-3) for different verbosity

**Weaknesses:**
- No unit tests
- Functions in header files (`weather_readings.h`, `MCP_23A1024.h`) - should be .cpp
- Global state makes testing difficult
- No API documentation
- Hardcoded I2C addresses scattered throughout

### 4.3 Best Practices Adherence

**Score: 6/10**

**Adherence:**
- Watchdog timer implementation
- Power management (forced mode, deep sleep)
- State persistence across resets
- Vector averaging for circular data

**Violations:**
- Functions defined in headers (not inline)
- No const correctness on function parameters
- Raw pointers without bounds checking
- No PROGMEM usage for constant strings
- Large String objects on stack

---

## 5. Improvement Recommendations

### 5.1 Critical Fixes

1. **Fix division by zero in wind direction calculation**
   ```c
   if (avgWindX == 0.0f) {
       dest->windDirection = (avgWindY >= 0) ? 90.0f : 270.0f;
   } else {
       float angleRadians = atan2(avgWindY, avgWindX); // Use atan2
       // ... rest of calculation
   }
   ```

2. **Use atan2() for wind direction** - Eliminates quadrant checking
   ```c
   float angleDegrees = atan2(avgWindY, avgWindX) * 180.0 / M_PI;
   if (angleDegrees < 0) angleDegrees += 360.0;
   ```

3. **Add atomic read for 64-bit variables**
   ```c
   cli();
   uint64_t localAccum = anemometerTDeltaAccumulator;
   unsigned int localPulses = anemometerPulses;
   anemometerPulses = 0;
   anemometerTDeltaAccumulator = 0;
   sei();
   if (localPulses > 0) {
       float averagePulseTDelta = localAccum / float(localPulses);
   }
   ```

### 5.2 Performance Optimizations

1. **Pre-allocate URL buffer**
   ```c
   char urlBuffer[300];
   snprintf(urlBuffer, sizeof(urlBuffer),
       "/report.php?temp=%.2f&hum=%.2f&pres=%.2f...",
       weatherReading.temperature, ...);
   ```

2. **Increase baud rate to 9600 or 19200**
   - SoftwareSerial can handle higher rates
   - Reduces transmission time by 50-75%

3. **Replace pow() with bit shift**
   ```c
   short numSamples = 1 << (2 * oversampleBits); // 4^n = 2^(2n)
   ```

4. **Use PROGMEM for constant strings**
   ```c
   const char URL_TEMP[] PROGMEM = "&temp=";
   ```

### 5.3 Security Improvements

1. **Implement HTTPS** using ESP8266's WiFiClientSecure
2. **Add server certificate validation**
3. **Consider MQTT** instead of HTTP for better efficiency and security
4. **Add CRC checksums** to SRAM data

### 5.4 Code Quality Improvements

1. **Refactor long functions** - Break setup() into init_sensors(), init_communication(), init_storage()
2. **Remove dead code** - Clean up commented sections
3. **Add doxygen comments** for API documentation
4. **Create proper .cpp files** for header implementations
5. **Add integration tests** using mock sensors

---

## 6. Feature Roadmap: Programmable Automation Rules

### [New Feature Name]
**Conditional Automation Engine (CAE)**

### [Summary of Feature]
A rule-based automation system that allows users to define conditional triggers and actions based on sensor readings. This transforms the weather logger into a true PLC-like controller capable of automated responses such as activating relays, controlling irrigation, or triggering alerts.

### [Justification of Feature]

**Why This Feature Is Essential for PLC/Automation:**

1. **Core PLC Functionality** - Professional PLCs (Allen-Bradley, Siemens) are defined by conditional logic: IF condition THEN action. The current system only logs data without acting on it.

2. **Greenhouse Requirements** - Automated climate control requires:
   - Temperature threshold ventilation
   - Humidity-based misting
   - Light-level shading
   - Frost protection

3. **Cost Reduction** - Eliminates need for separate cloud processing or additional controllers

4. **Offline Operation** - Rules execute locally even without WiFi connectivity

5. **Real-Time Response** - Sub-second reaction to sensor changes vs. cloud round-trip latency

**Competitive Analysis:**

| System | Local Automation | Price |
|--------|------------------|-------|
| This Project (current) | No | ~$30 |
| This Project (with CAE) | Yes | ~$35 |
| Raspberry Pi + Home Assistant | Yes | ~$60 |
| Commercial Greenhouse Controller | Yes | $200-2000 |

### [Overview of Functionality]

#### Rule Structure
```
RULE: [name]
  IF: [sensor] [operator] [threshold]
  AND/OR: [sensor] [operator] [threshold] (optional)
  THEN: [action] [parameters]
  ELSE: [action] [parameters] (optional)
  HYSTERESIS: [value] (optional)
  COOLDOWN: [seconds] (optional)
```

#### Example Rules
```
RULE: "Frost Protection"
  IF: temperature < 2.0
  THEN: relay_on(1)
  ELSE: relay_off(1)
  HYSTERESIS: 1.0

RULE: "Ventilation"
  IF: temperature > 28.0
  OR: humidity > 85.0
  THEN: relay_on(2)
  COOLDOWN: 300

RULE: "Night Lighting"
  IF: lux < 100
  AND: hour >= 6
  AND: hour <= 20
  THEN: relay_on(3)
```

#### Supported Conditions
- All sensor readings: temperature, humidity, pressure, windSpeed, windGust, windDirection, rain, lux, batteryMv
- Time-based: hour, minute, dayOfWeek
- Derived values: heatIndex, dewPoint

#### Supported Actions
- `relay_on(n)` / `relay_off(n)` - Control output relays
- `pwm_set(n, duty)` - PWM output control
- `alert(type)` - Send alert to server
- `log(message)` - Log custom message

### [Step-by-Step Implementation]

---

## Implementation Roadmap

### Phase 1: Hardware Foundation

| Step | Task | Files to Modify | Expected Outcome |
|------|------|-----------------|------------------|
| 1.1 | Add relay output capability | `ATmega328p.ino`, `Schematic/WeatherStation.sch` | MCP23008 configured with output pins for 4 relays |
| 1.2 | Add EEPROM storage for rules | `ATmega328p.ino` | Functions to read/write rule data to EEPROM |
| 1.3 | Define rule data structures | `automation_rules.h` (new) | Compact structs fitting in EEPROM |

**Step 1.1 Details: Relay Output Configuration**

Modify MCP23008 initialization to add output pins:

```c
// In ATmega328p.ino setup()
#define MCP_RELAY_1_PIN 3
#define MCP_RELAY_2_PIN 4

mcp.pinMode(MCP_RELAY_1_PIN, OUTPUT);
mcp.pinMode(MCP_RELAY_2_PIN, OUTPUT);
mcp.digitalWrite(MCP_RELAY_1_PIN, LOW);
mcp.digitalWrite(MCP_RELAY_2_PIN, LOW);
```

**Step 1.2 Details: EEPROM Storage**

```c
// EEPROM layout for rules
#define EEPROM_RULES_START 0
#define EEPROM_RULES_COUNT_ADDR 0
#define EEPROM_RULES_DATA_ADDR 2
#define MAX_RULES 8
```

**Step 1.3 Details: Rule Data Structures**

Create `automation_rules.h`:

```c
#ifndef _AUTOMATION_RULES_H_
#define _AUTOMATION_RULES_H_

#include <EEPROM.h>

// Sensor types (4 bits)
enum SensorType : uint8_t {
    SENSOR_TEMPERATURE = 0,
    SENSOR_HUMIDITY = 1,
    SENSOR_PRESSURE = 2,
    SENSOR_WIND_SPEED = 3,
    SENSOR_WIND_GUST = 4,
    SENSOR_WIND_DIR = 5,
    SENSOR_RAIN = 6,
    SENSOR_LUX = 7,
    SENSOR_BATTERY = 8,
    SENSOR_HOUR = 9,
    SENSOR_MINUTE = 10,
    SENSOR_HEAT_INDEX = 11
};

// Operators (3 bits)
enum Operator : uint8_t {
    OP_LESS_THAN = 0,
    OP_GREATER_THAN = 1,
    OP_EQUALS = 2,
    OP_NOT_EQUALS = 3,
    OP_LESS_EQUAL = 4,
    OP_GREATER_EQUAL = 5
};

// Action types (4 bits)
enum ActionType : uint8_t {
    ACTION_RELAY_ON = 0,
    ACTION_RELAY_OFF = 1,
    ACTION_PWM_SET = 2,
    ACTION_ALERT = 3,
    ACTION_NONE = 15
};

// Condition structure (4 bytes)
struct Condition {
    uint8_t sensorType : 4;
    uint8_t op : 3;
    uint8_t nextLogic : 1; // 0=AND, 1=OR
    int16_t threshold; // Fixed-point: actual = threshold / 100
    uint8_t reserved;
};

// Action structure (2 bytes)
struct Action {
    uint8_t actionType : 4;
    uint8_t param1 : 4;    // relay number or PWM channel
    uint8_t param2;        // PWM duty cycle or alert type
};

// Complete rule (16 bytes for EEPROM efficiency)
struct AutomationRule {
    uint8_t enabled : 1;
    uint8_t hasElse : 1;
    uint8_t conditionCount : 2;  // 1-3 conditions
    uint8_t reserved : 4;

    Condition conditions[2];     // 8 bytes
    Action thenAction;           // 2 bytes
    Action elseAction;           // 2 bytes

    uint8_t hysteresis;          // Fixed-point / 10
    uint8_t cooldownMinutes;     // Max 255 minutes
};

#define RULE_SIZE sizeof(AutomationRule)  // 16 bytes

#endif
```

---

### Phase 2: Rule Evaluation Engine

| Step | Task | Files to Modify | Expected Outcome |
|------|------|-----------------|------------------|
| 2.1 | Implement condition evaluation | `automation_engine.h` (new) | Function to evaluate single condition |
| 2.2 | Implement rule evaluation | `automation_engine.h` | Function to evaluate complete rule with AND/OR |
| 2.3 | Add hysteresis support | `automation_engine.h` | Prevent rapid on/off cycling |
| 2.4 | Add cooldown tracking | `automation_engine.h` | Rate limiting for actions |

**Step 2.1-2.4 Details: Create `automation_engine.h`**

```c
#ifndef _AUTOMATION_ENGINE_H_
#define _AUTOMATION_ENGINE_H_

#include "automation_rules.h"
#include "weather_readings.h"

// State tracking for hysteresis and cooldowns
struct RuleState {
    bool lastResult;
    uint32_t lastActionTime;
    float lastThresholdValue;
};

RuleState ruleStates[MAX_RULES];

// Get current sensor value
float getSensorValue(SensorType sensor, WeatherReading* reading) {
    switch (sensor) {
        case SENSOR_TEMPERATURE: return reading->temperature;
        case SENSOR_HUMIDITY: return reading->humidity;
        case SENSOR_PRESSURE: return reading->pressure / 100.0; // hPa
        case SENSOR_WIND_SPEED: return reading->windSpeed;
        case SENSOR_WIND_GUST: return reading->windGust;
        case SENSOR_WIND_DIR: return reading->windDirection;
        case SENSOR_RAIN: return reading->rain;
        case SENSOR_LUX: return reading->lux;
        case SENSOR_BATTERY: return reading->batteryMv;
        case SENSOR_HOUR: return RTC.now().hour();
        case SENSOR_MINUTE: return RTC.now().minute();
        case SENSOR_HEAT_INDEX:
            return computeHeatIndex(reading->temperature, reading->humidity, false);
        default: return 0;
    }
}

// Evaluate single condition
bool evaluateCondition(Condition* cond, WeatherReading* reading) {
    float value = getSensorValue((SensorType)cond->sensorType, reading);
    float threshold = cond->threshold / 100.0;

    switch ((Operator)cond->op) {
        case OP_LESS_THAN: return value < threshold;
        case OP_GREATER_THAN: return value > threshold;
        case OP_EQUALS: return abs(value - threshold) < 0.01;
        case OP_NOT_EQUALS: return abs(value - threshold) >= 0.01;
        case OP_LESS_EQUAL: return value <= threshold;
        case OP_GREATER_EQUAL: return value >= threshold;
        default: return false;
    }
}

// Evaluate complete rule with hysteresis
bool evaluateRule(AutomationRule* rule, WeatherReading* reading,
                  uint8_t ruleIndex, uint32_t currentTime) {

    // Check cooldown
    if (rule->cooldownMinutes > 0) {
        uint32_t cooldownSecs = rule->cooldownMinutes * 60;
        if (currentTime - ruleStates[ruleIndex].lastActionTime < cooldownSecs) {
            return ruleStates[ruleIndex].lastResult;
        }
    }

    // Evaluate conditions
    bool result = evaluateCondition(&rule->conditions[0], reading);

    for (int i = 1; i < rule->conditionCount && i < 2; i++) {
        bool condResult = evaluateCondition(&rule->conditions[i], reading);

        if (rule->conditions[i-1].nextLogic == 0) { // AND
            result = result && condResult;
        } else { // OR
            result = result || condResult;
        }
    }

    // Apply hysteresis
    if (rule->hysteresis > 0 && ruleStates[ruleIndex].lastResult != result) {
        float hysteresisValue = rule->hysteresis / 10.0;
        float primaryValue = getSensorValue(
            (SensorType)rule->conditions[0].sensorType, reading);
        float threshold = rule->conditions[0].threshold / 100.0;

        // Only allow state change if we've moved past hysteresis band
        if (result) {
            // Turning on - check we're sufficiently above threshold
            if (primaryValue < threshold + hysteresisValue) {
                result = ruleStates[ruleIndex].lastResult;
            }
        } else {
            // Turning off - check we're sufficiently below threshold
            if (primaryValue > threshold - hysteresisValue) {
                result = ruleStates[ruleIndex].lastResult;
            }
        }
    }

    return result;
}

// Execute action
void executeAction(Action* action) {
    switch ((ActionType)action->actionType) {
        case ACTION_RELAY_ON:
            mcp.digitalWrite(MCP_RELAY_1_PIN + action->param1, HIGH);
            Serial.print("Relay ON: ");
            Serial.println(action->param1);
            break;

        case ACTION_RELAY_OFF:
            mcp.digitalWrite(MCP_RELAY_1_PIN + action->param1, LOW);
            Serial.print("Relay OFF: ");
            Serial.println(action->param1);
            break;

        case ACTION_PWM_SET:
            // Would require additional hardware
            analogWrite(action->param1, action->param2);
            break;

        case ACTION_ALERT:
            // Queue alert for next ESP transmission
            esp_send_debug_request(String("ALERT,") + action->param1);
            break;

        case ACTION_NONE:
        default:
            break;
    }
}

// Main evaluation loop - call this after each reading
void evaluateAllRules(WeatherReading* reading, uint32_t currentTime) {
    uint8_t ruleCount = EEPROM.read(EEPROM_RULES_COUNT_ADDR);

    for (uint8_t i = 0; i < ruleCount && i < MAX_RULES; i++) {
        AutomationRule rule;
        EEPROM.get(EEPROM_RULES_DATA_ADDR + (i * RULE_SIZE), rule);

        if (!rule.enabled) continue;

        bool result = evaluateRule(&rule, reading, i, currentTime);

        // Execute action if state changed
        if (result != ruleStates[i].lastResult) {
            if (result) {
                executeAction(&rule.thenAction);
            } else if (rule.hasElse) {
                executeAction(&rule.elseAction);
            }

            ruleStates[i].lastResult = result;
            ruleStates[i].lastActionTime = currentTime;
        }
    }
}

#endif
```

---

### Phase 3: Integration and Configuration

| Step | Task | Files to Modify | Expected Outcome |
|------|------|-----------------|------------------|
| 3.1 | Integrate engine into main loop | `ATmega328p.ino` | Rules evaluated after each reading |
| 3.2 | Add serial configuration interface | `ATmega328p.ino` | Commands to add/edit/delete rules |
| 3.3 | Add rule status to server reports | `ATmega328p.ino` | Report relay states and rule triggers |

**Step 3.1 Details: Main Loop Integration**

```c
// In ATmega328p.ino, after line 771 (after creating currentReading)
#include "automation_engine.h"

// After: WeatherReading currentReading = get_averaged_accumulator(sampleAccumulator);
evaluateAllRules(&currentReading, get_unixtime());
```

**Step 3.2 Details: Serial Configuration**

Add command parser for rule management:

```c
// Serial command format: !R[cmd][data]
// !RA[rule_bytes] - Add rule
// !RD[index] - Delete rule
// !RL - List rules
// !RE[index] - Enable rule
// !RX[index] - Disable rule
```

**Step 3.3 Details: Server Reporting**

Add relay states to URL:

```c
// In generate_request_url()
outputUrl += "&r1=";
outputUrl += mcp.digitalRead(MCP_RELAY_1_PIN);
outputUrl += "&r2=";
outputUrl += mcp.digitalRead(MCP_RELAY_2_PIN);
```

---

### Phase 4: Advanced Features (Optional)

| Step | Task | Files to Modify | Expected Outcome |
|------|------|-----------------|------------------|
| 4.1 | Add web-based configuration | `ESP8266.ino` | HTTP endpoint for rule management |
| 4.2 | Add rule scheduling | `automation_rules.h` | Time-window restrictions for rules |
| 4.3 | Add rule chaining | `automation_engine.h` | Rules can trigger other rules |
| 4.4 | Add PID control | `pid_control.h` (new) | Proportional control for gradual adjustments |

---

## Technical Approach Alternatives

### Approach 1: EEPROM-Based Rules (Recommended)

**Description:** Store rules in ATmega328p's internal EEPROM (1KB)

**Pros:**
- No additional hardware required
- Persists across power cycles
- Fast read access
- Simple implementation

**Cons:**
- Limited to ~50-60 rules (16 bytes each)
- EEPROM has limited write cycles (100,000)
- No easy backup/restore

**Implementation Complexity:** Medium

### Approach 2: External SRAM Rules

**Description:** Store rules in existing 23A1024 SRAM alongside weather data

**Pros:**
- Much larger capacity (thousands of rules)
- Unlimited write cycles
- Already have hardware

**Cons:**
- Rules lost on power loss (unless battery-backed)
- More complex memory management
- Potential conflicts with weather data

**Implementation Complexity:** High

### Approach 3: SD Card Storage

**Description:** Add SD card module for rule storage and logging

**Pros:**
- Virtually unlimited storage
- Easy backup/restore (remove card)
- Human-readable rule files possible
- Extended data logging

**Cons:**
- Additional hardware cost (~$5)
- Higher power consumption
- More complex code (FAT filesystem)
- Slower access than EEPROM

**Implementation Complexity:** High

### Approach 4: Server-Side Rules with Local Cache

**Description:** Rules defined on server, cached locally for offline operation

**Pros:**
- Easy web-based configuration
- Unlimited rule complexity
- Version control possible
- No local storage limits

**Cons:**
- Requires server infrastructure
- Initial rules need network
- Sync complexity
- Higher latency for rule changes

**Implementation Complexity:** Very High

---

## Recommendation

**Implement Approach 1 (EEPROM-Based Rules)** for initial release with a migration path to Approach 3 (SD Card) for advanced users.

**Rationale:**
1. Matches the "budget-friendly" project goal
2. No additional hardware cost
3. Sufficient for typical greenhouse automation (5-20 rules)
4. Can add SD card support later as optional upgrade
5. EEPROM write cycle concerns mitigated by infrequent rule changes

---

## Memory Budget Analysis

| Component | RAM (bytes) | Flash (bytes) |
|-----------|-------------|---------------|
| Current firmware | ~1400 | ~22000 |
| Rule structures (8 rules) | 128 | 0 |
| Rule states | 96 | 0 |
| Engine code | 50 | ~2000 |
| **Total with CAE** | **~1674** | **~24000** |
| **ATmega328p capacity** | **2048** | **32768** |
| **Remaining** | **374** | **8768** |

The ATmega328p has sufficient resources for this implementation.

---

## Timeline Estimate

| Phase | Duration | Dependencies |
|-------|----------|--------------|
| Phase 1: Hardware Foundation | 1 week | None |
| Phase 2: Rule Engine | 2 weeks | Phase 1 |
| Phase 3: Integration | 1 week | Phase 2 |
| Phase 4: Advanced (optional) | 2-4 weeks | Phase 3 |
| Testing & Documentation | 1 week | Phase 3 |

**Total: 5-9 weeks** depending on optional features

---

## Conclusion

The ATmega Weather Logger provides a solid foundation for greenhouse automation. The primary gap for PLC-like functionality is the lack of conditional automation - the system collects data but cannot act on it autonomously.

Implementing the Conditional Automation Engine (CAE) would transform this from a passive data logger into an active controller capable of:
- Automated ventilation control
- Irrigation scheduling
- Frost protection
- Light supplementation
- Alert generation

The recommended EEPROM-based approach provides this functionality with zero additional hardware cost while leaving room for future expansion.
