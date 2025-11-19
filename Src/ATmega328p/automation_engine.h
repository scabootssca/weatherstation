/*
Copyright 2024 Weather Station Project Contributors

Conditional Automation Engine - Rule Evaluation
Evaluates rules and executes actions for PLC-like control.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
*/

#ifndef _AUTOMATION_ENGINE_H_
#define _AUTOMATION_ENGINE_H_

#include "automation_rules.h"
#include "weather_readings.h"
#include "helpers.h"

// Forward declarations for external dependencies
extern Adafruit_MCP23008 mcp;
extern RTC_DS1307 RTC;
extern void esp_send_debug_request(String message);

// Relay pin definitions on MCP23008
#define MCP_RELAY_1_PIN 3
#define MCP_RELAY_2_PIN 4
#define NUM_RELAYS 2

// Runtime state for all rules
RuleState ruleStates[MAX_RULES];

// Relay states for reporting
uint8_t relayStates = 0;

// Initialize the automation engine
void initAutomationEngine() {
    // Initialize rule states
    for (uint8_t i = 0; i < MAX_RULES; i++) {
        ruleStates[i].lastResult = false;
        ruleStates[i].lastActionTime = 0;
        ruleStates[i].actionExecuted = false;
    }

    // Initialize relay pins on MCP23008
    mcp.pinMode(MCP_RELAY_1_PIN, OUTPUT);
    mcp.pinMode(MCP_RELAY_2_PIN, OUTPUT);
    mcp.digitalWrite(MCP_RELAY_1_PIN, LOW);
    mcp.digitalWrite(MCP_RELAY_2_PIN, LOW);

    // Initialize EEPROM storage
    initAutomationRules();

    Serial.println(F("Automation engine initialized"));
}

// Calculate dew point from temperature and humidity
float calculateDewPoint(float tempC, float humidity) {
    // Magnus formula approximation
    float a = 17.27;
    float b = 237.7;
    float alpha = ((a * tempC) / (b + tempC)) + log(humidity / 100.0);
    return (b * alpha) / (a - alpha);
}

// Get current value for a sensor type
float getSensorValue(SensorType sensor, WeatherReading* reading) {
    DateTime now = RTC.now();

    switch (sensor) {
        case SENSOR_TEMPERATURE:
            return reading->temperature;

        case SENSOR_HUMIDITY:
            return reading->humidity;

        case SENSOR_PRESSURE:
            return reading->pressure / 100.0; // Convert to hPa

        case SENSOR_WIND_SPEED:
            return reading->windSpeed;

        case SENSOR_WIND_GUST:
            return reading->windGust;

        case SENSOR_WIND_DIR:
            return reading->windDirection;

        case SENSOR_RAIN:
            return (float)reading->rain;

        case SENSOR_LUX:
            return reading->lux;

        case SENSOR_BATTERY:
            return reading->batteryMv;

        case SENSOR_HOUR:
            return (float)now.hour();

        case SENSOR_MINUTE:
            return (float)now.minute();

        case SENSOR_HEAT_INDEX:
            return computeHeatIndex(reading->temperature, reading->humidity, false);

        case SENSOR_DEW_POINT:
            return calculateDewPoint(reading->temperature, reading->humidity);

        case SENSOR_DAY_OF_WEEK:
            return (float)now.dayOfTheWeek(); // 0=Sunday, 6=Saturday

        default:
            return 0.0;
    }
}

// Get sensor name for debugging
const char* getSensorName(SensorType sensor) {
    switch (sensor) {
        case SENSOR_TEMPERATURE: return "Temp";
        case SENSOR_HUMIDITY: return "Hum";
        case SENSOR_PRESSURE: return "Press";
        case SENSOR_WIND_SPEED: return "Wind";
        case SENSOR_WIND_GUST: return "Gust";
        case SENSOR_WIND_DIR: return "WDir";
        case SENSOR_RAIN: return "Rain";
        case SENSOR_LUX: return "Lux";
        case SENSOR_BATTERY: return "Bat";
        case SENSOR_HOUR: return "Hour";
        case SENSOR_MINUTE: return "Min";
        case SENSOR_HEAT_INDEX: return "HeatIdx";
        case SENSOR_DEW_POINT: return "DewPt";
        case SENSOR_DAY_OF_WEEK: return "Day";
        default: return "?";
    }
}

// Evaluate a single condition
bool evaluateCondition(Condition* cond, WeatherReading* reading) {
    if (cond->sensorType == SENSOR_NONE) {
        return true; // Empty condition always passes
    }

    float value = getSensorValue((SensorType)cond->sensorType, reading);
    float threshold = cond->threshold / 100.0;

    bool result;
    switch ((Operator)cond->op) {
        case OP_LESS_THAN:
            result = value < threshold;
            break;
        case OP_GREATER_THAN:
            result = value > threshold;
            break;
        case OP_EQUALS:
            result = fabs(value - threshold) < 0.01;
            break;
        case OP_NOT_EQUALS:
            result = fabs(value - threshold) >= 0.01;
            break;
        case OP_LESS_EQUAL:
            result = value <= threshold;
            break;
        case OP_GREATER_EQUAL:
            result = value >= threshold;
            break;
        default:
            result = false;
    }

    #if DEBUG >= 3
    Serial.print(F("    Eval: "));
    Serial.print(getSensorName((SensorType)cond->sensorType));
    Serial.print(F("="));
    Serial.print(value);
    Serial.print(F(" "));
    switch ((Operator)cond->op) {
        case OP_LESS_THAN: Serial.print(F("<")); break;
        case OP_GREATER_THAN: Serial.print(F(">")); break;
        case OP_EQUALS: Serial.print(F("==")); break;
        case OP_NOT_EQUALS: Serial.print(F("!=")); break;
        case OP_LESS_EQUAL: Serial.print(F("<=")); break;
        case OP_GREATER_EQUAL: Serial.print(F(">=")); break;
    }
    Serial.print(F(" "));
    Serial.print(threshold);
    Serial.print(F(" -> "));
    Serial.println(result ? F("TRUE") : F("FALSE"));
    #endif

    return result;
}

// Evaluate a complete rule with all conditions
bool evaluateRule(AutomationRule* rule, WeatherReading* reading,
                  uint8_t ruleIndex, uint32_t currentTime) {

    // Check cooldown first
    if (rule->cooldownMinutes > 0 && ruleStates[ruleIndex].lastActionTime > 0) {
        uint32_t cooldownSecs = (uint32_t)rule->cooldownMinutes * 60;
        if (currentTime - ruleStates[ruleIndex].lastActionTime < cooldownSecs) {
            // Still in cooldown, return last result without triggering action
            return ruleStates[ruleIndex].lastResult;
        }
    }

    // Evaluate first condition
    bool result = evaluateCondition(&rule->conditions[0], reading);

    // Evaluate additional conditions with AND/OR logic
    for (uint8_t i = 1; i < rule->conditionCount && i < MAX_CONDITIONS_PER_RULE; i++) {
        bool condResult = evaluateCondition(&rule->conditions[i], reading);

        if (rule->conditions[i - 1].nextLogic == 0) {
            // AND
            result = result && condResult;
        } else {
            // OR
            result = result || condResult;
        }
    }

    // Apply NOT if configured
    if (rule->invertResult) {
        result = !result;
    }

    // Apply hysteresis to prevent rapid switching
    if (rule->hysteresis > 0 && ruleStates[ruleIndex].lastResult != result) {
        float hysteresisValue = rule->hysteresis / 10.0;
        float primaryValue = getSensorValue(
            (SensorType)rule->conditions[0].sensorType, reading);
        float threshold = rule->conditions[0].threshold / 100.0;

        // Determine if we should allow the state change
        bool allowChange = false;

        if (result) {
            // Trying to turn ON - only if we're past threshold + hysteresis
            Operator op = (Operator)rule->conditions[0].op;
            if (op == OP_GREATER_THAN || op == OP_GREATER_EQUAL) {
                allowChange = primaryValue > (threshold + hysteresisValue);
            } else if (op == OP_LESS_THAN || op == OP_LESS_EQUAL) {
                allowChange = primaryValue < (threshold - hysteresisValue);
            } else {
                allowChange = true;
            }
        } else {
            // Trying to turn OFF - only if we're past threshold - hysteresis
            Operator op = (Operator)rule->conditions[0].op;
            if (op == OP_GREATER_THAN || op == OP_GREATER_EQUAL) {
                allowChange = primaryValue < (threshold - hysteresisValue);
            } else if (op == OP_LESS_THAN || op == OP_LESS_EQUAL) {
                allowChange = primaryValue > (threshold + hysteresisValue);
            } else {
                allowChange = true;
            }
        }

        if (!allowChange) {
            result = ruleStates[ruleIndex].lastResult; // Keep previous state
        }
    }

    return result;
}

// Execute an action
void executeAction(Action* action, uint8_t ruleIndex, bool isThen) {
    if (action->actionType == ACTION_NONE) {
        return;
    }

    uint8_t relayPin;

    switch ((ActionType)action->actionType) {
        case ACTION_RELAY_ON:
            if (action->param1 < NUM_RELAYS) {
                relayPin = MCP_RELAY_1_PIN + action->param1;
                mcp.digitalWrite(relayPin, HIGH);
                relayStates |= (1 << action->param1);

                Serial.print(F("Relay "));
                Serial.print(action->param1);
                Serial.println(F(" ON"));
            }
            break;

        case ACTION_RELAY_OFF:
            if (action->param1 < NUM_RELAYS) {
                relayPin = MCP_RELAY_1_PIN + action->param1;
                mcp.digitalWrite(relayPin, LOW);
                relayStates &= ~(1 << action->param1);

                Serial.print(F("Relay "));
                Serial.print(action->param1);
                Serial.println(F(" OFF"));
            }
            break;

        case ACTION_RELAY_TOGGLE:
            if (action->param1 < NUM_RELAYS) {
                relayPin = MCP_RELAY_1_PIN + action->param1;
                uint8_t current = mcp.digitalRead(relayPin);
                mcp.digitalWrite(relayPin, !current);

                if (current) {
                    relayStates &= ~(1 << action->param1);
                } else {
                    relayStates |= (1 << action->param1);
                }

                Serial.print(F("Relay "));
                Serial.print(action->param1);
                Serial.println(current ? F(" OFF (toggle)") : F(" ON (toggle)"));
            }
            break;

        case ACTION_PWM_SET:
            // PWM would require additional hardware setup
            // param1 = channel, param2 = duty cycle
            Serial.print(F("PWM "));
            Serial.print(action->param1);
            Serial.print(F(" = "));
            Serial.println(action->param2);
            break;

        case ACTION_ALERT:
            {
                String alertMsg = "RULE_ALERT,";
                alertMsg += ruleIndex;
                alertMsg += ",";
                alertMsg += action->param1; // Alert type

                esp_send_debug_request(alertMsg);

                Serial.print(F("Alert sent: type="));
                Serial.println(action->param1);
            }
            break;

        case ACTION_LOG:
            Serial.print(F("Rule "));
            Serial.print(ruleIndex);
            Serial.print(F(" triggered: "));
            Serial.println(isThen ? F("THEN") : F("ELSE"));
            break;

        default:
            break;
    }
}

// Main function to evaluate all rules - call after each weather reading
void evaluateAllRules(WeatherReading* reading, uint32_t currentTime) {
    uint8_t ruleCount = getRuleCount();

    if (ruleCount == 0) {
        return;
    }

    #if DEBUG >= 2
    Serial.println(F("--Evaluating automation rules--"));
    #endif

    for (uint8_t i = 0; i < ruleCount && i < MAX_RULES; i++) {
        AutomationRule rule;
        getRule(i, &rule);

        if (!rule.enabled) {
            continue;
        }

        #if DEBUG >= 3
        Serial.print(F("  Rule "));
        Serial.print(i);
        Serial.println(F(":"));
        #endif

        bool result = evaluateRule(&rule, reading, i, currentTime);
        bool stateChanged = (result != ruleStates[i].lastResult);

        #if DEBUG >= 2
        Serial.print(F("  Rule "));
        Serial.print(i);
        Serial.print(F(": "));
        Serial.print(result ? F("TRUE") : F("FALSE"));
        if (stateChanged) {
            Serial.print(F(" (CHANGED)"));
        }
        Serial.println();
        #endif

        // Execute action if state changed
        if (stateChanged) {
            if (result) {
                executeAction(&rule.thenAction, i, true);
            } else if (rule.hasElse) {
                executeAction(&rule.elseAction, i, false);
            }

            ruleStates[i].lastResult = result;
            ruleStates[i].lastActionTime = currentTime;
            ruleStates[i].actionExecuted = true;
        } else {
            ruleStates[i].actionExecuted = false;
        }
    }
}

// Get current relay states as bitmask for reporting
uint8_t getRelayStates() {
    return relayStates;
}

// Manually set a relay (for testing or override)
void setRelay(uint8_t relay, bool state) {
    if (relay >= NUM_RELAYS) {
        return;
    }

    uint8_t pin = MCP_RELAY_1_PIN + relay;
    mcp.digitalWrite(pin, state ? HIGH : LOW);

    if (state) {
        relayStates |= (1 << relay);
    } else {
        relayStates &= ~(1 << relay);
    }
}

// Get state of a specific relay
bool getRelayState(uint8_t relay) {
    if (relay >= NUM_RELAYS) {
        return false;
    }
    return (relayStates >> relay) & 1;
}

// Print all rule states for debugging
void printRuleStates() {
    uint8_t count = getRuleCount();
    Serial.println(F("Rule States:"));

    for (uint8_t i = 0; i < count; i++) {
        Serial.print(F("  ["));
        Serial.print(i);
        Serial.print(F("] "));
        Serial.print(ruleStates[i].lastResult ? F("ON") : F("OFF"));
        Serial.print(F(" LastAction="));
        Serial.println(ruleStates[i].lastActionTime);
    }

    Serial.print(F("Relays: "));
    for (uint8_t i = 0; i < NUM_RELAYS; i++) {
        Serial.print(getRelayState(i) ? F("1") : F("0"));
    }
    Serial.println();
}

// Create and add example rules for greenhouse automation
void createExampleRules() {
    uint8_t idx = 0;

    // Rule 0: Frost Protection - Turn on heater if temp < 2C
    AutomationRule frostRule = createThresholdRule(
        SENSOR_TEMPERATURE,
        OP_LESS_THAN,
        2.0,                  // 2 degrees C
        ACTION_RELAY_ON, 0,   // Turn on relay 0
        ACTION_RELAY_OFF, 0,  // Turn off relay 0
        1.0,                  // 1 degree hysteresis
        5                     // 5 minute cooldown
    );
    setRule(idx++, &frostRule);

    // Rule 1: Ventilation - Open vent if temp > 28C
    AutomationRule ventRule = createThresholdRule(
        SENSOR_TEMPERATURE,
        OP_GREATER_THAN,
        28.0,                 // 28 degrees C
        ACTION_RELAY_ON, 1,   // Turn on relay 1
        ACTION_RELAY_OFF, 1,  // Turn off relay 1
        2.0,                  // 2 degree hysteresis
        0                     // No cooldown
    );
    setRule(idx++, &ventRule);

    Serial.print(F("Created "));
    Serial.print(idx);
    Serial.println(F(" example rules"));
}

#endif // _AUTOMATION_ENGINE_H_
