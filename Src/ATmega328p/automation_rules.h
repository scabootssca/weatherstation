/*
Copyright 2024 Weather Station Project Contributors

Conditional Automation Engine - Rule Definitions
Provides PLC-like automation capability for greenhouse and garden control.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
*/

#ifndef _AUTOMATION_RULES_H_
#define _AUTOMATION_RULES_H_

#include <EEPROM.h>

// EEPROM Layout for automation rules
#define EEPROM_RULES_MAGIC_ADDR 0      // 2 bytes - magic number to verify initialization
#define EEPROM_RULES_COUNT_ADDR 2      // 1 byte - number of active rules
#define EEPROM_RULES_DATA_ADDR 4       // Rules start here
#define EEPROM_RULES_MAGIC 0xCAE1      // "CAE" + version 1

#define MAX_RULES 8                    // Maximum rules (8 * 16 = 128 bytes)
#define MAX_CONDITIONS_PER_RULE 2      // Max conditions per rule

// Sensor types for conditions (4 bits = 0-15)
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
    SENSOR_HEAT_INDEX = 11,
    SENSOR_DEW_POINT = 12,
    SENSOR_DAY_OF_WEEK = 13,
    SENSOR_NONE = 15
};

// Comparison operators (3 bits = 0-7)
enum Operator : uint8_t {
    OP_LESS_THAN = 0,
    OP_GREATER_THAN = 1,
    OP_EQUALS = 2,
    OP_NOT_EQUALS = 3,
    OP_LESS_EQUAL = 4,
    OP_GREATER_EQUAL = 5
};

// Action types (4 bits = 0-15)
enum ActionType : uint8_t {
    ACTION_RELAY_ON = 0,
    ACTION_RELAY_OFF = 1,
    ACTION_RELAY_TOGGLE = 2,
    ACTION_PWM_SET = 3,
    ACTION_ALERT = 4,
    ACTION_LOG = 5,
    ACTION_NONE = 15
};

// Alert types for ACTION_ALERT
enum AlertType : uint8_t {
    ALERT_FROST = 0,
    ALERT_OVERHEAT = 1,
    ALERT_LOW_BATTERY = 2,
    ALERT_HIGH_WIND = 3,
    ALERT_RAIN = 4,
    ALERT_CUSTOM = 15
};

// Single condition structure (4 bytes)
// Threshold is stored as fixed-point: actual_value = threshold / 100.0
struct Condition {
    uint8_t sensorType : 4;   // SensorType enum
    uint8_t op : 3;           // Operator enum
    uint8_t nextLogic : 1;    // 0 = AND with next condition, 1 = OR
    int16_t threshold;        // Fixed-point value (divide by 100 for actual)
    uint8_t padding;          // Alignment padding
};

// Action structure (2 bytes)
struct Action {
    uint8_t actionType : 4;   // ActionType enum
    uint8_t param1 : 4;       // Relay number (0-3), PWM channel, or alert type
    uint8_t param2;           // PWM duty (0-255), or additional parameter
};

// Complete automation rule (16 bytes for EEPROM efficiency)
struct AutomationRule {
    // Flags byte
    uint8_t enabled : 1;           // Rule is active
    uint8_t hasElse : 1;           // Has else action
    uint8_t conditionCount : 2;    // Number of conditions (1-3)
    uint8_t invertResult : 1;      // Invert final result (NOT)
    uint8_t reserved : 3;          // Future use

    // Conditions (8 bytes)
    Condition conditions[MAX_CONDITIONS_PER_RULE];

    // Actions (4 bytes)
    Action thenAction;
    Action elseAction;

    // Behavior modifiers (2 bytes)
    uint8_t hysteresis;            // Hysteresis value / 10 (0-25.5)
    uint8_t cooldownMinutes;       // Cooldown in minutes (0-255)
};

#define RULE_SIZE sizeof(AutomationRule)  // Should be 16 bytes

// Runtime state for each rule (not stored in EEPROM)
struct RuleState {
    bool lastResult;               // Last evaluation result
    uint32_t lastActionTime;       // Unix time of last action execution
    bool actionExecuted;           // Whether action was executed this cycle
};

// Function declarations

// Initialize EEPROM storage for rules (call once in setup)
void initAutomationRules() {
    uint16_t magic;
    EEPROM.get(EEPROM_RULES_MAGIC_ADDR, magic);

    if (magic != EEPROM_RULES_MAGIC) {
        // First time - initialize EEPROM
        Serial.println(F("Initializing automation rules EEPROM"));
        EEPROM.put(EEPROM_RULES_MAGIC_ADDR, (uint16_t)EEPROM_RULES_MAGIC);
        EEPROM.write(EEPROM_RULES_COUNT_ADDR, 0);

        // Clear rule slots
        AutomationRule emptyRule;
        memset(&emptyRule, 0, sizeof(AutomationRule));
        for (uint8_t i = 0; i < MAX_RULES; i++) {
            EEPROM.put(EEPROM_RULES_DATA_ADDR + (i * RULE_SIZE), emptyRule);
        }
    }

    uint8_t ruleCount = EEPROM.read(EEPROM_RULES_COUNT_ADDR);
    Serial.print(F("Automation rules loaded: "));
    Serial.println(ruleCount);
}

// Get number of configured rules
uint8_t getRuleCount() {
    return EEPROM.read(EEPROM_RULES_COUNT_ADDR);
}

// Read a rule from EEPROM
void getRule(uint8_t index, AutomationRule* rule) {
    if (index >= MAX_RULES) {
        memset(rule, 0, sizeof(AutomationRule));
        return;
    }
    EEPROM.get(EEPROM_RULES_DATA_ADDR + (index * RULE_SIZE), *rule);
}

// Write a rule to EEPROM
bool setRule(uint8_t index, AutomationRule* rule) {
    if (index >= MAX_RULES) {
        return false;
    }
    EEPROM.put(EEPROM_RULES_DATA_ADDR + (index * RULE_SIZE), *rule);

    // Update count if needed
    uint8_t currentCount = getRuleCount();
    if (index >= currentCount) {
        EEPROM.write(EEPROM_RULES_COUNT_ADDR, index + 1);
    }
    return true;
}

// Delete a rule (shift others down)
bool deleteRule(uint8_t index) {
    uint8_t count = getRuleCount();
    if (index >= count) {
        return false;
    }

    // Shift rules down
    for (uint8_t i = index; i < count - 1; i++) {
        AutomationRule rule;
        getRule(i + 1, &rule);
        setRule(i, &rule);
    }

    // Clear last slot and decrement count
    AutomationRule emptyRule;
    memset(&emptyRule, 0, sizeof(AutomationRule));
    EEPROM.put(EEPROM_RULES_DATA_ADDR + ((count - 1) * RULE_SIZE), emptyRule);
    EEPROM.write(EEPROM_RULES_COUNT_ADDR, count - 1);

    return true;
}

// Print rule details for debugging
void printRule(uint8_t index) {
    AutomationRule rule;
    getRule(index, &rule);

    Serial.print(F("Rule "));
    Serial.print(index);
    Serial.print(F(": "));
    Serial.println(rule.enabled ? F("ENABLED") : F("DISABLED"));

    Serial.print(F("  Conditions: "));
    Serial.println(rule.conditionCount);

    for (uint8_t i = 0; i < rule.conditionCount && i < MAX_CONDITIONS_PER_RULE; i++) {
        Serial.print(F("    ["));
        Serial.print(i);
        Serial.print(F("] Sensor="));
        Serial.print(rule.conditions[i].sensorType);
        Serial.print(F(" Op="));
        Serial.print(rule.conditions[i].op);
        Serial.print(F(" Thresh="));
        Serial.print(rule.conditions[i].threshold / 100.0);
        if (i < rule.conditionCount - 1) {
            Serial.print(rule.conditions[i].nextLogic ? F(" OR") : F(" AND"));
        }
        Serial.println();
    }

    Serial.print(F("  Then: Action="));
    Serial.print(rule.thenAction.actionType);
    Serial.print(F(" P1="));
    Serial.print(rule.thenAction.param1);
    Serial.print(F(" P2="));
    Serial.println(rule.thenAction.param2);

    if (rule.hasElse) {
        Serial.print(F("  Else: Action="));
        Serial.print(rule.elseAction.actionType);
        Serial.print(F(" P1="));
        Serial.print(rule.elseAction.param1);
        Serial.print(F(" P2="));
        Serial.println(rule.elseAction.param2);
    }

    Serial.print(F("  Hysteresis="));
    Serial.print(rule.hysteresis / 10.0);
    Serial.print(F(" Cooldown="));
    Serial.print(rule.cooldownMinutes);
    Serial.println(F("min"));
}

// Create a simple threshold rule helper
AutomationRule createThresholdRule(
    SensorType sensor,
    Operator op,
    float threshold,
    ActionType thenAction,
    uint8_t thenParam,
    ActionType elseAction = ACTION_NONE,
    uint8_t elseParam = 0,
    float hysteresis = 0.0,
    uint8_t cooldownMins = 0
) {
    AutomationRule rule;
    memset(&rule, 0, sizeof(AutomationRule));

    rule.enabled = 1;
    rule.conditionCount = 1;
    rule.hasElse = (elseAction != ACTION_NONE);

    rule.conditions[0].sensorType = sensor;
    rule.conditions[0].op = op;
    rule.conditions[0].threshold = (int16_t)(threshold * 100);
    rule.conditions[0].nextLogic = 0;

    rule.thenAction.actionType = thenAction;
    rule.thenAction.param1 = thenParam;
    rule.thenAction.param2 = 0;

    rule.elseAction.actionType = elseAction;
    rule.elseAction.param1 = elseParam;
    rule.elseAction.param2 = 0;

    rule.hysteresis = (uint8_t)(hysteresis * 10);
    rule.cooldownMinutes = cooldownMins;

    return rule;
}

#endif // _AUTOMATION_RULES_H_
