
// bool waitForState(bool state, timeout=10) {
//   for (int i=0; i<timeout; i++) {
//     if (mcp.digitalRead(ONEWIRE_RX_PIN) == state) {
//       return true;
//     }
//
//     delay(1);
//   }
//
//   return false;
// }

// void sendACK() {
//   setTxState(LOW);
//   setTxState(HIGH);
// }
//
// bool waitACK() {
//   // Wait for a low
//   if (!getRxState()) {
//     return false;
//   }
//
//   delay(5);
//
//   // Then a high
//   return getRxState();
// }

#ifdef DEBUG
#define DEBUG_PRINT(...) Serial.print( __VA_ARGS__ )
#define DEBUG_PRINTLN(...) Serial.println( __VA_ARGS__ )
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif

#define CLOCK_HOLD_TIME 6
#define CLOCK_EDGE_TIME 1

#define ACK_TIMEOUT 500
#define MAX_REQUEST_ATTEMPTS 5


void setTxState(bool state) {
  mcp.digitalWrite(ONEWIRE_TX_PIN, state);
}

bool getRxState() {
  return mcp.digitalRead(ONEWIRE_RX_PIN);
}

bool waitForRxState(bool state=false) {
  delay(CLOCK_EDGE_TIME);

  for (int i=0; i<CLOCK_HOLD_TIME; i++) {
    if (getRxState() == state) {
      return true;
    }

    delay(1);
  }

  return false;
}

bool clockOutBit(bool bit) {
  // Wait for master to go low or break
  if (!waitForRxState(LOW)) {
    DEBUG_PRINTLN("... Timed out (LOW)");
    return false;
  }

  setTxState(bit);
  DEBUG_PRINT(bit);

  // It should go high again before the next pulse
  if (!waitForRxState(HIGH)) {
    DEBUG_PRINTLN("... Timed out (HIGH)");
    return false;
  }

  return true;
}

bool getClockedBit(int extraHoldMs=0) {
  // Tell it we want data
  setTxState(LOW);
  delay(CLOCK_HOLD_TIME);
  bool result = getRxState();
  DEBUG_PRINT(result);
  setTxState(HIGH);
  delay(CLOCK_HOLD_TIME);
  return result;
}

bool initTransfer() {
  // Tell it we want data
  setTxState(LOW);

  bool gotAck = false;

  // Wait for it to go high
  for (int i=0; i<ACK_TIMEOUT; i++) {
    if (getRxState()) {
      gotAck = true;
      break;
    }

    delay(1);
  }

  // Then we go high
  setTxState(HIGH);

  delay(CLOCK_HOLD_TIME);

  return gotAck;
}


void sendResponse(uint16_t data, int dataLen=16) {
  // Send that transfer is started and we're listening as high ACK bit
  DEBUG_PRINT("Got request sending ACK: ");
  if (!clockOutBit(1)) {
    DEBUG_PRINTLN(" Failed (No Response)");
    return;
  }
  DEBUG_PRINTLN(" Success!");

  // CRC
  DEBUG_PRINT("Sending CRC (");
  DEBUG_PRINT(data%2);
  DEBUG_PRINT("): ");
  if (!clockOutBit(data%2)) {
    DEBUG_PRINTLN(" Failed!");
    return;
  }
  DEBUG_PRINTLN(" Success!");

  DEBUG_PRINT("Sending bits: ");

  // Send each bit when the master goes low
  for (int i=0; i<dataLen; i++) {
    if (!clockOutBit((data>>i)&1)) {
      break;
    }
  }

  DEBUG_PRINTLN();

  // Done
  setTxState(LOW);

  Serial.print("Sent data Sucessfully: ");
  Serial.println(data);
}


uint16_t sendRequest(int dataLen=16) {
  uint16_t result = 0;
  bool success = false;

  for (int attemptNum=0; attemptNum < MAX_REQUEST_ATTEMPTS; attemptNum++) {
    DEBUG_PRINT("Getting ACK: ");

    if (!initTransfer()) {
      DEBUG_PRINTLN("No ACK!");
      continue;
    }

    DEBUG_PRINTLN();

    // Get the crc of the reply data
    bool crc = getClockedBit();

    DEBUG_PRINT("Got CRC: ");
    DEBUG_PRINTLN(crc);

    DEBUG_PRINT("Getting bits: ");

    // It'll now send its bits
    for (int i=0; i<dataLen; i++) {
      result |= getClockedBit()<<i;
    }

    DEBUG_PRINTLN();

    // High again till we pull low again
    setTxState(HIGH);

    if (result%2 != crc) {
      DEBUG_PRINTLN("CRC failed!");
      continue;
    } else {
      DEBUG_PRINTLN("CRC passed!");
      success = true;
      break;
    }
  }

  if (success) {
    Serial.print("Request Successful!: ");
    Serial.println(result);
  } else {
    Serial.println("Request Failed!");
  }

  return result;
}

bool checkRequest() {
  if (getRxState() == LOW) {
    return true;
  }

  return false;
}
