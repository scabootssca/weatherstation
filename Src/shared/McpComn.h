
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

#define CLOCK_HOLD_TIME 30
#define CLOCK_EDGE_TIME 10

#define ACK_TIMEOUT 200
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
    Serial.println("... Timed out");
    return false;
  }

  setTxState(bit);
  //Serial.print(bit);

  // It should go high again before the next pulse
  if (!waitForRxState(HIGH)) {
    Serial.println("... Timed out");
    return false;
  }

  return true;
}

bool getClockedBit(int extraHoldMs=0) {
  // Tell it we want data
  setTxState(LOW);
  delay(CLOCK_HOLD_TIME);
  bool result = getRxState();
  setTxState(HIGH);
  delay(CLOCK_HOLD_TIME);
  return result;
}

bool getAck() {
  // Tell it we want data
  setTxState(LOW);

  bool gotAck = false;

  for (int i=0; i<ACK_TIMEOUT; i++) {
    if (getRxState()) {
      gotAck = true;
      break;
    }

    delay(1);
  }

  setTxState(HIGH);

  delay(CLOCK_HOLD_TIME);

  return gotAck;
}


void sendResponse(int data, int dataLen=16) {
  // The high ACK
  //Serial.print("Sending ACK: ");
  if (!clockOutBit(1)) {
    return;
  }
  //Serial.println();

  // CRC
  //Serial.print("Sending CRC: ");
  if (!clockOutBit(data%2)) {
    return;
  }
  //Serial.println();

  // /* This'll send the LSB first */
  // Serial.print("Sending data (");
  // Serial.print(data);
  // Serial.print("): ");

  // Send each bit when the master goes low
  for (int i=0; i<dataLen; i++) {
    if (!clockOutBit((data>>i)&1)) {
      break;
    }
  }

  // Done
  setTxState(LOW);

  Serial.print("Sent data: ");
  Serial.println(data);
  //Serial.println("");
}


int sendRequest(int dataLen=16) {
  int result = 0;

  for (int attemptNum=0; attemptNum < MAX_REQUEST_ATTEMPTS; attemptNum++) {
    if (!getAck()) {
      Serial.println("No ACK!");
      continue;
      //return result;
    }

    bool crc = getClockedBit();

    // It'll now send its bits
    for (int i=0; i<dataLen; i++) {
      result |= getClockedBit()<<i;
    }

    // High again till we pull low again
    setTxState(HIGH);

    if (result%2 != crc) {
      Serial.println("CRC failed!");
      continue;
    } else {
      Serial.println("CRC passed!");
      break;
    }
  }

  return result;
}

bool checkRequest() {
  if (getRxState() == LOW) {
    return true;
  }

  return false;
}
