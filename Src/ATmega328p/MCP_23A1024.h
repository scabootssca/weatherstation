/*
On Esp:
  int: 4
  double: 8
  float: 4
  char: 1
  long: 4
*/
//
#define SRAM_DEBUG 1
#define SRAM_MODE_READ 0
#define SRAM_MODE_WRITE 1
#define SRAM_MAX_ADDRESS 0x1FFFF // 131072 (2^17) addresses

/***************************
* SRAM memory layout
***************************/
/*
[0-1]    populated
[2-5]    readIndex
[6-10]   writeIndex
[11-31]  BLANK
[32-127] readingAccumulator
[128+]   readings
*/
#define SRAM_POPULATED_KEY 0b0110100110010110 // 27030

// Hopefully it won't randomly be this
#define SRAM_ADDR_POPULATED 0
#define SRAM_SIZE_POPULATED 2

// Reading Buffer Pointers
#define SRAM_ADDR_READINGS_READ_INDEX 2
#define SRAM_SIZE_READINGS_READ_INDEX 4
#define SRAM_ADDR_READINGS_WRITE_INDEX 6
#define SRAM_SIZE_READINGS_WRITE_INDEX 4

// Ongoing accumulator
#define SRAM_ADDR_ACCUMULATOR 32
#define SRAM_SIZE_ACCUMULATOR 96

// Readings will directly follow in sram, next page (Each reading will use 1 page)
#define SRAM_ADDR_READINGS 128
#define SRAM_SIZE_READINGS 32
#define SRAM_MAX_READINGS int((SRAM_MAX_ADDRESS-SRAM_ADDR_READINGS)/SRAM_SIZE_READINGS)

void print_bin(uint64_t bytes, int numBits, bool newLine=true) {
  // Bytes in the union have the lsb as 0

  if (newLine) {
    Serial.print("Bytes: ");
  }

  for (int bit=0; bit<numBits; bit++) {
    // Space between bytes
    if (bit > 0 && bit%8 == 0) {
      Serial.print(" ");
    }

    Serial.print( ((bytes >> (numBits-1-bit)) & 1) ? "1":"0" );
  }

  if (newLine) {
    Serial.println();
  }
}

void sram_init(int csPin) {
  Serial.println("Initilizing SRAM");

  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, LOW);
  delay(1);
  digitalWrite(csPin, HIGH);
}

bool sram_begin_transaction(bool mode, uint32_t address, int size) {
  // Only 1mbit
  if (address+size > SRAM_MAX_ADDRESS) {
    Serial.println("Problem starting SRAM transaction (address overflow)");
    return false;
  }

  #if SRAM_DEBUG
  Serial.println();
  Serial.print("Starting SRAM ");
  Serial.print((mode == SRAM_MODE_READ)?"READ":"WRITE");
  Serial.print(" at address: ");
  Serial.println(address);
  #endif


  uint32_t spiHz = 10000000; /* Seems it doesn't it wants to take longer, so a slower clock */
  SPI.beginTransaction(SPISettings(spiHz, MSBFIRST, SPI_MODE0));
  digitalWrite(SRAM_CS_PIN, LOW);
  delay(1);

  SPI.transfer((mode == SRAM_MODE_READ)?0b00000011:0b00000010); // Read cmd: Write cmd

  SPI.transfer(uint8_t(address >> 16));
  SPI.transfer(uint8_t(address >> 8));
  SPI.transfer(uint8_t(address));

  return true;
}

void sram_end_transaction() {
  digitalWrite(SRAM_CS_PIN, HIGH);
  SPI.endTransaction();

  #if SRAM_DEBUG
  Serial.println("Finished SRAM transfer");
  #endif
}

uint8_t sram_transfer(uint8_t value) {
  Serial.print("Sending (uint8_t): ");
  print_bin(value, 8, false);
  Serial.println();

  uint8_t result = SPI.transfer(value);

  Serial.print("Recieved (uint8_t): ");
  print_bin(result, 16, false);
  Serial.println();

  return result;
}

uint16_t sram_transfer(uint16_t value) {
  Serial.print("Sending (uint16_t): ");
  print_bin(value, 16, false);
  Serial.println();

  uint16_t result = SPI.transfer16(value);

  Serial.print("Recieved (uint16_t): ");
  print_bin(result, 16, false);
  Serial.println();

  return result;
}

uint32_t sram_transfer(uint32_t value) {
  Serial.print("Sending (uint32_t): ");
  print_bin(value, 32, false);
  Serial.println();

  union {
    uint32_t value;

    struct {
      uint16_t bit0;
      uint16_t bit1;
    };
  } send, recv;

  send.value = value;

  recv.bit0 = SPI.transfer16(send.bit0);
  recv.bit1 = SPI.transfer16(send.bit1);


  Serial.print("Recieved (uint32_t): ");
  print_bin(recv.value, 32, false);
  Serial.println();

  return recv.value;
}

uint64_t sram_transfer(uint64_t value) {
  Serial.print("Sending (uint64_t): ");
  print_bin(value, 64, false);
  Serial.println();

  union {
    uint64_t value;

    struct {
      uint16_t bit0;
      uint16_t bit1;
      uint16_t bit2;
      uint16_t bit3;
    };
  } send, recv;

  send.value = value;

  recv.bit0 = SPI.transfer16(send.bit0);
  recv.bit1 = SPI.transfer16(send.bit1);
  recv.bit2 = SPI.transfer16(send.bit2);
  recv.bit3 = SPI.transfer16(send.bit3);

  Serial.print("Recieved (uint64_t): ");
  print_bin(recv.value, 64, false);
  Serial.println();

  return recv.value;
}

void sram_read_accumulator(WeatherReadingAccumulator *sampleAccumulator) {
  sram_begin_transaction(SRAM_MODE_READ, SRAM_ADDR_ACCUMULATOR, SRAM_SIZE_ACCUMULATOR);
  
  sampleAccumulator->timestamp = sram_transfer(uint64_t(0));
  sampleAccumulator->temperature = static_cast<double>(sram_transfer(uint64_t(0)));
  sampleAccumulator->humidity = static_cast<double>(sram_transfer(uint64_t(0)));
  sampleAccumulator->pressure = static_cast<double>(sram_transfer(uint64_t(0)));
  sampleAccumulator->battery = static_cast<double>(sram_transfer(uint64_t(0)));
  sampleAccumulator->windSpeed = static_cast<double>(sram_transfer(uint64_t(0)));
  sampleAccumulator->windDirection = static_cast<double>(sram_transfer(uint64_t(0)));

  sampleAccumulator->numSamples = sram_transfer(uint32_t(0));
  sampleAccumulator->numBatterySamples = sram_transfer(uint32_t(0));

  sram_end_transaction();
}

void sram_write_accumulator(WeatherReadingAccumulator *sampleAccumulator) {
  sram_begin_transaction(SRAM_MODE_WRITE, SRAM_ADDR_ACCUMULATOR, SRAM_SIZE_ACCUMULATOR);

  sram_transfer(sampleAccumulator->timestamp);
  sram_transfer(static_cast<uint64_t>(sampleAccumulator->temperature));
  sram_transfer(static_cast<uint64_t>(sampleAccumulator->humidity));
  sram_transfer(static_cast<uint64_t>(sampleAccumulator->pressure));
  sram_transfer(static_cast<uint64_t>(sampleAccumulator->battery));
  sram_transfer(static_cast<uint64_t>(sampleAccumulator->windSpeed));
  sram_transfer(static_cast<uint64_t>(sampleAccumulator->windDirection));

  sram_transfer(sampleAccumulator->numSamples);
  sram_transfer(sampleAccumulator->numBatterySamples);

  sram_end_transaction();
}

bool sram_restore(WeatherReadingAccumulator *sampleAccumulator) {
  // See if the populated key was set
  sram_begin_transaction(SRAM_MODE_READ, SRAM_ADDR_POPULATED, SRAM_SIZE_POPULATED);
  uint16_t storedValue = sram_transfer(uint16_t(0));
  bool previouslyPopulated = (storedValue == SRAM_POPULATED_KEY);
  sram_end_transaction();

  Serial.print("Stored value: ");
  print_bin(storedValue, 16, false);
  Serial.println();

  // Set the populated key
  sram_begin_transaction(SRAM_MODE_WRITE, SRAM_ADDR_POPULATED, SRAM_SIZE_POPULATED);
  sram_transfer(uint16_t(SRAM_POPULATED_KEY));
  sram_end_transaction();

  // Read the accumulator here
  if (previouslyPopulated) {
    sram_read_accumulator(sampleAccumulator);
  } else {
    sram_write_accumulator(sampleAccumulator);
  }

  return previouslyPopulated ;
}
//
// uint8_t sram_transfer(uint8_t value = 0) {
//   uint8_t reply = SPI.transfer(value);
//
//   #if SRAM_DEBUG
//   Serial.print("SRAM Transfer (uint8_t (");
//   Serial.print(sizeof(uint8_t));
//   Serial.print(")): ");
//   Serial.print(value);
//   Serial.print(" -> ");
//   Serial.println(reply);
//   #endif
//
//   return reply;
// }
//
// bool sram_transfer(bool value = 0) {
//   bool reply = SPI.transfer(value);
//
//   #if SRAM_DEBUG
//   Serial.print("SRAM Transfer (bool (");
//   Serial.print(sizeof(bool));
//   Serial.print(")): ");
//   Serial.print(value);
//   Serial.print(" -> ");
//   Serial.println(reply);
//   #endif
//
//   return reply;
// }
//
// uint32_t sram_transfer(uint32_t value = 0) {
//   uint8_t *outBytes = reinterpret_cast<uint8_t*>(&value);
//   uint8_t inBytes[sizeof(uint32_t)];
//
//   SPI.transferBytes(outBytes, inBytes, sizeof(uint32_t));
//
//   uint32_t reply = *reinterpret_cast<uint32_t*>(&inBytes);
//
//   #if SRAM_DEBUG
//   Serial.print("SRAM Transfer (uint32_t (");
//   Serial.print(sizeof(uint32_t));
//   Serial.print(")): ");
//   Serial.print(value);
//   Serial.print(" -> ");
//   Serial.println(reply);
//   #endif
//
//   return reply;
// }
//
// double sram_transfer(double value = 0) {
//   uint8_t *outBytes = reinterpret_cast<uint8_t*>(&value);
//   uint8_t inBytes[sizeof(double)];
//
//   // Serial.print("Out: ");
//   // for (int i=0; i<sizeof(double);i++){
//   //   for (int j=0;j<8;j++) {
//   //     Serial.print(((outBytes[i]>>j)&1)?"1":"0");
//   //   }
//   // }
//   // Serial.println();
//
//   SPI.transferBytes(outBytes, inBytes, sizeof(double));
//
//   // Serial.print("In: ");
//   // for (int i=0; i<sizeof(double);i++){
//   //   for (int j=0;j<8;j++) {
//   //     Serial.print(((inBytes[i]>>j)&1)?"1":"0");
//   //   }
//   // }
//   // Serial.println();
//
//   double reply = *reinterpret_cast<double*>(&inBytes);
//
//   #if SRAM_DEBUG
//   Serial.print("SRAM Transfer (double (");
//   Serial.print(sizeof(double));
//   Serial.print(")): ");
//   Serial.print(value);
//   Serial.print(" -> ");
//   Serial.println(reply);
//   #endif
//
//   return reply;
// }
//
// float sram_transfer(float value = 0) {
//   uint8_t *outBytes = reinterpret_cast<uint8_t*>(&value);
//   uint8_t inBytes[sizeof(float)];
//
//   SPI.transferBytes(outBytes, inBytes, sizeof(float));
//
//   float reply = *reinterpret_cast<float*>(&inBytes);
//
//   #if SRAM_DEBUG
//   Serial.print("SRAM Transfer (float (");
//   Serial.print(sizeof(float));
//   Serial.print(")): ");
//   Serial.print(value);
//   Serial.print(" -> ");
//   Serial.println(reply);
//   #endif
//
//   return reply;
// }


// uint32_t sram_transfer32(uint32_t data) {
//   union {
//         uint32_t val;
//         struct {
//                 uint8_t lsb;
//                 uint8_t mlsb;
//                 uint8_t mmsb;
//                 uint8_t msb;
//         };
//   } in, out;
//   in.val = data;
//
//   if((SPI1C & (SPICWBO | SPICRBO))) {
//       //LSBFIRST
//       out.lsb = transfer(in.lsb);
//       out.mlsb = transfer(in.mlsb);
//       out.mmsb = transfer(in.mmsb);
//       out.msb = transfer(in.msb);
//   } else {
//       //MSBFIRST
//       out.msb = transfer(in.msb);
//       out.mmsb = transfer(in.mmsb);
//       out.mlsb = transfer(in.mlsb);
//       out.lsb = transfer(in.lsb);
//   }
//
//   return out.val;
// }

// uint8_t sram_transfer8(uint8_t data) {
//   return SPI.transfer(data);
// }
//
// uint16_t sram_transfer16(uint16_t data) {
//   return SPI.transfer16(data);
// }
//
// uint32_t sram_transfer32(uint32_t data) {
//   union {
//         uint32_t val;
//         struct {
//                 uint16_t lsb;
//                 uint16_t msb;
//         };
//   } in, out;
//   in.val = data;
//
//   if((SPI1C & (SPICWBO | SPICRBO))) {
//       //LSBFIRST
//       out.lsb = sram_transfer16(in.lsb);
//       out.msb = sram_transfer16(in.msb);
//   } else {
//       //MSBFIRST
//       out.msb = sram_transfer16(in.msb);
//       out.lsb = sram_transfer16(in.lsb);
//   }
//
//   return out.val;
// }
//
// uint64_t sram_transfer64(uint64_t data) {
//   union {
//         uint64_t val;
//         struct {
//                 uint32_t lsb;
//                 uint32_t msb;
//         };
//   } in, out;
//   in.val = data;
//
//   if((SPI1C & (SPICWBO | SPICRBO))) {
//       //LSBFIRST
//       out.lsb = sram_transfer32(in.lsb);
//       out.msb = sram_transfer32(in.msb);
//   } else {
//       //MSBFIRST
//       out.msb = sram_transfer32(in.msb);
//       out.lsb = sram_transfer32(in.lsb);
//   }
//
//   return out.val;
// }

// void print_bin(uint8_t *bytes, int size, bool newLine=true) {
//   // Bytes in the union have the lsb as 0
//
//   if (newLine) {
//     Serial.print("Bytes: ");
//   }
//
//   // For i from size to 0 so we print msb first
//   for (int i=size-1; i>=0;i--){
//     // First bit is lsb and we want to print msb first
//     for (int j=7;j>=0;j--) {
//       Serial.print(((bytes[i]>>j)&1)?"1":"0");
//     }
//     Serial.print(" ");
//   }
//   if (newLine) {
//     Serial.println();
//   }
// }
//
// void print_bin_32(uint32_t value, bool newLine=true) {
//   // Bytes in the union have the lsb as 0
//
//   if (newLine) {
//     Serial.print("Bytes: ");
//   }
//
//   // For i from size to 0 so we print msb first
//   for (int i=0; i<32; i++) {
//     if ((i%8) == 0) {
//       Serial.print(" ");
//     }
//
//     Serial.print(((value>>(31-i))&1)?"1":"0");
//   }
//
//   if (newLine) {
//     Serial.println();
//   }
// }
//
// void print_bin_64(uint64_t value, bool newLine=true) {
//   // Bytes in the union have the lsb as 0
//
//   if (newLine) {
//     Serial.print("Bytes: ");
//   }
//
//   // For i from size to 0 so we print msb first
//   for (int i=0; i<64; i++) {
//     if ((i%8) == 0) {
//       Serial.print(" ");
//     }
//
//     Serial.print(((value>>(63-i))&1)?"1":"0");
//   }
//
//   if (newLine) {
//     Serial.println();
//   }
// }
//
// uint8_t sram_transfer_8(uint8_t data) {
//   uint8_t reply = SPI.transfer(data);
//
//   // #if SRAM_DEBUG
//   // Serial.print("SRAM Transfer (uint8_t (");
//   // Serial.print(sizeof(data));
//   // Serial.print(")): ");
//   // Serial.println(data);
//   // Serial.print("Write (");
//   // Serial.print(data);
//   // Serial.print(") ");
//   // print_bin(&data, sizeof(data));
//   // Serial.print("Read (");
//   // Serial.print(reply);
//   // Serial.print(") ");
//   // print_bin(&reply, sizeof(reply));
//   // #endif
//
//   return reply;
// }
// //
// // uint32_t sram_transfer_32(uint32_t data) {
// //   union {
// //     uint32_t val;
// //     unsigned char bytes[sizeof(data)];
// //   } in, out;
// //
// //   in.val = data;
// //
// //   #if SRAM_DEBUG
// //   Serial.print("SRAM Transfer (uint32_t (");
// //   Serial.print(sizeof(data));
// //   Serial.print(")): ");
// //   Serial.println(data);
// //   Serial.print("Write (");
// //   Serial.print(in.val);
// //   Serial.print(") ");
// //   print_bin(in.bytes, sizeof(data));
// //   #endif
// //
// //   SPI.transferBytes(out.bytes, in.bytes, sizeof(data));
// //
// //   #if SRAM_DEBUG
// //   Serial.print("Read (");
// //   Serial.print(out.val);
// //   Serial.print(") ");
// //   print_bin(out.bytes, sizeof(data));
// //   #endif
// //
// //   return out.val;
// // }
//
// // float sram_transfer_float(float data) {
// //   union {
// //     float val;
// //
// //     struct {
// //       uint8_t lsb0;
// //       uint8_t lsb1;
// //       uint8_t lsb2;
// //       uint8_t lsb3;
// //     };
// //   } in, out;
// //
// //   in.val = data;
// //
// //   #if SRAM_DEBUG
// //   Serial.print("SRAM Transfer (float (");
// //   Serial.print(sizeof(data));
// //   Serial.print(")): ");
// //   Serial.println(data);
// //   Serial.print("Write (");
// //   Serial.print(in.val);
// //   Serial.print(") ");
// //   print_bin(&in.lsb0, 1, false);
// //   print_bin(&in.lsb1, 1, false);
// //   print_bin(&in.lsb2, 1, false);
// //   print_bin(&in.lsb3, 1, false);
// //   Serial.println();
// //   #endif
// //
// //   //SPI.transferBytes(out.bytes, in.bytes, sizeof(data));
// //   if((SPI1C & (SPICWBO | SPICRBO))) {
// //     //LSBFIRST
// //     out.lsb0 = SPI.transfer(in.lsb0);
// //     out.lsb1 = SPI.transfer(in.lsb1);
// //     out.lsb2 = SPI.transfer(in.lsb2);
// //     out.lsb3 = SPI.transfer(in.lsb3);
// //   } else {
// //     //MSBFIRST
// //     out.lsb3 = SPI.transfer(in.lsb3);
// //     out.lsb2 = SPI.transfer(in.lsb2);
// //     out.lsb1 = SPI.transfer(in.lsb1);
// //     out.lsb0 = SPI.transfer(in.lsb0);
// //   }
// //
// //   #if SRAM_DEBUG
// //   Serial.print("Read (");
// //   Serial.print(out.val);
// //   Serial.print(") ");
// //   print_bin(&out.lsb0, 1, false);
// //   print_bin(&out.lsb1, 1, false);
// //   print_bin(&out.lsb2, 1, false);
// //   print_bin(&out.lsb3, 1, false);
// //   Serial.println();
// //   #endif
// //
// //   return out.val;
// // }
//
// uint32_t sram_transfer_32(uint32_t data) {
//   union {
//     uint32_t val;
//
//     struct {
//       uint8_t lsb0;
//       uint8_t lsb1;
//       uint8_t lsb2;
//       uint8_t lsb3;
//     };
//   } in, out;
//
//   in.val = data;
//
//   // #if SRAM_DEBUG
//   // Serial.print("SRAM Transfer (uint32_t (");
//   // Serial.print(sizeof(data));
//   // Serial.println(")) ");
//   // //Serial.println(data);
//   // // Serial.print("Write (");
//   // // Serial.print(in.val);
//   // // Serial.print(") ");
//   // // print_bin(&in.lsb3, 1, false);
//   // // print_bin(&in.lsb2, 1, false);
//   // // print_bin(&in.lsb1, 1, false);
//   // // print_bin(&in.lsb0, 1, false);
//   // // Serial.println();
//   // #endif
//
//   //SPI.transferBytes(out.bytes, in.bytes, sizeof(data));
//   // if((SPI1C & (SPICWBO | SPICRBO))) {
//   //   //LSBFIRST
//   //   out.lsb0 = SPI.transfer(in.lsb0);
//   //   out.lsb1 = SPI.transfer(in.lsb1);
//   //   out.lsb2 = SPI.transfer(in.lsb2);
//   //   out.lsb3 = SPI.transfer(in.lsb3);
//   // } else {
//   if (1) {
//     //MSBFIRST
//     out.lsb3 = SPI.transfer(in.lsb3);
//     out.lsb2 = SPI.transfer(in.lsb2);
//     out.lsb1 = SPI.transfer(in.lsb1);
//     out.lsb0 = SPI.transfer(in.lsb0);
//   }
//
//   // #if SRAM_DEBUG
//   // Serial.print("Read (");
//   // Serial.print(out.val);
//   // Serial.print(") ");
//   // print_bin(&out.lsb3, 1, false);
//   // print_bin(&out.lsb2, 1, false);
//   // print_bin(&out.lsb1, 1, false);
//   // print_bin(&out.lsb0, 1, false);
//   // Serial.println();
//   // #endif
//
//   return out.val;
// }
//
// uint64_t sram_transfer_64(uint64_t data) {
//   union {
//     uint64_t val;
//
//     struct {
//       uint8_t lsb0;
//       uint8_t lsb1;
//       uint8_t lsb2;
//       uint8_t lsb3;
//       uint8_t lsb4;
//       uint8_t lsb5;
//       uint8_t lsb6;
//       uint8_t lsb7;
//     };
//   } in, out;
//
//   in.val = data;
//
//   // #if SRAM_DEBUG
//   // Serial.print("SRAM Transfer (uint64_t (");
//   // Serial.print(sizeof(data));
//   // Serial.println(")) ");
//   // //
//   // // Serial.print("Write: ");
//   // // print_bin(&out.lsb7, 1, false);
//   // // print_bin(&out.lsb6, 1, false);
//   // // print_bin(&out.lsb5, 1, false);
//   // // print_bin(&out.lsb4, 1, false);
//   // // print_bin(&out.lsb3, 1, false);
//   // // print_bin(&out.lsb2, 1, false);
//   // // print_bin(&out.lsb1, 1, false);
//   // // print_bin(&out.lsb0, 1, false);
//   //
//   // //Serial.println();
//   // #endif
//
//   // //SPI.transferBytes(out.bytes, in.bytes, sizeof(data));
//   // if((SPI1C & (SPICWBO | SPICRBO))) {
//   //   //LSBFIRST
//   //   out.lsb0 = SPI.transfer(in.lsb0);
//   //   out.lsb1 = SPI.transfer(in.lsb1);
//   //   out.lsb2 = SPI.transfer(in.lsb2);
//   //   out.lsb3 = SPI.transfer(in.lsb3);
//   //   out.lsb4 = SPI.transfer(in.lsb4);
//   //   out.lsb5 = SPI.transfer(in.lsb5);
//   //   out.lsb6 = SPI.transfer(in.lsb6);
//   //   out.lsb7 = SPI.transfer(in.lsb7);
//   //} else {
//   if (1) {
//     //MSBFIRST
//     out.lsb7 = SPI.transfer(in.lsb7);
//     out.lsb6 = SPI.transfer(in.lsb6);
//     out.lsb5 = SPI.transfer(in.lsb5);
//     out.lsb4 = SPI.transfer(in.lsb4);
//     out.lsb3 = SPI.transfer(in.lsb3);
//     out.lsb2 = SPI.transfer(in.lsb2);
//     out.lsb1 = SPI.transfer(in.lsb1);
//     out.lsb0 = SPI.transfer(in.lsb0);
//   }
//
//   // #if SRAM_DEBUG
//   // Serial.print("Read: ");
//   // print_bin(&out.lsb7, 1, false);
//   // print_bin(&out.lsb6, 1, false);
//   // print_bin(&out.lsb5, 1, false);
//   // print_bin(&out.lsb4, 1, false);
//   // print_bin(&out.lsb3, 1, false);
//   // print_bin(&out.lsb2, 1, false);
//   // print_bin(&out.lsb1, 1, false);
//   // print_bin(&out.lsb0, 1, false);
//   // Serial.println();
//   // #endif
//
//   return out.val;
// }
//
// double sram_transfer_double(double data) {
//   union {
//     double typeVal;
//     uint64_t intVal;
//   } in, out;
//
//   in.typeVal = data;
//
//   out.intVal = sram_transfer_64(in.intVal);
//
//   // Serial.print("Write (double)(");
//   // Serial.print(in.typeVal);
//   // Serial.print("): ");
//   // print_bin_64(in.intVal, false);
//   // Serial.println();
//   //
//   // Serial.print("Read (double)(");
//   // Serial.print(out.typeVal);
//   // Serial.print("): ");
//   // print_bin_64(out.intVal, false);
//   // Serial.println();
//
//   return out.typeVal;
// }
//
// float sram_transfer_float(float data) {
//   union {
//     float typeVal;
//     uint32_t intVal;
//   } in, out;
//
//   in.typeVal = data;
//
//   out.intVal = sram_transfer_32(in.intVal);
//
//   // Serial.print("Write (float)(");
//   // Serial.print(in.typeVal);
//   // Serial.print("): ");
//   // print_bin_32(in.intVal, false);
//   // Serial.println();
//   //
//   // Serial.print("Read (float)(");
//   // Serial.print(out.typeVal);
//   // Serial.print("): ");
//   // print_bin_32(out.intVal, false);
//   // Serial.println();
//
//   return out.typeVal;
// }
//
// void sram_read(uint32_t *dest, int address, int size) {
//   sram_start(SRAM_MODE_READ, address, size);
//   *dest = sram_transfer_32(0);
//   sram_end();
// }
//
// void sram_write(uint32_t value, int address, int size) {
//   sram_start(SRAM_MODE_WRITE, address, size);
//   sram_transfer_32(value);
//   sram_end();
// }
//
//
// void sram_write_reading(WeatherReading reading, int readingIndex) {
//   int addrReading = SRAM_ADDR_READINGS + (SRAM_SIZE_READINGS * readingIndex);
//   Serial.print("Writing WeatherReading to Sram Addr: ");
//   Serial.println(addrReading);
//
//   if (!sram_start(SRAM_MODE_WRITE, addrReading, SRAM_SIZE_READINGS)) {
//     return;
//   }
//
//   sram_transfer_32(reading.timestamp);
//   sram_transfer_float(reading.temperature);
//   sram_transfer_float(reading.humidity);
//   sram_transfer_float(reading.pressure);
//   sram_transfer_float(reading.battery);
//   sram_transfer_float(reading.windSpeed);
//   sram_transfer_float(reading.windDirection);
//
//   sram_end();
// }
//
// void sram_read_reading(WeatherReading *reading, int readingIndex) {
//   int addrReading = SRAM_ADDR_READINGS + (SRAM_SIZE_READINGS * readingIndex);
//   Serial.print("Reading WeatherReading from Sram Addr: ");
//   Serial.println(addrReading);
//
//   if (!sram_start(SRAM_MODE_READ, addrReading, SRAM_SIZE_READINGS)) {
//     return;
//   }
//
//   reading->timestamp = sram_transfer_32(0);
//   reading->temperature = sram_transfer_float(0);
//   reading->humidity = sram_transfer_float(0);
//   reading->pressure = sram_transfer_float(0);
//   reading->battery = sram_transfer_float(0);
//   reading->windSpeed = sram_transfer_float(0);
//   reading->windDirection = sram_transfer_float(0);
//
//   sram_end();
// }
//
//
// void sram_write_accumulator(WeatherReadingAccumulator accumulator) {
//   sram_start(SRAM_MODE_WRITE, SRAM_ADDR_ACCUMULATOR, SRAM_SIZE_ACCUMULATOR);
//
//   sram_transfer_32(accumulator.timestamp);
//   sram_transfer_double(accumulator.temperature);
//   sram_transfer_double(accumulator.humidity);
//   sram_transfer_double(accumulator.pressure);
//   sram_transfer_double(accumulator.battery);
//   sram_transfer_double(accumulator.windSpeed);
//   sram_transfer_double(accumulator.windDirection);
//   sram_transfer_32(accumulator.numSamples);
//   sram_transfer_32(accumulator.numBatterySamples);
//
//   sram_end();
// }
//
// void sram_read_accumulator(WeatherReadingAccumulator *accumulator) {
//   sram_start(SRAM_MODE_READ, SRAM_ADDR_ACCUMULATOR, SRAM_SIZE_ACCUMULATOR);
//
//   accumulator->timestamp = sram_transfer_32(0);
//   accumulator->temperature = sram_transfer_double(0);
//   accumulator->humidity = sram_transfer_double(0);
//   accumulator->pressure = sram_transfer_double(0);
//   accumulator->battery = sram_transfer_double(0);
//   accumulator->windSpeed = sram_transfer_double(0);
//   accumulator->windDirection = sram_transfer_double(0);
//   accumulator->numSamples = sram_transfer_32(0);
//   accumulator->numBatterySamples = sram_transfer_32(0);
//
//   sram_end();
// }
//
// bool sram_get_populated() {
//   sram_start(SRAM_MODE_READ, SRAM_ADDR_POPULATED, SRAM_SIZE_POPULATED);
//   bool result = (sram_transfer_8(0) == 0b10101010);
//   sram_end();
//
//   return result;
// }
//
// void sram_set_populated(bool populated = true) {
//   sram_start(SRAM_MODE_WRITE, SRAM_ADDR_POPULATED, SRAM_SIZE_POPULATED);
//   if (populated) {
//     sram_transfer_8(0b10101010);
//   } else {
//     sram_transfer_8(0);
//   }
//   sram_end();
// }
