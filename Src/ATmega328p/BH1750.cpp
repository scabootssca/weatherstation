/*

This is a library for the BH1750FVI Digital Light Sensor
breakout board.

The board uses I2C for communication. 2 pins are required to
interface to the device.


Written by Christopher Laws, March, 2013.

*/

#include "BH1750.h"
#include <util/delay.h>


BH1750::BH1750() {
}

void BH1750::begin(uint8_t mode) {

  //I2C.begin();
  //write8(mode);
  configure(mode);

  //if (read8(BME280_REGISTER_CHIPID) != 0x60)

}


void BH1750::configure(uint8_t mode) {

    switch (mode) {
        case BH1750_CONTINUOUS_HIGH_RES_MODE:
        case BH1750_CONTINUOUS_HIGH_RES_MODE_2:
        case BH1750_CONTINUOUS_LOW_RES_MODE:
        case BH1750_ONE_TIME_HIGH_RES_MODE:
        case BH1750_ONE_TIME_HIGH_RES_MODE_2:
        case BH1750_ONE_TIME_LOW_RES_MODE:
            // apply a valid mode change
            write8(mode);
            _delay_ms(10);
            break;
        default:
            // Invalid measurement mode
            #if BH1750_DEBUG == 1
            Serial.println("Invalid measurement mode");
            #endif
            break;
    }
}


uint16_t BH1750::readLightLevel(void) {

  uint16_t level;

  uint8_t returnCode = I2c.read(BH1750_I2CADDR, 2);

  delay(120); // 120ms delay for lux sensor at High res mode

  level = I2c.receive();
  level <<= 8;
  level |= I2c.receive();

  if (returnCode != 0) {
    I2c.countError(I2C_READ, BH1750_I2CADDR, 0, returnCode, 0);
  }

// #if (ARDUINO >= 100)
//   level = Wire.read();
//   level <<= 8;
//   level |= Wire.read();
// #else
//   level = Wire.receive();
//   level <<= 8;
//   level |= Wire.receive();
// #endif
//   Wire.endTransmission();

#if BH1750_DEBUG == 1
  Serial.print("Raw light level: ");
  Serial.println(level);
#endif

  level = level/1.2; // convert to lux

#if BH1750_DEBUG == 1
  Serial.print("Light level: ");
  Serial.println(level);
#endif
  return level;
}



/*********************************************************************/


void BH1750::write8(uint8_t d) {
  I2c.write(uint8_t(BH1750_I2CADDR), d);
//   Wire.beginTransmission(BH1750_I2CADDR);
// #if (ARDUINO >= 100)
//   Wire.write(d);
// #else
//   Wire.send(d);
// #endif
//   Wire.endTransmission();
}
