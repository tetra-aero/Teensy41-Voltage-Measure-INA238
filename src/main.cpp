//#include <Arduino.h>
#include "LEDblink.hpp"

#if defined(ESP32)
// for ESP32 devkit v4
#define LED_BUILTIN 2
#endif

LEDblink led_test;

#include <Wire.h>

// Arduino UNO
// Connect INA226 with SCL (A5), SDA (A4)
// Teensy 4.1
// Connect INA226 with SCL0(19), SDA0(18) : Wire
// Connect INA226 with SCL1(16), SDA1(17) : Wire1
// Connect INA226 with SCL2(24), SDA2(15) : Wire2
// https://github.com/Richard-Gemmell/teensy4_i2c

// const byte INA226_ADDR = B1000000;
const byte INA226_ADDR = B1000101;

const byte INA226_CONFIG    = 0x00;
const byte INA226_SHUNTV    = 0x01;
const byte INA226_BUSV      = 0x02;
const byte INA226_POWER     = 0x03;
const byte INA226_CURRENT   = 0x04;
const byte INA226_CALIB     = 0x05;
const byte INA226_MASK      = 0x06;
const byte INA226_ALERTL    = 0x07;
const byte INA226_DIE_ID    = 0xff;

void INA226_write(byte reg, unsigned short val)
{
Wire.beginTransmission(INA226_ADDR);
Wire.write(reg);
Wire.write(val >> 8);
Wire.write(val & 0xff);
Wire.endTransmission();
}

short INA226_read(byte reg)
{
short ret = 0;
// request the registor
Wire.beginTransmission(INA226_ADDR);
Wire.write(reg);
Wire.endTransmission();

// read
Wire.requestFrom((int)INA226_ADDR, 2);

while(Wire.available()) {
ret = (ret << 8) | Wire.read();
}

return ret;
}

void setup() {
    led_test.init(LED_BUILTIN);
    Serial.begin(115200);
    while (!Serial) {}

    Wire.begin();
    // average: 16 times, conversion time: 8.244ms/8.244ms
    INA226_write(INA226_CONFIG, 0x45ff);
    // current conversion
    INA226_write(INA226_CALIB, 2560);
}
 
void loop() {
    int sv, bv, c;
    float bv_;

    bv_ = bv = INA226_read(INA226_BUSV);
    sv = INA226_read(INA226_SHUNTV);
    c = INA226_read(INA226_CURRENT);

    bv_ *= 1.25;
    //bv_ *= 1.18;

    Serial.print(bv); // bus voltage (reading)
    Serial.print(" ");
    Serial.print(sv); // shunt voltage (reading)
    Serial.print(" ");
    Serial.print(bv_); // bus voltage in [mV]
    Serial.print(" ");
    Serial.println(c); // current in [mA]

    led_test.led_on();
    Serial.println("LED On!!"); 
    delay(500); 
    led_test.led_off();
    Serial.println("LED Off!!");
    delay(500); 
}