//#include <Arduino.h>
#include "LEDblink.hpp"

#if defined(ESP32)
// for ESP32 devkit v4
#define LED_BUILTIN 2
#endif

LEDblink led_test;

#include <Wire.h>

// Arduino UNO
// Connect INA238 with SCL (A5), SDA (A4)
// Teensy 4.1
// Connect INA238 with SCL0(19), SDA0(18) : Wire
// Connect INA238 with SCL1(16), SDA1(17) : Wire1
// Connect INA238 with SCL2(24), SDA2(15) : Wire2
// https://github.com/Richard-Gemmell/teensy4_i2c

const byte INA238_ADDR = B1000000;
// const byte INA238_ADDR = B1000101;

const byte INA238_CONFIG          = 0x00;
const byte INA238_ADC_CONFIG      = 0x01;
const byte INA238_SHUNT_CAL       = 0x02;
const byte INA238_VSHUNT          = 0x04;
const byte INA238_VBUS            = 0x05;
const byte INA238_DIETEMP         = 0x06;
const byte INA238_CURRENT         = 0x07;
const byte INA238_POWER           = 0x08;
const byte INA238_DIAG_ALRT       = 0x0B;
const byte INA238_SOVL            = 0x0C;
const byte INA238_SUVL            = 0x0D;
const byte INA238_BOVL            = 0x0E;
const byte INA238_BUVL            = 0x0F;
const byte INA238_TEMP_LIMIT      = 0x10;
const byte INA238_PWR_LIMIT       = 0x11;
const byte INA238_MANUFACTURER_ID = 0x3E;
const byte INA238_DEVICE_ID       = 0x3F;

void INA238_write(byte reg, unsigned short val)
{
Wire1.beginTransmission(INA238_ADDR);
Wire1.write(reg);
Wire1.write(val >> 8);
Wire1.write(val & 0xff);
Wire1.endTransmission();
}

short INA238_read(byte reg)
{
short ret = 0;
// request the registor
Wire1.beginTransmission(INA238_ADDR);
Wire1.write(reg);
Wire1.endTransmission();

// read
Wire1.requestFrom((int)INA238_ADDR, 2);

while(Wire1.available()) {
ret = (ret << 8) | Wire1.read();
}

return ret;
}

void setup() {
    led_test.init(LED_BUILTIN);
    Serial.begin(115200);
    while (!Serial) {}

    //Wire.begin();
    //Wire.setClock(100000); // fSCL = 100kHz
    Wire1.begin();
    Wire1.setClock(100000); // fSCL = 100kHz
    //Wire.setClock(20000); // fSCL = 20kHz
    //Wire.setClock(400000); // fSCL = 400kHz
    // INA226, Configuration 00h, 0x45ff
    // 0100 0101 1111 1111
    // 0: RST
    // 100: None
    // 010: AVG, AVeraGe, 16 times
    // 111: VBusCT, 8.244 ms
    // 111: VShCT, 8.244 ms
    // 111: MODE, Shunt and Bus, Continuous
    // INA238, Configuration 00h, 0x0080
    // 0000 0000 0100 0000
    // 0: RST
    // 0: RESERVED 
    // 0000 0001: ADC conversion, 0h = 0 s, 1h = 2 ms, FFh = 510 ms
    // 0: RESERVED
    // 0: ADCRANGE, Shunt full scale range 0h = +-163.84mV, 1h = +-40.96mV
    // 0000: RESERVED
    INA238_write(INA238_CONFIG, 0x0080);
    // INA226, Calibration 05h, 2560, 0A00h, current conversion
    // 0000 0110 0000 0000
    // (1) CALibration = 0.00512/(Current_LSB*Rshunt)
    // (2) Current_LSB = Maximum Expected Current/2^15
    // Example1, Rshunt = 2 mOhm, Current_LSB = 1 mA/bit
    // CALibration = 0.00512/1 mA/bit * 2 mOhm = 2560
    // Example2, Maximum Expected Current = 15 A
    // Current_LSB = 15 A/ 2^15 = 457.7 uA/bit
    // INA238, ADC_Config 01h, 0xBB6A
    // 1011 1011 0110 1010
    // 1011: MODE, Bh = Continuous shunt and bus voltage,
    //             9h = Continuous bus voltage only, 
    //             Fh = Continuous bus voltage, shunt voltage and termperature, 
    //             Dh = Continuous bus voltage and temperature 
    // 101: VBUSCT, 5h = 1052 us
    // 101: VSHCT, 5h = 1052 us
    // 101: VTCT, 5h = 1052 us
    // 010: AVG, 2h = 16 times
    //INA238_write(INA238_ADC_CONFIG, 0xBB6A);
    INA238_write(INA238_ADC_CONFIG, 0xFB6A);
    //INA238_write(INA238_ADC_CONFIG, 0xDB6A);
    // INA238, SHUNT_CAL 02h, 0x1000
    // 0001 0000 0000 0000
    // 0: RESERVED
    // 001 0000 0000 0000: 
    INA238_write(INA238_SHUNT_CAL, 0x1000);
}
 
void loop() {
    int vs;     // Shunt Voltage, 5 uV/LSB when ADCRANGE = 0, +-164.84 mV
    int vb;     // Bus Voltage, 3.125 mV/LSB, 0~85V
    int dt;     // Temperature, 125 m Celcius Degree/LSB, -40~125 Celsius Degree
    int cs;     // Shunt Current, 2.5 mA/LSB = 163.84 mV / 2^15 / 0.002 Ohm when R_shunt = 2 mOhm
    float vs_;
    float vb_;
    float dt_;
    float cs_;

    vs_ = vs = INA238_read(INA238_VSHUNT);
    vb_ = vb = INA238_read(INA238_VBUS);
    dt_ = dt = (INA238_read(INA238_DIETEMP)) >> 4;
    cs_ = cs = INA238_read(INA238_CURRENT);

    vs_ *= 0.005;
    vb_ *= 3.125;
    dt_ *= 0.125;
    cs_ *= 2.5;

    Serial.print(vb); // bus voltage (reading)
    Serial.print(" ");
    Serial.print(vs); // shunt voltage (reading)
    Serial.print(" ");
    Serial.print(vb_); // bus voltage in [mV]
    Serial.print(" ");
    Serial.print(cs); // current in [mA]
    Serial.print(" ");
    Serial.print(dt); // dietemperature [125mdegree/LSB]
    Serial.print(" ");
    Serial.println(dt_); // temperature [Degree]

    led_test.led_on();
    Serial.println("LED On!!"); 
    delay(1000); 
    led_test.led_off();
    Serial.println("LED Off!!");
    delay(1000); 
}