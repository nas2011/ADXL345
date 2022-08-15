+/*
 The MIT License (MIT)

 Copyright (c) 2022 Nick Sexson nick@nicksexson.com

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
*/


/*
Minimal driver for ADXL345 accelerometer. Based on the circuitpython driver by
Bryan Siepert : https://github.com/adafruit/Adafruit_CircuitPython_ADXL34x

*/

import gpio
import i2c
import binary
import serial
import fixed_point


_ADXL345_MG2G_MULTIPLIER ::= 0.004  // 4mg per lsb
_STANDARD_GRAVITY ::= 9.80665  // earth standard gravity

_REG_DEVID ::= 0x00  // Device ID
_REG_THRESH_TAP ::= 0x1D  // Tap threshold
_REG_OFSX ::= 0x1E  // X-axis offset
_REG_OFSY ::= 0x1F  // Y-axis offset
_REG_OFSZ ::= 0x20  // Z-axis offset
_REG_DUR ::= 0x21  // Tap duration
_REG_LATENT ::= 0x22  // Tap latency
_REG_WINDOW ::= 0x23  // Tap window
_REG_THRESH_ACT ::= 0x24  // Activity threshold
_REG_THRESH_INACT ::= 0x25  // Inactivity threshold
_REG_TIME_INACT ::= 0x26  // Inactivity time
_REG_ACT_INACT_CTL ::= 0x27  // Axis enable control for [in]activity detection
_REG_THRESH_FF ::= 0x28  // Free-fall threshold
_REG_TIME_FF ::= 0x29  // Free-fall time
_REG_TAP_AXES ::= 0x2A  // Axis control for single/double tap
_REG_ACT_TAP_STATUS ::= 0x2B  // Source for single/double tap
_REG_BW_RATE ::= 0x2C  // Data rate and power mode control
_REG_POWER_CTL ::= 0x2D  // Power-saving features control
_REG_INT_ENABLE ::= 0x2E  // Interrupt enable control
_REG_INT_MAP ::= 0x2F  // Interrupt mapping control
_REG_INT_SOURCE ::= 0x30  // Source of interrupts
_REG_DATA_FORMAT ::= 0x31  // Data format control
_REG_DATAX0 ::= 0x32  // X-axis data 0
_REG_DATAX1 ::= 0x33  // X-axis data 1
_REG_DATAY0 ::= 0x34  // Y-axis data 0
_REG_DATAY1 ::= 0x35  // Y-axis data 1
_REG_DATAZ0 ::= 0x36  // Z-axis data 0
_REG_DATAZ1 ::= 0x37  // Z-axis data 1
_REG_FIFO_CTL ::= 0x38  // FIFO control
_REG_FIFO_STATUS ::= 0x39  // FIFO status
_INT_SINGLE_TAP ::= 0b01000000  // SINGLE_TAP bit
_INT_DOUBLE_TAP ::= 0b00100000  // DOUBLE_TAP bit
_INT_ACT ::= 0b00010000  // ACT bit
_INT_INACT ::= 0b00001000  // INACT bit
_INT_FREE_FALL ::= 0b00000100  // FREE_FALL  bit


// Data Rates
RATE_3200_HZ ::= 0b1111  // 1600Hz Bandwidth   140mA IDD
RATE_1600_HZ ::= 0b1110  //  800Hz Bandwidth    90mA IDD
RATE_800_HZ ::= 0b1101  //  400Hz Bandwidth   140mA IDD
RATE_400_HZ ::= 0b1100  //  200Hz Bandwidth   140mA IDD
RATE_200_HZ ::= 0b1011  //  100Hz Bandwidth   140mA IDD
RATE_100_HZ ::= 0b1010  //   50Hz Bandwidth   140mA IDD (default value
RATE_50_HZ ::= 0b1001  //   25Hz Bandwidth    90mA IDD
RATE_25_HZ ::= 0b1000  // 12.5Hz Bandwidth    60mA IDD
RATE_12_5_HZ ::= 0b0111  // 6.25Hz Bandwidth    50mA IDD
RATE_6_25HZ ::= 0b0110  // 3.13Hz Bandwidth    45mA IDD
RATE_3_13_HZ ::= 0b0101  // 1.56Hz Bandwidth    40mA IDD
RATE_1_56_HZ ::= 0b0100  // 0.78Hz Bandwidth    34mA IDD
RATE_0_78_HZ ::= 0b0011  // 0.39Hz Bandwidth    23mA IDD
RATE_0_39_HZ ::= 0b0010  // 0.20Hz Bandwidth    23mA IDD
RATE_0_20_HZ ::= 0b0001  // 0.10Hz Bandwidth    23mA IDD
RATE_0_10_HZ ::= 0b0000  // 0.05Hz Bandwidth    23mA IDD 


//Ranges
RANGE_16_G ::= 0b11  // +/- 16g
RANGE_8_G ::= 0b10  // +/- 8g
RANGE_4_G ::= 0b01  // +/- 4g
RANGE_2_G ::= 0b00  // +/- 2g (default value

//Common Register Values
POWER_CTL_NORMAL ::= 0b00001000
INT_ENABLE_NORMAL ::= 0b00000000

_ADXL345_DEFAULT_ADDRESS ::= 0x53 

// ESP32 Default  SDA and SCL Pins

sda /gpio.Pin := gpio.Pin 21
scl /gpio.Pin := gpio.Pin 23

// Default I2C bus

default_i2c /i2c.Bus := 
  i2c.Bus --sda = sda --scl= scl

class ADXL345:
  registers/serial.Registers


  constructor device/i2c.Device:
    registers = device.registers
  
  // Sets the power setup 
  //   D7-0 D6-0 D5-Link D4-AutoSleep 
  //   D3-Measure D2-Sleep (D1 D0)-Wakeup
  set_power val/int:
    registers.write_u8 _REG_POWER_CTL val

  // Reads out the current power status  
  power_status:
    return registers.read_u8 _REG_POWER_CTL

  // Sets the measurement read rate. See Data Rates in statics
  set_rate val/int:
    registers.write_u8  _REG_BW_RATE val
  

  // Reads all three axes and returns the list [x,y,z]
  read->List:
    x := (registers.read_i16_le _REG_DATAX0).to_float * _ADXL345_MG2G_MULTIPLIER * _STANDARD_GRAVITY
    y := (registers.read_i16_le _REG_DATAY0).to_float * _ADXL345_MG2G_MULTIPLIER * _STANDARD_GRAVITY
    z := (registers.read_i16_le _REG_DATAZ0).to_float * _ADXL345_MG2G_MULTIPLIER * _STANDARD_GRAVITY
    return [x,y,z]





/* Example

main:

  device := default_i2c.device _ADXL345_DEFAULT_ADDRESS

  sensor := ADXL345 device

  sensor.set_power POWER_CTL_NORMAL
  
  reading := sensor.read

*/
