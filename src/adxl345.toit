/*
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


ADXL345_MG2G_MULTIPLIER_ ::= 0.004  // 4mg per lsb
STANDARD_GRAVITY_ ::= 9.80665  // earth standard gravity

REG_DEVID_ ::= 0x00  // Device ID
REG_THRESH_TAP_ ::= 0x1D  // Tap threshold
REG_OFSX_ ::= 0x1E  // X-axis offset
REG_OFSY_ ::= 0x1F  // Y-axis offset
REG_OFSZ_ ::= 0x20  // Z-axis offset
REG_DUR_ ::= 0x21  // Tap duration
REG_LATENT_ ::= 0x22  // Tap latency
REG_WINDOW_ ::= 0x23  // Tap window
REG_THRESH_ACT_ ::= 0x24  // Activity threshold
REG_THRESH_INACT_ ::= 0x25  // Inactivity threshold
REG_TIME_INACT_ ::= 0x26  // Inactivity time
REG_ACT_INACT_CTL_ ::= 0x27  // Axis enable control for [in]activity detection
REG_THRESH_FF_ ::= 0x28  // Free-fall threshold
REG_TIME_FF_ ::= 0x29  // Free-fall time
REG_TAP_AXES_ ::= 0x2A  // Axis control for single/double tap
REG_ACT_TAP_STATUS_ ::= 0x2B  // Source for single/double tap
REG_BW_RATE_ ::= 0x2C  // Data rate and power mode control
REG_POWER_CTL_ ::= 0x2D  // Power-saving features control
REG_INT_ENABLE_ ::= 0x2E  // Interrupt enable control
REG_INT_MAP_ ::= 0x2F  // Interrupt mapping control
REG_INT_SOURCE_ ::= 0x30  // Source of interrupts
REG_DATA_FORMAT_ ::= 0x31  // Data format control
REG_DATAX0_ ::= 0x32  // X-axis data 0
REG_DATAX1_ ::= 0x33  // X-axis data 1
REG_DATAY0_ ::= 0x34  // Y-axis data 0
REG_DATAY1_ ::= 0x35  // Y-axis data 1
REG_DATAZ0_ ::= 0x36  // Z-axis data 0
REG_DATAZ1_ ::= 0x37  // Z-axis data 1
REG_FIFO_CTL_ ::= 0x38  // FIFO control
REG_FIFO_STATUS_ ::= 0x39  // FIFO status
INT_SINGLE_TAP_ ::= 0b01000000  // SINGLE_TAP bit
INT_DOUBLE_TAP_ ::= 0b00100000  // DOUBLE_TAP bit
INT_ACT_ ::= 0b00010000  // ACT bit
INT_INACT_ ::= 0b00001000  // INACT bit
INT_FREE_FALL_ ::= 0b00000100  // FREE_FALL  bit


// Data Rates
RATE_3200_HZ_ ::= 0b1111  // 1600Hz Bandwidth   140mA IDD
RATE_1600_HZ_ ::= 0b1110  //  800Hz Bandwidth    90mA IDD
RATE_800_HZ_ ::= 0b1101  //  400Hz Bandwidth   140mA IDD
RATE_400_HZ_ ::= 0b1100  //  200Hz Bandwidth   140mA IDD
RATE_200_HZ_ ::= 0b1011  //  100Hz Bandwidth   140mA IDD
RATE_100_HZ_ ::= 0b1010  //   50Hz Bandwidth   140mA IDD (default value
RATE_50_HZ_ ::= 0b1001  //   25Hz Bandwidth    90mA IDD
RATE_25_HZ_ ::= 0b1000  // 12.5Hz Bandwidth    60mA IDD
RATE_12_5_HZ_ ::= 0b0111  // 6.25Hz Bandwidth    50mA IDD
RATE_6_25HZ_ ::= 0b0110  // 3.13Hz Bandwidth    45mA IDD
RATE_3_13_HZ_ ::= 0b0101  // 1.56Hz Bandwidth    40mA IDD
RATE_1_56_HZ_ ::= 0b0100  // 0.78Hz Bandwidth    34mA IDD
RATE_0_78_HZ_ ::= 0b0011  // 0.39Hz Bandwidth    23mA IDD
RATE_0_39_HZ_ ::= 0b0010  // 0.20Hz Bandwidth    23mA IDD
RATE_0_20_HZ_ ::= 0b0001  // 0.10Hz Bandwidth    23mA IDD
RATE_0_10_HZ_ ::= 0b0000  // 0.05Hz Bandwidth    23mA IDD 


//Ranges
RANGE_16_G_ ::= 0b11  // +/- 16g
RANGE_8_G_ ::= 0b10  // +/- 8g
RANGE_4_G_ ::= 0b01  // +/- 4g
RANGE_2_G_ ::= 0b00  // +/- 2g (default value

//Common Register Values
POWER_CTL_NORMAL_ ::= 0b00001000
INT_ENABLE_NORMAL_ ::= 0b00000000

ADXL345_DEFAULT_ADDRESS_ ::= 0x53 

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
    registers.write_u8 REG_POWER_CTL_ val

  // Reads out the current power status  
  power_status:
    return registers.read_u8 REG_POWER_CTL_

  // Sets the measurement read rate. See Data Rates in statics
  set_rate val/int:
    registers.write_u8  REG_BW_RATE_ val
  

  // Reads all three axes and returns the list [x,y,z]
  read->List:
    x := (registers.read_i16_le REG_DATAX0_).to_float * ADXL345_MG2G_MULTIPLIER_ * STANDARD_GRAVITY_
    y := (registers.read_i16_le REG_DATAY0_).to_float * ADXL345_MG2G_MULTIPLIER_ * STANDARD_GRAVITY_
    z := (registers.read_i16_le REG_DATAZ0_).to_float * ADXL345_MG2G_MULTIPLIER_ * STANDARD_GRAVITY_
    return [x,y,z]





/* Example

main:

  device := default_i2c.device _ADXL345_DEFAULT_ADDRESS

  sensor := ADXL345 device

  sensor.set_power POWER_CTL_NORMAL
  
  reading := sensor.read

*/
