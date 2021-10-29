/*
   @file    LPS22HH_DataLogTerminal.ino
   @author  Frederic Pillon <frederic.pillon@st.com>
   @brief   Example to use the LPS22HH 260-1260 hPa absolute digital
            output barometer
 *******************************************************************************
   Copyright (c) 2021, STMicroelectronics
   All rights reserved.

   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause

 *******************************************************************************
*/


// Includes
#include <LPS22HHSensor.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN PNUM_NOT_DEFINED
#warning "LED_BUILTIN is not defined."
#endif

#define SerialPort  Serial

#if defined(ARDUINO_B_L4S5I_IOT01A) || defined(ARDUINO_B_U585I_IOT02A)
#define USE_I2C_INTERFACE
#else
// Uncomment one communication interface to use:
// X_NUCLEO_IKS01A3 uses I2C interface (set by default)
// #define USE_SPI_INTERFACE
#define USE_I2C_INTERFACE
#endif

#if !defined(USE_SPI_INTERFACE) && !defined(USE_I2C_INTERFACE) || \
    defined(USE_SPI_INTERFACE) && defined(USE_I2C_INTERFACE)
#error "Uncomment one communication interface to use!"
#endif

#ifdef USE_SPI_INTERFACE
// Uncomment to set SPI pins to use else default instance will be used
// #define LPS22HH_SPI_MOSI PB10
// #define LPS22HH_SPI_MISO PB11
// #define LPS22HH_SPI_SCLK PB11
// Define Software Chip Select pin to use (default: SS)
#define LPS22HH_SPI_SSEL    SS

#if defined(LPS22HH_SPI_MOSI) && defined(LPS22HH_SPI_MISO) && \
    defined(LPS22HH_SPI_SCLK) && defined(LPS22HH_SPI_SSEL)
SPIClass dev_interface(LPS22HH_SPI_MOSI, LPS22HH_SPI_MISO, LPS22HH_SPI_SCLK);
#else
#define dev_interface       SPI
#endif

LPS22HHSensor PressTemp(&dev_interface, LPS22HH_SPI_SSEL);
#else // USE_I2C_INTERFACE
#if defined(ARDUINO_B_L4S5I_IOT01A)
#define LPS22HH_I2C_SCL     PB10
#define LPS22HH_I2C_SDA     PB11
#elif defined(ARDUINO_B_U585I_IOT02A)
#define LPS22HH_I2C_SCL     PH4
#define LPS22HH_I2C_SDA     PH5
#else
// Uncomment to set I2C pins to use else default instance will be used
// #define LPS22HH_I2C_SCL  PYn
// #define LPS22HH_I2C_SDA  PYn
#endif
#if defined(LPS22HH_I2C_SCL) && defined(LPS22HH_I2C_SDA)
TwoWire dev_interface(LPS22HH_I2C_SDA, LPS22HH_I2C_SCL);
#else
// X_NUCLEO_IKS01A3 uses default instance
#define dev_interface       Wire
#endif

LPS22HHSensor PressTemp(&dev_interface);
#endif

void setup() {
  // Led
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize serial for output
  SerialPort.begin(9600);

  // Initialize bus interface
  dev_interface.begin();

  // Initlialize component
  PressTemp.begin();
  PressTemp.Enable();
}

void loop() {
  // Led blinking
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);

  // Read pressure
  float pressure, temperature;
  PressTemp.GetPressure(&pressure);
  PressTemp.GetTemperature(&temperature);

  SerialPort.print("Pres[hPa]:");
  SerialPort.print(pressure, 2);
  SerialPort.print(", Temp[C]:");
  SerialPort.println(temperature, 2);
}
