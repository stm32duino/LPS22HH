/*
   @file    LPS22HH_Datalog_Terminal_I3C.ino
   @author  STMicroelectronics
   @brief   Example to use the LPS22HH pressure sensor with I3C and SETDASA command
 *******************************************************************************
   Copyright (c) 2026, STMicroelectronics
   All rights reserved.

   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause

 *******************************************************************************
*/
#include "LPS22HHSensor.h"

#define LPS22HH_DYNAMIC_ADDRESS 0x30

LPS22HHSensor sensor(&I3C, LPS22HH_I3C_ADD_H);

void setup()
{
  Serial.begin(115200);
  while (!Serial) {}
  delay(1000);

  Serial.println("=== LPS22HH SETDASA ===");

  if (!I3C.begin(I3C_SDA, I3C_SCL, 1000000U)) {
    Serial.println("begin() failed");
    while (1) {}
  }

  if (!I3C.resetDynamicAddresses()) {
    Serial.println("resetDynamicAddresses() failed");
    while (1) {}
  }

  if (!I3C.assignDynamicAddress(sensor.getStaticAddress(), LPS22HH_DYNAMIC_ADDRESS)) {
    Serial.println("assignDynamicAddress() failed");
    while (1) {}
  }

  if (sensor.begin(LPS22HH_DYNAMIC_ADDRESS) != LPS22HH_OK) {
    Serial.println("sensor.begin() failed");
    while (1) {}
  }

  if (!I3C.setClock(12500000)) {
    Serial.println("setClock() failed");
    while (1) {}
  }

  if (sensor.Enable() != LPS22HH_OK) {
    Serial.println("sensor.Enable() failed");
    while (1) {}
  }

  Serial.println("LPS22HH ready");
}

void loop()
{
  float pressure = 0;
  float temperature = 0;

  if (sensor.GetPressure(&pressure) == LPS22HH_OK && sensor.GetTemperature(&temperature) == LPS22HH_OK) {
    Serial.print("Pres[hPa]:");
    Serial.print(pressure, 2);
    Serial.print(",Temp[C]:");
    Serial.println(temperature, 2);
  } else {
    Serial.println("Read failed");
  }

  delay(500);
}
