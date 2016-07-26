/**
   The MySensors Arduino library handles the wireless radio link and protocol
   between your home built sensors/actuators and HA controller of choice.
   The sensors forms a self healing radio network with optional repeaters. Each
   repeater and gateway builds a routing tables in EEPROM which keeps track of the
   network topology allowing messages to be routed to nodes.

   Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
   Copyright (C) 2013-2015 Sensnology AB
   Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors

   Documentation: http://www.mysensors.org
   Support Forum: http://forum.mysensors.org

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   version 2 as published by the Free Software Foundation.

 *******************************

   REVISION HISTORY
   Version 1.0 - Henrik EKblad

   DESCRIPTION
   Example sketch showing how to measue light level using a LM393 photo-resistor
   http://www.mysensors.org/build/light
*/

// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_RFM69
#define MY_IS_RFM69HW
#define MY_RFM69_FREQUENCY   RF69_868MHZ
#define MY_REPEATER_FEATURE

#include <SPI.h>
#include <MySensors.h>

#include <DallasTemperature.h>
#include <OneWire.h>

#define CHILD_TEMPERATURE 1
#define CHILD_POWER 2
#define ONE_WIRE_BUS 7


OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

float lastTemperature  = 0;
long lastTemperatureSent = 0;
unsigned long lastSend;
unsigned long SEND_FREQUENCY = 900000; // Sleep time between reads (in milliseconds)


MyMessage temperatureMessage(CHILD_TEMPERATURE, V_TEMP);


void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Basement temperature", "2.0");

  // Register all sensors to gateway (they will be created as child devices)
  present(CHILD_TEMPERATURE, S_TEMP);
}

void loop()
{
  unsigned long now = millis();
  // Only send values at a maximum frequency or woken up from sleep
  bool sendTime = now - lastSend > SEND_FREQUENCY || now - lastSend < 0;
  if (sendTime) {
    lastSend = now;
    sensors.requestTemperatures();
    delay(100);
    float temperature = static_cast<float>(sensors.getTempCByIndex(0));
    if (temperature  != -127) {
      send(temperatureMessage.set(temperature, 1));
    }
  }
}



