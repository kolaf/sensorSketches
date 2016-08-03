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
#define MY_RFM69_FREQUENCY   RF69_869MHZ

#include <SPI.h>
#include <MySensors.h>

#include <DHT.h>
#include <SFE_BMP180.h>    //get it here: https://github.com/LowPowerLab/SFE_BMP180
#include <SI7021.h>        //get it here: https://github.com/LowPowerLab/SI7021
#include <Wire.h>

#define REPORT_INTERVAL 900000
#define ALTITUDE 220


#define CHILD_TEMPERATURE 1
#define CHILD_HUMIDITY 2
#define CHILD_PRESSURE 3
#define CHILD_POWER 4

SI7021 sensor;
SFE_BMP180 pressure;
int BATTERY_SENSE_PIN  = A7;  // select the input pin for the battery sense point
char *weather[] = {"stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown"};
int minutes;
float pressureSamples[180];
int minuteCount = 0;
bool firstRound = true;
float pressureAvg[7];
float dP_dt;
const int batteryReportCycle = 10;
int currentBatteryReportCycle = batteryReportCycle;
MyMessage temperatureMessage(CHILD_TEMPERATURE, V_TEMP);
MyMessage humidityMessage(CHILD_HUMIDITY, V_HUM);
MyMessage pressureMessage(CHILD_PRESSURE, V_PRESSURE);
MyMessage forecastMsg(CHILD_PRESSURE, V_FORECAST);
MyMessage powerMessage(CHILD_POWER, V_VOLTAGE);

bool changed = false;
float lastPressure = -1;
float lastTemperature = -1;
float lastHumidity = -1;
int lastForecast = -1;
float lastBattery = -1;
void setup()
{
  // use the 1.1 V internal reference
#if defined(__AVR_ATmega2560__)
  analogReference(INTERNAL1V1);
#else
  analogReference(INTERNAL);
#endif
  sensor.begin();
  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.

    Serial.println("BMP180 init fail\n\n");
    //while(1); // Pause forever.
  }
}

void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Outdoors environment", "2.0");

  // Register all sensors to gateway (they will be created as child devices)
  present(CHILD_TEMPERATURE, S_TEMP);
  delay(250);
  present(CHILD_HUMIDITY, S_HUM);
  delay(250);
  present(CHILD_PRESSURE, S_BARO);
  delay(250);
  present(CHILD_POWER, S_POWER);

}

void loop()
{
  changed = false;
  currentBatteryReportCycle--;
  float temperature = sensor.getCelsiusHundredths() / 100;
  int humidity = sensor.getHumidityPercent();
  if (lastTemperature != temperature) {
    changed = true;
    send(temperatureMessage.set(lastTemperature, 1));
    lastTemperature = temperature;
  }
  if (lastHumidity != humidity) {
    changed = true;
    send(humidityMessage.set(lastHumidity, 1));
    lastHumidity = humidity;
  }

  pinMode(A3, OUTPUT);
  digitalWrite(A3, LOW);
  lastBattery = analogRead(A7) / 10;
  pinMode(A3, INPUT);
#ifdef MY_DEBUG
  float batteryV  = lastBattery * 0.003363075 * 10;
  Serial.print("Battery Voltage: ");
  Serial.print(batteryV);
  Serial.println(" V");

  Serial.print("Battery percent: ");
  Serial.print(lastBattery);
  Serial.println(" %");
#endif

  char status;
  double T, P, p0, a;
  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0) {

      status = pressure.startPressure(3);
      if (status != 0)
      {
        delay(status);
        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          p0 = pressure.sealevel(P, ALTITUDE); // we're at 1655 meters (Boulder, CO)
          int forecast = 5;//sample(p0);
          if (lastPressure != p0) {
            changed = true;
            send(pressureMessage.set(lastPressure, 1));
            lastPressure = p0;
            Serial.print("relative (sea-level) pressure: ");
            Serial.print(p0, 2);
            Serial.print(" mb, ");
          }
          if (lastForecast != forecast) {
            changed = true;
            if (lastForecast > -1)
              send(forecastMsg.set(weather[lastForecast]));
            lastForecast = forecast;
          }
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");

  if (changed) {

  }

  if (currentBatteryReportCycle == 0) {
    sendBatteryLevel(lastBattery);
    currentBatteryReportCycle = batteryReportCycle;
  }
  Serial.println("Sleeping");
  sleep(REPORT_INTERVAL);
  Serial.println("Woken up");
}


int sample(float pressure) {
  // Algorithm found here
  // http://www.freescale.com/files/sensors/doc/app_note/AN3914.pdf
  if (minuteCount == 180)
    minuteCount = 5;

  pressureSamples[minuteCount] = pressure;
  minuteCount++;

  if (minuteCount == 5) {
    // Avg pressure in first 5 min, value averaged from 0 to 5 min.
    pressureAvg[0] = ((pressureSamples[0] + pressureSamples[1]
                       + pressureSamples[2] + pressureSamples[3] + pressureSamples[4])
                      / 5);
  } else if (minuteCount == 35) {
    // Avg pressure in 30 min, value averaged from 0 to 5 min.
    pressureAvg[1] = ((pressureSamples[30] + pressureSamples[31]
                       + pressureSamples[32] + pressureSamples[33]
                       + pressureSamples[34]) / 5);
    float change = (pressureAvg[1] - pressureAvg[0]);
    if (firstRound) // first time initial 3 hour
      dP_dt = ((65.0 / 1023.0) * 2 * change); // note this is for t = 0.5hour
    else
      dP_dt = (((65.0 / 1023.0) * change) / 1.5); // divide by 1.5 as this is the difference in time from 0 value.
  } else if (minuteCount == 60) {
    // Avg pressure at end of the hour, value averaged from 0 to 5 min.
    pressureAvg[2] = ((pressureSamples[55] + pressureSamples[56]
                       + pressureSamples[57] + pressureSamples[58]
                       + pressureSamples[59]) / 5);
    float change = (pressureAvg[2] - pressureAvg[0]);
    if (firstRound) //first time initial 3 hour
      dP_dt = ((65.0 / 1023.0) * change); //note this is for t = 1 hour
    else
      dP_dt = (((65.0 / 1023.0) * change) / 2); //divide by 2 as this is the difference in time from 0 value
  } else if (minuteCount == 95) {
    // Avg pressure at end of the hour, value averaged from 0 to 5 min.
    pressureAvg[3] = ((pressureSamples[90] + pressureSamples[91]
                       + pressureSamples[92] + pressureSamples[93]
                       + pressureSamples[94]) / 5);
    float change = (pressureAvg[3] - pressureAvg[0]);
    if (firstRound) // first time initial 3 hour
      dP_dt = (((65.0 / 1023.0) * change) / 1.5); // note this is for t = 1.5 hour
    else
      dP_dt = (((65.0 / 1023.0) * change) / 2.5); // divide by 2.5 as this is the difference in time from 0 value
  } else if (minuteCount == 120) {
    // Avg pressure at end of the hour, value averaged from 0 to 5 min.
    pressureAvg[4] = ((pressureSamples[115] + pressureSamples[116]
                       + pressureSamples[117] + pressureSamples[118]
                       + pressureSamples[119]) / 5);
    float change = (pressureAvg[4] - pressureAvg[0]);
    if (firstRound) // first time initial 3 hour
      dP_dt = (((65.0 / 1023.0) * change) / 2); // note this is for t = 2 hour
    else
      dP_dt = (((65.0 / 1023.0) * change) / 3); // divide by 3 as this is the difference in time from 0 value
  } else if (minuteCount == 155) {
    // Avg pressure at end of the hour, value averaged from 0 to 5 min.
    pressureAvg[5] = ((pressureSamples[150] + pressureSamples[151]
                       + pressureSamples[152] + pressureSamples[153]
                       + pressureSamples[154]) / 5);
    float change = (pressureAvg[5] - pressureAvg[0]);
    if (firstRound) // first time initial 3 hour
      dP_dt = (((65.0 / 1023.0) * change) / 2.5); // note this is for t = 2.5 hour
    else
      dP_dt = (((65.0 / 1023.0) * change) / 3.5); // divide by 3.5 as this is the difference in time from 0 value
  } else if (minuteCount == 180) {
    // Avg pressure at end of the hour, value averaged from 0 to 5 min.
    pressureAvg[6] = ((pressureSamples[175] + pressureSamples[176]
                       + pressureSamples[177] + pressureSamples[178]
                       + pressureSamples[179]) / 5);
    float change = (pressureAvg[6] - pressureAvg[0]);
    if (firstRound) // first time initial 3 hour
      dP_dt = (((65.0 / 1023.0) * change) / 3); // note this is for t = 3 hour
    else
      dP_dt = (((65.0 / 1023.0) * change) / 4); // divide by 4 as this is the difference in time from 0 value
    pressureAvg[0] = pressureAvg[5]; // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
    firstRound = false; // flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
  }

  if (minuteCount < 35 && firstRound) //if time is less than 35 min on the first 3 hour interval.
    return 5; // Unknown, more time needed
  else if (dP_dt < (-0.25))
    return 4; // Quickly falling LP, Thunderstorm, not stable
  else if (dP_dt > 0.25)
    return 3; // Quickly rising HP, not stable weather
  else if ((dP_dt > (-0.25)) && (dP_dt < (-0.05)))
    return 2; // Slowly falling Low Pressure System, stable rainy weather
  else if ((dP_dt > 0.05) && (dP_dt < 0.25))
    return 1; // Slowly rising HP stable good weather
  else if ((dP_dt > (-0.05)) && (dP_dt < 0.05))
    return 0; // Stable weather
  else
    return 5; // Unknown
}

