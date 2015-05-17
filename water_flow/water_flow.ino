#include <MySigningNone.h>
#include <MyTransportRFM69.h>
#include <MyTransportNRF24.h>
#include <MyHwATMega328.h>
#include <MySigningAtsha204Soft.h>
#include <MySigningAtsha204.h>

#include <MySensor.h>
#include <SPI.h>
#include <DHT.h>
#include <Bounce2.h>
#include <SFE_BMP180.h>    //get it here: https://github.com/LowPowerLab/SFE_BMP180
#include <SI7021.h>        //get it here: https://github.com/LowPowerLab/SI7021
#include <Wire.h>

#define REPORT_INTERVAL 300000
#define ALTITUDE 220
#define BUTTON 3 //cannot change because of interrupt
#define RELAY 6
#define LED 9

#define CHILD_TEMPERATURE 1
#define CHILD_HUMIDITY 2
#define CHILD_PRESSURE 3
#define CHILD_WATER 4
#define CHILD_TIME_REMAINING 5
#define CHILD_POWER 9

SI7021 sensor;
SFE_BMP180 pressure;
int BATTERY_SENSE_PIN  = A7;  // select the input pin for the battery sense point

MyTransportRFM69 transport;
// Hardware profile
MyHwATMega328 hw;

MySensor gw(transport, hw /*, signer*/);

Bounce buttonBounce = Bounce();

char *weather[] = {"stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown"};
int minutes;
float pressureSamples[180];
int minuteCount = 0;
bool firstRound = true;
float pressureAvg[7];
float dP_dt;

// Change to V_LIGHT if you use S_LIGHT in presentation below
MyMessage temperatureMessage(CHILD_TEMPERATURE, V_TEMP);
MyMessage humidityMessage(CHILD_HUMIDITY, V_HUM);
MyMessage pressureMessage(CHILD_PRESSURE, V_PRESSURE);
MyMessage waterMessage(CHILD_WATER, V_LIGHT);
//MyMessage powerMessage(CHILD_POWER, V_VOLTAGE);
MyMessage timeRemainingMessage(CHILD_TIME_REMAINING, V_DISTANCE);
MyMessage forecastMsg(CHILD_PRESSURE, V_FORECAST);

signed long timeRemaining = 0;
unsigned long lastReport = 0, time, buttonStart = 0, buttonFinish = 0, lastCheck = 0, lastReduce = 0,lastPressureRead = 0;
bool changed = false;

float lastPressure = -1;
float lastTemperature = -1;
float lastHumidity = -1;
int lastForecast = -1;

void setup()
{
  pinMode(RELAY, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(BATTERY_SENSE_PIN, INPUT);
  pinMode(BUTTON, INPUT);
  digitalWrite(BUTTON, HIGH);
  digitalWrite(RELAY, HIGH);
  sensor.begin();
  pressure.begin();
  buttonBounce.attach(BUTTON);
  buttonBounce.interval(5);
  gw.begin(incomingMessage, 6, true);
  gw.sendSketchInfo("Water control", "1.0");
  delay(250);
  gw.present(CHILD_TEMPERATURE, S_TEMP);
  delay(250);
  gw.present(CHILD_HUMIDITY, S_HUM);
  delay(250);
  gw.present(CHILD_WATER, S_LIGHT);
  delay(250);
  gw.present(CHILD_PRESSURE, S_BARO);
  //  delay(250);
  //  gw.present(CHILD_POWER, S_POWER);
  delay(250);
  gw.present(CHILD_TIME_REMAINING, S_DISTANCE);
}


//  Check if digital input has changed and send in new value
void loop()
{
  // Get the update value

  gw.process();
  time = millis();
  float temperature = sensor.getCelsiusHundredths() / 100;
  int humidity = sensor.getHumidityPercent();
  if (lastTemperature != temperature) {
    gw.send(temperatureMessage.set(temperature, 1));
    Serial.println("SendingTemperatureMessage");
    lastTemperature = temperature;
  }
  if (lastHumidity != humidity) {
    gw.send(humidityMessage.set(humidity, 1));
    lastHumidity = humidity;
  }
  if(time-lastPressureRead>60000){
    lastPressureRead = time;
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
          int forecast = sample(p0);
          if (lastPressure != p0) {
            gw.send(pressureMessage.set(p0, 1));
            lastPressure = p0;
          }
          if (lastForecast != forecast) {
            gw.send(forecastMsg.set(weather[forecast]));
            lastForecast = forecast;
          }
        }
      }
    }
  }
  }
  //  }

  buttonBounce.update();
  //  int value = digitalRead(BUTTON);
  int value = buttonBounce.read();
  if (value == 0 && buttonStart == 0) {
    buttonStart = time;
    //    Serial.println("bbutton pressed");
  } else if (value == 1 && buttonStart  > 0) {
    buttonFinish = time;
    //    Serial.print("bbutton released: ");
    //  Serial.println(buttonFinish - buttonStart);
    if (buttonFinish - buttonStart < 3000) { //increase flow time
      timeRemaining += 60000;
      lastReduce = time;
      changed = true;
    } else {
      timeRemaining = 0;
      changed = true;
      lastReduce = time;
      digitalWrite(LED, HIGH);
      delay(1000);
      digitalWrite(LED, LOW);
    }
    buttonFinish = 0;
    buttonStart = 0;
  }
  if (time - lastReduce > 15000  || changed) {
    changed = false;
    timeRemaining -= (time - lastReduce);
    if (timeRemaining > 0) {
      if (digitalRead(RELAY) == HIGH) {
        digitalWrite(RELAY, LOW);
        gw.send(waterMessage.set(true));
      }
      Serial.println((timeRemaining));
      gw.send(timeRemainingMessage.set(timeRemaining / 60000.0, 1));
    } else {
      timeRemaining = 0;
      if (digitalRead(RELAY) == LOW) {
        digitalWrite(RELAY, HIGH);
        gw.send(waterMessage.set(false));
        gw.send(timeRemainingMessage.set(timeRemaining / 60000.0, 1));
        Serial.println((timeRemaining));
      }



    }
    lastReduce = time;
    blinkLight(timeRemaining);
  }
  //  Serial.print("Time remaining : ");
  //  Serial.println(timeRemaining);

  lastCheck = time;
  /*
    if(buttonStart>0){
    if (timeRemaining > 0) {
      gw.sleep(BUTTON - 2, CHANGE, 15000);
    } else {
      gw.sleep(BUTTON - 2, CHANGE, REPORT_INTERVAL);
    }
    }
  */
}

void blinkLight(signed long remaining) {
  int number = 1 + remaining / 60000;
  if (remaining == 0) {
    number = 0;
  }
  for (int i = 0; i < number ; i++) {
    digitalWrite(LED, HIGH);
    delay(50);
    digitalWrite(LED, LOW);
    delay(200);
  }
}

void incomingMessage(const MyMessage & message) {
  // We only expect one type of message from controller. But we better check anyway.
  bool state;
  /*  if (message.isAck()) {
      Serial.println("This is an ack from gateway");
    }*/

  if (message.type == V_LIGHT && strlen(message.getString()) != 0) {
    // Change relay state
    state = message.getBool();
    if (state) {
      timeRemaining = 480000;
    } else {
      timeRemaining = 0;
    }
    digitalWrite(RELAY, state ? LOW : HIGH);
    // Store state in eeprom
    //     gw.saveState(CHILD_ID, state);

    // Write some debug info

    /*    Serial.print("Incoming change for sensor:");
        Serial.print(message.sensor);
        Serial.print(", New status: ");
        Serial.println(message.getBool());
        */
  }
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

