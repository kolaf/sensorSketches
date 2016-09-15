

// Enable debug prints to serial monitor
#define MY_DEBUG
#define USE_RADIO
// Enable and select radio type attached
#define MY_RADIO_RFM69
#define MY_IS_RFM69HW
#define MY_RFM69_FREQUENCY   RF69_867MHZ
#define MY_NODE_ID 6
#define MY_REPEATER_FEATURE
#define MY_OTA_FIRMWARE_FEATURE

#include <SPI.h>
#ifdef USE_RADIO
#include <MySensors.h>
#endif


#include <DHT.h>
#include <Bounce2.h>
#include <SFE_BMP180.h>    //get it here: https://github.com/LowPowerLab/SFE_BMP180
#include <SI7021.h>        //get it here: https://github.com/LowPowerLab/SI7021
#include <Wire.h>

#define REPORT_INTERVAL 900000
#define ALTITUDE 220
#define BUTTON 3 //cannot change because of interrupt
#define RELAY 6
#define LED 9

#define CHILD_TEMPERATURE 1
#define CHILD_HUMIDITY 2
#define CHILD_PRESSURE 3
#define CHILD_WATER 4
#define CHILD_TIME_REMAINING 5
//#define CHILD_POWER 9

SI7021 sensor;
SFE_BMP180 pressure;
int BATTERY_SENSE_PIN  = A7;  // select the input pin for the battery sense point


Bounce buttonBounce = Bounce();

char *weather[] = {"stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown"};
int minutes;
//float pressureSamples[180];
int minuteCount = 0;
bool firstRound = true;
float pressureAvg[7];
float dP_dt;

#ifdef USE_RADIO
// Change to V_LIGHT if you use S_LIGHT in presentation below
MyMessage temperatureMessage(CHILD_TEMPERATURE, V_TEMP);
MyMessage humidityMessage(CHILD_HUMIDITY, V_HUM);
MyMessage pressureMessage(CHILD_PRESSURE, V_PRESSURE);
MyMessage waterMessage(CHILD_WATER, V_LIGHT);
//MyMessage powerMessage(CHILD_POWER, V_VOLTAGE);
MyMessage timeRemainingMessage(CHILD_TIME_REMAINING, V_DISTANCE);
MyMessage forecastMsg(CHILD_PRESSURE, V_FORECAST);
#endif

signed long timeRemaining = 0;
unsigned long lastReport = 0, time, buttonStart = 0, buttonFinish = 0, lastCheck = 0, lastReduce = 0, lastPressureRead = 0;
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
  digitalWrite(RELAY, LOW);
  sensor.begin();
  pressure.begin();
  buttonBounce.attach(BUTTON);
  buttonBounce.interval(5);
}

#ifdef USE_RADIO
void presentation () {
  sendSketchInfo("Water control", "2.0");
  delay(250);
  present(CHILD_TEMPERATURE, S_TEMP);
  delay(250);
  present(CHILD_HUMIDITY, S_HUM);
  delay(250);
  present(CHILD_WATER, S_LIGHT);
  delay(250);
  present(CHILD_PRESSURE, S_BARO);
  //  delay(250);
  //  present(CHILD_POWER, S_POWER);
  delay(250);
  present(CHILD_TIME_REMAINING, S_DISTANCE);
}
#endif

void report(long time) {
  if (time - lastPressureRead > 60000) {
    float temperature = sensor.getCelsiusHundredths() / 100;
    int humidity = sensor.getHumidityPercent();
    if (lastTemperature != temperature) {
#ifdef USE_RADIO
      send(temperatureMessage.set(temperature, 1));
#endif
      Serial.println("SendingTemperatureMessage");
      lastTemperature = temperature;
    }
    if (lastHumidity != humidity) {
#ifdef USE_RADIO
      send(humidityMessage.set(humidity, 1));
#endif
      lastHumidity = humidity;
    }

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
            int forecast = 5;//sample(p0);
            if (lastPressure != p0) {
#ifdef USE_RADIO
              send(pressureMessage.set(p0, 1));
#endif
              lastPressure = p0;
            }
            if (lastForecast != forecast) {
#ifdef USE_RADIO
              send(forecastMsg.set(weather[forecast]));
#endif
              lastForecast = forecast;
            }
          }
        }
      }
    }
  }
}


void loop()
{
  buttonBounce.update();
  //  int value = digitalRead(BUTTON);
  int value = buttonBounce.read();
  if (value == 0 && buttonStart == 0) {
    time = millis();
    buttonStart = time;
  }
  do {
    delay(800);
    int remaining = (millis() - buttonStart);
    if (remaining > 1000) {
      blinkLight( (remaining - 1000) * 60);
    }
    buttonBounce.update();
  } while (buttonBounce.read() == 0);
  int remaining = millis() - buttonStart;
  buttonStart = 0; // Reset so that we can detect a new button press
  if (remaining > 1000) {
    remaining /= 1000;
    timeRemaining = 60000 * remaining;
    changed = true;
    lastReduce = millis();
  } else {
    timeRemaining = 0;
    changed = true;
    lastReduce = millis();
    digitalWrite(LED, HIGH);
    delay(1000);
    digitalWrite(LED, LOW);
  }
  time = millis();
  report(time);
  if (time - lastReduce > 15000  || changed) {
    changed = false;
    timeRemaining -= (time - lastReduce);
    if (timeRemaining > 0) {
      if (digitalRead(RELAY) == LOW) {
        digitalWrite(RELAY, HIGH);
#ifdef USE_RADIO
        send(waterMessage.set(true));
#endif
      }
      // Serial.println((timeRemaining));
#ifdef USE_RADIO
      send(timeRemainingMessage.set(timeRemaining / 60000.0, 1));
#endif
    } else {
      timeRemaining = 0;
      if (digitalRead(RELAY) == HIGH) {
        digitalWrite(RELAY, LOW);
#ifdef USE_RADIO
        send(waterMessage.set(false));
        send(timeRemainingMessage.set(0));
#endif
        // Serial.println((timeRemaining));
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
      sleep(BUTTON - 2, CHANGE, 15000);
    } else {
      sleep(BUTTON - 2, CHANGE, REPORT_INTERVAL);
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
    delay(100);
  }
}
#ifdef USE_RADIO
void receive(const MyMessage & message) {
  // We only expect one type of message from controller. But we better check anyway.
  bool state;
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
    //     saveState(CHILD_ID, state);

    // Write some debug info


  }
}
#endif

