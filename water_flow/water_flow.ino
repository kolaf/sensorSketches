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

// Change to V_LIGHT if you use S_LIGHT in presentation below
MyMessage temperatureMessage(CHILD_TEMPERATURE, V_TEMP);
MyMessage humidityMessage(CHILD_HUMIDITY, V_HUM);
MyMessage pressureMessage(CHILD_PRESSURE, V_PRESSURE);
MyMessage waterMessage(CHILD_WATER, V_LIGHT);
MyMessage powerMessage(CHILD_POWER, V_VOLTAGE);
MyMessage timeRemainingMessage(CHILD_TIME_REMAINING, V_DISTANCE);

signed long timeRemaining = 0;
unsigned long lastReport = 0, time, buttonStart = 0, buttonFinish = 0, lastCheck = 0, lastReduce = 0;
bool changed = false;

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
  delay(250);
  gw.present(CHILD_POWER, S_POWER);
  delay(250);
  gw.present(CHILD_TIME_REMAINING, S_DISTANCE);
}


//  Check if digital input has changed and send in new value
void loop()
{
  // Get the update value

  gw.process();
  time = millis();
  if (time - lastReport > REPORT_INTERVAL) {
    lastReport = time;
    //*************** READING BATTERY VOLTAGE *********************
    //turn MOSFET ON and read voltage, should give a valid reading
    pinMode(A3, OUTPUT);
    digitalWrite(A3, LOW);
    Serial.print("  BATT: ");
    int sensorValue = analogRead(A7);
    Serial.println(sensorValue);
    float batteryV  = sensorValue / 10 * 0.00520430107;
    int batteryPcnt = sensorValue / 100;
    gw.send(powerMessage.set(batteryV, 1));
    gw.sendBatteryLevel(batteryPcnt);
    pinMode(A3, INPUT); //put A3 in HI-Z mode (to allow mosfet gate pullup to turn it OFF)
    //*************** READING BATTERY VOLTAGE *********************
    Serial.println("************ BMP180 *********************************");
    Serial.print("provided altitude: ");
    Serial.print(ALTITUDE, 0);
    Serial.print(" meters, ");
    Serial.print(ALTITUDE * 3.28084, 0);
    Serial.println(" feet");

    float temperature = sensor.getCelsiusHundredths() / 100;
    Serial.println("************ Si7021 *********************************");
    Serial.print("C: "); Serial.print(temperature);
    int humidity = sensor.getHumidityPercent();
    Serial.print("   H: "); Serial.print(humidity); Serial.print("%   ");
    gw.send(temperatureMessage.set(temperature, 1));
    gw.send(humidityMessage.set(humidity, 1));
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
        Serial.print("C: ");
        Serial.print(T, 2);
        Serial.print("    F:");
        Serial.print((9.0 / 5.0)*T + 32.0, 2);
        Serial.println("");
        status = pressure.startPressure(3);

        if (status != 0)
        {
          delay(status);
          status = pressure.getPressure(P, T);
          if (status != 0)
          {
            // Print out the measurement:
            Serial.print("abs pressure: ");
            Serial.print(P, 2);
            Serial.print(" mb, ");
            Serial.print(P * 0.0295333727, 2);
            Serial.println(" inHg");

            // The pressure sensor returns abolute pressure, which varies with altitude.
            // To remove the effects of altitude, use the sealevel function and your current altitude.
            // This number is commonly used in weather reports.
            // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
            // Result: p0 = sea-level compensated pressure in mb

            p0 = pressure.sealevel(P, ALTITUDE); // we're at 1655 meters (Boulder, CO)
            Serial.print("relative (sea-level) pressure: ");
            Serial.print(p0, 2);
            Serial.print(" mb, ");
            Serial.print(p0 * 0.0295333727, 2);
            Serial.println(" inHg");
            gw.send(pressureMessage.set(p0, 1));
          }
        }
      }
    }
  }

  buttonBounce.update();
  //  int value = digitalRead(BUTTON);
  int value = buttonBounce.read();
  if (value == 0 && buttonStart == 0) {
    buttonStart = time;
    Serial.println("bbutton pressed");
  } else if (value == 1 && buttonStart  > 0) {
    buttonFinish = time;
    Serial.print("bbutton released: ");
    Serial.println(buttonFinish - buttonStart);
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

void incomingMessage(const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.
  bool state;
  if (message.isAck()) {
    Serial.println("This is an ack from gateway");
  }

  if (message.type == V_LIGHT && strlen(message.getString()) != 0) {
    // Change relay state
    state = message.getBool();
    if (state) {
      timeRemaining += 480000;
    } else {
      timeRemaining=0;
    }
    digitalWrite(RELAY, state ? LOW : HIGH);
    // Store state in eeprom
    //     gw.saveState(CHILD_ID, state);

    // Write some debug info
    Serial.print("Incoming change for sensor:");
    Serial.print(message.sensor);
    Serial.print(", New status: ");
    Serial.println(message.getBool());
  }
}
