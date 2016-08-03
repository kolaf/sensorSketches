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
   Version 1.0 - Henrik Ekblad

   DESCRIPTION

   Secret Knock Sensor
   http://www.mysensors.org/build/knock

   See original instructions here (note: The MySensors adopted code might differ in wiring. The instructions below is correct):
   https://learn.adafruit.com/secret-knock-activated-drawer-lock/
   Version 13.10.31  Built with Arduino IDE 1.0.5

   By Steve Hoefer http://grathio.com
   Adapted to MySensors by Henrik Ekblad

   Licensed under Creative Commons Attribution-Noncommercial-Share Alike 3.0
   http://creativecommons.org/licenses/by-nc-sa/3.0/us/
   (In short: Do what you want, as long as you credit me, don't relicense it, and don't sell it or use it in anything you sell without contacting me.)

   ------Wiring------
   Pin 0: Program button used for recording a new Knock (connect Pin0 -> button -> GND)
   Pin 1: Optional: Connect LED here (remember resisor in series)
   Pin 2: Optional: Piezo element (for beeps).
   Pin 5: A sound sensor (digital output) for sensing knocks. See MySensors purchase guide. I used this: http://rover.ebay.com/rover/1/711-53200-19255-0/1?icep_ff3=2&pub=5575069610&toolid=10001&campid=5337433187&customid=&icep_item=200941260251&ipn=psmain&icep_vectorid=229466&kwid=902099&mtid=824&kw=lg
   Pin 4: Connects to either 1. Relay which open door or lock or
                             2. transistor that opens a solenoid lock when HIGH (see adafruit guide for this option).


   Connect radio according as usual(you can skip IRQ pin)
   http://www.mysensors.org/build/connect_radio
*/

// Enable debug prints to serial monitor
#define MY_DEBUG
// Enable and select radio type attached
#define MY_RADIO_RFM69
#define MY_IS_RFM69HW
#define MY_RFM69_FREQUENCY   RF69_869MHZ
#define MY_NODE_ID 7
#define MY_REPEATER_FEATURE

#include <MySensors.h>
#include <SPI.h>
#include <DHT.h>

#define CHILD_ID_HUM 3
#define CHILD_ID_TEMP 4
#define HUMIDITY_SENSOR_DIGITAL_PIN 3

#define CHILD_ID0 1   // Id of the sensor child
#define CHILD_ID1 2   // Id of the sensor child
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(HUMIDITY_SENSOR_DIGITAL_PIN, DHTTYPE);
float lastTemp;
float lastHum;
const int maximumNoChange = 3;
int noChangeCounter = maximumNoChange;
long lastTemperatureReport = 10000000;
const long temperatureReportInterval = 900000;
boolean metric = true;
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);

const byte eepromValid = 120;    // If the first byte in eeprom is this then the data is valid.

/*Pin definitions*/
const int programButton = 0;   // (Digital 0) Record A New Knock button.
const int ledPin = 7;          // (Digital 1) The LED pin (if any)
const int knockSensor = 5;     // (Digital 5) for using the microphone digital output (tune knob to register knock)
const int audioOut = 2;        // (Digital 2) for using the peizo as an output device. (Thing that goes beep.)
const int lockPin = 4;         // (Digital 4) The pin that activates the relay/solenoid lock.

/*Tuning constants. Changing the values below changes the behavior of the device.*/
int threshold = 3;                 // Minimum signal from the piezo to register as a knock. Higher = less sensitive. Typical values 1 - 10
const int rejectValue = 25;        // If an individual knock is off by this percentage of a knock we don't unlock. Typical values 10-30
const int averageRejectValue = 15; // If the average timing of all the knocks is off by this percent we don't unlock. Typical values 5-20
const int knockFadeTime = 150;     // Milliseconds we allow a knock to fade before we listen for another one. (Debounce timer.)
const int lockOperateTime = 2500;  // Milliseconds that we operate the lock solenoid latch before releasing it.
const int maximumKnocks = 20;      // Maximum number of knocks to listen for.
const int maximumKnockCodes = 2;
const int knockComplete = 1200;    // Longest time to wait for a knock before we assume that it's finished. (milliseconds)

byte secretCodes[maximumKnockCodes][maximumKnocks] = {{100, 50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {50, 25, 25, 50, 100, 50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}; // Initial setup: "Shave and a Hair Cut, two bits."
int knockReadings[maximumKnocks];    // When someone knocks this array fills with the delays between knocks.
int knockSensorValue = 0;            // Last reading of the knock sensor.
boolean programModeActive = false;   // True if we're trying to program a new knock.

bool switchStatus[maximumKnockCodes] = {false, false};

bool lockStatus;
MyMessage clapperMessage0(CHILD_ID0, V_LIGHT);
MyMessage clapperMessage1(CHILD_ID1, V_LIGHT);


void setup() {

  pinMode(ledPin, OUTPUT);
  pinMode(knockSensor, INPUT);
  pinMode(lockPin, OUTPUT);
  pinMode(programButton, INPUT);
  digitalWrite(ledPin, LOW);
  digitalWrite(programButton, HIGH); // Enable internal pull up
  readSecretKnock();   // Load the secret knock (if any) from EEPROM.
  dht.begin();
  playbackKnocks();

}

void presentation() {
  sendSketchInfo("Bedroom clapper", "1.0");
  present(CHILD_ID0, S_LIGHT);
  present(CHILD_ID1, S_LIGHT);
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);

  metric = getConfig().isMetric;
}


void handleTemperature() {
  Serial.println("Measuring temperature and humidity");
  float temperature = dht.readTemperature();
  if (isnan(temperature)) {
    Serial.println("Failed reading temperature from DHT");
  } else if (temperature != lastTemp || noChangeCounter == 0) {
    lastTemp = temperature;
    noChangeCounter = maximumNoChange;
    if (!metric) {
      temperature = dht.readTemperature(true);
    }
    send(msgTemp.set(temperature, 2));
    Serial.print("T: ");
    Serial.println(temperature);
  } else {
    noChangeCounter--;
  }

  float humidity = dht.readHumidity();
  if (isnan(humidity)) {
    Serial.println("Failed reading humidity from DHT");
  } else if (humidity != lastHum) {
    lastHum = humidity;
    send(msgHum.set(humidity, 2));
    Serial.print("H: ");
    Serial.println(humidity);
  }
}


void loop() {
  // Listen for any knock at all.
  long now = millis();
  if (abs(lastTemperatureReport - now) > temperatureReportInterval) {
    // Serial.println( now);
    // Serial.println(lastTemperatureReport);
    handleTemperature();
    lastTemperatureReport = now;
  }
  knockSensorValue = digitalRead(knockSensor);
  if (digitalRead(programButton) == LOW) { // is the program button pressed?
    delay(100);   // Cheap debounce.
    if (digitalRead(programButton) == LOW) {
      if (programModeActive == false) {    // If we're not in programming mode, turn it on.
        programModeActive = true;          // Remember we're in programming mode.
        digitalWrite(ledPin, HIGH);        // Turn on the red light too so the user knows we're programming.
        chirp(500, 1500);                  // And play a tone in case the user can't see the LED.
        chirp(500, 1000);
      } else {                             // If we are in programing mode, turn it off.
        programModeActive = false;
        digitalWrite(ledPin, LOW);
        chirp(500, 1000);                  // Turn off the programming LED and play a sad note.
        chirp(500, 1500);
        delay(500);
      }
      while (digitalRead(programButton) == LOW) {
        delay(10);                         // Hang around until the button is released.
      }
    }
    delay(250);   // Another cheap debounce. Longer because releasing the button can sometimes be sensed as a knock.
  }


  if (knockSensorValue == 0) {
    if (programModeActive == true) { // Blink the LED when we sense a knock.
      digitalWrite(ledPin, LOW);
    } else {
      digitalWrite(ledPin, HIGH);
    }
    knockDelay();
    if (programModeActive == true) { // Un-blink the LED.
      digitalWrite(ledPin, HIGH);
    } else {
      digitalWrite(ledPin, LOW);
    }
    listenToSecretKnock();           // We have our first knock. Go and see what other knocks are in store...
  }

}

// Records the timing of knocks.
void listenToSecretKnock() {
  int i = 0;
  // First reset the listening array.
  for (i = 0; i < maximumKnocks; i++) {
    knockReadings[i] = 0;
  }

  int currentKnockNumber = 0;               // Position counter for the array.
  int startTime = millis();                 // Reference for when this knock started.
  int now = millis();

  do {                                      // Listen for the next knock or wait for it to timeout.
    knockSensorValue = digitalRead(knockSensor);

    if (knockSensorValue == 0) {                  // Here's another knock. Save the time between knocks.
      Serial.println("knock");

      now = millis();
      knockReadings[currentKnockNumber] = now - startTime;
      currentKnockNumber ++;
      startTime = now;

      if (programModeActive == true) {  // Blink the LED when we sense a knock.
        digitalWrite(ledPin, LOW);
      } else {
        digitalWrite(ledPin, HIGH);
      }
      knockDelay();
      if (programModeActive == true) { // Un-blink the LED.
        digitalWrite(ledPin, HIGH);
      } else {
        digitalWrite(ledPin, LOW);
      }
    }

    now = millis();

    // Stop listening if there are too many knocks or there is too much time between knocks.
  } while ((now - startTime < knockComplete) && (currentKnockNumber < maximumKnocks));
  Serial.println("end");

  //we've got our knock recorded, lets see if it's valid
  if (programModeActive == false) {          // Only do this if we're not recording a new knock.

    if (validateKnock(0) == true) {
      // Lock/unlock door
      chirp(500, 1500);                  // And play a tone in case the user can't see the LED.
      chirp(500, 1000);
      digitalWrite(ledPin, HIGH);
      switchStatus[0] = !switchStatus[0];
      send( clapperMessage0.set(switchStatus[0]));

      delay(2000);
      digitalWrite(ledPin,  LOW);

    } else if (validateKnock(1) == true) {
      // Lock/unlock door
      chirp(500, 1500);                  // And play a tone in case the user can't see the LED.
      chirp(500, 1000);
      digitalWrite(ledPin, HIGH);
      switchStatus[1] = !switchStatus[1];
      send( clapperMessage1.set(switchStatus[1]));

      delay(2000);
      digitalWrite(ledPin,  LOW);

    } else {
      Serial.println("fail unlock");
      /*
            // knock is invalid. Blink the LED as a warning to others.
            for (i=0; i < 4; i++){
              digitalWrite(ledPin, HIGH);
              delay(50);
              digitalWrite(ledPin, LOW);
              delay(50);
            }
      */
    }

  } else { // If we're in programming mode we still validate the lock because it makes some numbers we need, we just don't do anything with the return.
    validateKnock(0);
  }
}


// Checks to see if our knock matches the secret.
// Returns true if it's a good knock, false if it's not.
boolean validateKnock(int number) {
  int i = 0;

  int currentKnockCount = 0;
  int secretKnockCount = 0;
  int maxKnockInterval = 0;               // We use this later to normalize the times.

  for (i = 0; i < maximumKnocks; i++) {
    if (knockReadings[i] > 0) {
      currentKnockCount++;
    }
    if (secretCodes[number][i] > 0) {
      secretKnockCount++;
    }
    if (knockReadings[i] > maxKnockInterval) {  // Collect normalization data while we're looping.
      maxKnockInterval = knockReadings[i];
    }
  }

  // If we're recording a new knock, save the info and get out of here.
  /*
    if (programModeActive == true) {
    for (i = 0; i < maximumKnocks; i++) { // Normalize the time between knocks. (the longest time = 100)
      secretCode[i] = map(knockReadings[i], 0, maxKnockInterval, 0, 100);
    }
    saveSecretKnock();                // save the result to EEPROM
    programModeActive = false;
    playbackKnock(maxKnockInterval);
    return false;
    }
  */
  if (currentKnockCount != secretKnockCount) { // Easiest check first. If the number of knocks is wrong, don't unlock.
    return false;
  }

  /*  Now we compare the relative intervals of our knocks, not the absolute time between them.
      (ie: if you do the same pattern slow or fast it should still open the door.)
      This makes it less picky, which while making it less secure can also make it
      less of a pain to use if you're tempo is a little slow or fast.
  */
  int totaltimeDifferences = 0;
  int timeDiff = 0;
  for (i = 0; i < maximumKnocks; i++) { // Normalize the times
    knockReadings[i] = map(knockReadings[i], 0, maxKnockInterval, 0, 100);
    timeDiff = abs(knockReadings[i] - secretCodes[number][i]);
    if (timeDiff > rejectValue) {       // Individual value too far out of whack. No access for this knock!
      return false;
    }
    totaltimeDifferences += timeDiff;
  }
  // It can also fail if the whole thing is too inaccurate.
  if (totaltimeDifferences / secretKnockCount > averageRejectValue) {
    return false;
  }

  return true;
}


// reads the secret knock from EEPROM. (if any.)
void readSecretKnock() {
  byte reading;
  reading = loadState(1);
  if (reading == eepromValid) {   // only read EEPROM if the signature byte is correct.
    for (int i = 0; i < maximumKnocks * maximumKnockCodes; i++) {
      secretCodes[i % maximumKnocks][i] =  loadState(i + 2);
    }
  }
}


//saves a new pattern too eeprom
void saveSecretKnock() {
  saveState(1, 0); // clear out the signature. That way we know if we didn't finish the write successfully.
  for (int i = 0; i < maximumKnocks * maximumKnockCodes; i++) {
    saveState(i + 2, secretCodes[i % maximumKnocks][i]);
  }
  saveState(1, eepromValid);  // all good. Write the signature so we'll know it's all good.
}

// Plays back the pattern of the knock in blinks and beeps

void playbackKnocks() {
  for (int i = 0; i < maximumKnockCodes; i++) {
    for (int blink = 0; blink < i + 1; blink++) {
      digitalWrite(ledPin, HIGH);
      delay(1000);
      digitalWrite(ledPin, LOW);
      delay(500);
    }
    playbackKnock(i, 400);
    delay(2000);
  }
}

void playbackKnock(int number, int maxKnockInterval) {
  digitalWrite(ledPin, LOW);
  delay(1000);
  digitalWrite(ledPin, HIGH);
  chirp(200, 1800);
  for (int i = 0; i < maximumKnocks ; i++) {
    digitalWrite(ledPin, LOW);
    // only turn it on if there's a delay
    if (secretCodes[number][i] > 0) {
      delay(map(secretCodes[number][i], 0, 100, 0, maxKnockInterval)); // Expand the time back out to what it was. Roughly.
      digitalWrite(ledPin, HIGH);
      chirp(200, 1800);
    }
  }
  digitalWrite(ledPin, LOW);
}

// Deals with the knock delay thingy.
void knockDelay() {
  int itterations = (knockFadeTime / 20);      // Wait for the peak to dissipate before listening to next one.
  for (int i = 0; i < itterations; i++) {
    delay(10);
    analogRead(knockSensor);                  // This is done in an attempt to defuse the analog sensor's capacitor that will give false readings on high impedance sensors.
    delay(10);
  }
}

// Plays a non-musical tone on the piezo.
// playTime = milliseconds to play the tone
// delayTime = time in microseconds between ticks. (smaller=higher pitch tone.)
void chirp(int playTime, int delayTime) {
  long loopTime = (playTime * 1000L) / delayTime;
  pinMode(audioOut, OUTPUT);
  for (int i = 0; i < loopTime; i++) {
    digitalWrite(audioOut, HIGH);
    delayMicroseconds(delayTime);
    digitalWrite(audioOut, LOW);
  }
  pinMode(audioOut, INPUT);
}


