

// Enable debug prints to serial monitor
#define MY_DEBUG
#define USE_RADIO
// Enable and select radio type attached
#define MY_RADIO_RFM69
#define MY_IS_RFM69HW
#define MY_RFM69_FREQUENCY   RF69_867MHZ
//#define MY_RFM69_FREQUENCY   RF69_869MHZ
#define MY_NODE_ID 9
#define MY_REPEATER_FEATURE
#define MY_OTA_FIRMWARE_FEATURE

#include <SPI.h>
#ifdef USE_RADIO
#include <MySensors.h>
#endif


#include <DHT.h>
#include <Bounce2.h>
#define REPORT_INTERVAL 900000
#define WATER_REPORT_INTERVAL 15000
#define ALTITUDE 220
#define WATER_BUTTON 3 //cannot change because of interrupt
#define CAMERA_BUTTON 4 //cannot change because of interrupt
#define RELAY 6
#define LED 9
#define CAMERA_LED 8
#define CHILD_CAMERA_SWITCH 1
#define CHILD_WATER_SWITCH 2
#define CHILD_TIME_REMAINING 3

Bounce waterButtonBounce = Bounce();
Bounce cameraButtonBounce = Bounce();

int minutes;
//float pressureSamples[180];
int minuteCount = 0, lastCamera = 1;
bool firstRound = true, cameraOn = true;

#ifdef USE_RADIO
// Change to V_LIGHT if you use S_LIGHT in presentation below
MyMessage waterMessage(CHILD_WATER_SWITCH, V_LIGHT);
MyMessage cameraMessage(CHILD_CAMERA_SWITCH, V_LIGHT);
MyMessage timeRemainingMessage(CHILD_TIME_REMAINING, V_DISTANCE);
#endif

signed long timeRemaining = 0;
unsigned long lastReport = 0, time, buttonStart = 0, buttonFinish = 0, lastCheck = 0, lastReduce = 0;
bool changed = false;


void setup()
{
  pinMode(RELAY, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(CAMERA_LED, OUTPUT);
  pinMode(WATER_BUTTON, INPUT);
  pinMode(CAMERA_BUTTON, INPUT);
  digitalWrite(WATER_BUTTON, HIGH);
  digitalWrite(CAMERA_BUTTON, HIGH);
  digitalWrite(RELAY, LOW);
  cameraButtonBounce.attach(CAMERA_BUTTON);
  cameraButtonBounce.interval(5);
  waterButtonBounce.attach(WATER_BUTTON);
  waterButtonBounce.interval(5);
}

#ifdef USE_RADIO
void presentation () {
  sendSketchInfo("Water and camera control", "2.0");
  present(CHILD_WATER_SWITCH, S_LIGHT);
  present(CHILD_CAMERA_SWITCH, S_LIGHT);
  present(CHILD_TIME_REMAINING, S_DISTANCE);
}
#endif

void handleCameraButton() {
  int value = cameraButtonBounce.read();
  if (value != lastCamera) {
    if (value == 0) {
      cameraOn = !cameraOn;
      Serial.println(" Detected camera button");
      send(cameraMessage.set(cameraOn), true);
      digitalWrite(CAMERA_LED, cameraOn ? HIGH : LOW);
    }
    lastCamera = value;
  }
}

void handleWaterButton() {
  int value = waterButtonBounce.read();
  int remaining = 0, minutes = 0, lastMinutes = -1;
  bool report = true;
  if (value == 0 && buttonStart == 0) {
    time = millis();
    buttonStart = time;
    Serial.println(" Detected water bottle press");
    if (digitalRead(RELAY) == LOW) {
      buttonStart -= 2000;
    }
    do {
      delay(300);
      remaining = (millis() - buttonStart);
      if (remaining > 15000) {
        buttonStart = millis();
        remaining = (millis() - buttonStart);
      }
      minutes = remaining / ( 1000 * 2);
      if (minutes != lastMinutes) {
        report = true;
      }
      if (minutes >= 1) {
        if (report) {
          blinkLight((minutes) * 60000 - 1000);
          lastMinutes = minutes;
          report = false;
        }
      }
      waterButtonBounce.update();
    } while (waterButtonBounce.read() == 0);
    Serial.println(" detected water bottle release");

    //int remaining = millis() - buttonStart;
    buttonStart = 0; // Reset so that we can detect a new button press
    if (minutes >= 1) {
      timeRemaining = 60000 * minutes;
      changed = true;
      lastReduce = millis();
      //Serial.println(" Switching on time remaining:");
      //Serial.println(timeRemaining);
    } else {
      timeRemaining = 0;
      changed = true;
      lastReduce = millis();
      digitalWrite(LED, HIGH);
      delay(1000);
      digitalWrite(LED, LOW);
    }
    delay(1000);
  }
}
void loop()
{
  cameraButtonBounce.update();
  waterButtonBounce.update();
  handleWaterButton();
  handleCameraButton();
  time = millis();
  if (timeRemaining < 0) {
    timeRemaining = 0;
  }
  if ((long)(time - lastReduce) >= 0  || changed) {
    changed = false;

    //Serial.println(" Switching on time remaining:");
    //Serial.println(timeRemaining);
    if (timeRemaining > 0) {
      if (digitalRead(RELAY) == LOW) {
        digitalWrite(RELAY, HIGH);
#ifdef USE_RADIO
        send(waterMessage.set(true));
#endif
      }
      // Serial.println((timeRemaining));
#ifdef USE_RADIO
      send(timeRemainingMessage.set(timeRemaining / 60000.0, 2));
#endif
    } else {
      timeRemaining = 0;
      if (digitalRead(RELAY) == HIGH) {
        digitalWrite(RELAY, LOW);
        digitalWrite(LED, HIGH);
        delay(1000);
        digitalWrite(LED, LOW);
#ifdef USE_RADIO
        send(waterMessage.set(false));
        send(timeRemainingMessage.set(0));
#endif
        // Serial.println((timeRemaining));
      }



    }

    lastReduce = millis() + WATER_REPORT_INTERVAL;
    blinkLight(timeRemaining);
    timeRemaining -= WATER_REPORT_INTERVAL;//(time - lastReduce);
    if (timeRemaining < 0) {
      timeRemaining = 0;
    }

  }
  //}
  //  Serial.print("Time remaining : ");
  //  Serial.println(timeRemaining);

  lastCheck = time;

  if ((long)(millis() - lastReport) >= 0) {
#ifdef USE_RADIO
    send(timeRemainingMessage.set(timeRemaining / 60000.0, 1));
    send(waterMessage.set(digitalRead(RELAY) ? true : false));
    send(cameraMessage.set(cameraOn));
#endif
    lastReport = millis() + REPORT_INTERVAL;
  }
}

void blinkLight(signed long remaining) {
  int number = 1 + (remaining - 1000) / 60000;
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
  if (message.sensor == CHILD_WATER_SWITCH && message.type == V_LIGHT && strlen(message.getString()) != 0) {
    // Change relay state
    state = message.getBool();
    if (state) {
      timeRemaining = 480000;
    } else {
      timeRemaining = 0;
    }
    digitalWrite(RELAY, state ? LOW : HIGH);

  }
  if (message.sensor == CHILD_CAMERA_SWITCH && message.type == V_LIGHT && strlen(message.getString()) != 0) {
    cameraOn = message.getBool();
    digitalWrite(CAMERA_LED, cameraOn ? HIGH : LOW);
  }
}
#endif

