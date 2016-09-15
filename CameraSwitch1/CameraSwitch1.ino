
// Enable debug prints to serial monitor
#define MY_DEBUG
#define USE_RADIO
// Enable and select radio type attached
#define MY_RADIO_RFM69
#define MY_IS_RFM69HW
#define MY_RFM69_FREQUENCY   RF69_867MHZ
//#define MY_RFM69_FREQUENCY   RF69_869MHZ
#define MY_NODE_ID 21
#define MY_REPEATER_FEATURE
#define MY_OTA_FIRMWARE_FEATURE
#define RELAY_ON 1  // GPIO value to write to turn on attached relay
#define RELAY_OFF 0 // GPIO value to write to turn off attached relay

#include <SPI.h>
#include <MySensors.h>

#define CHILD_RELAY 1
#define REPORT_INTERVAL 900000
#define RELAY 5
MyMessage relayMessage(CHILD_RELAY, V_LIGHT);

unsigned long lastReport = 0;


void setup()
{
  pinMode(RELAY, OUTPUT);
  //digitalWrite(RELAY, loadState(RELAY)?RELAY_ON:RELAY_OFF);
  digitalWrite(RELAY, RELAY_ON);
  send(relayMessage.set(digitalRead(RELAY) ? true : false));
}

void presentation () {
  sendSketchInfo("Camera relay 1", "2.0");
  present(CHILD_RELAY, S_LIGHT);
}

void loop()
{
  if ((long)(millis() - lastReport) >= 0) {
    send(relayMessage.set(digitalRead(RELAY) ? true : false));
    lastReport = millis() + REPORT_INTERVAL;
  }
}

void receive(const MyMessage & message) {
  // We only expect one type of message from controller. But we better check anyway.
  bool state;
  //saveState(message.sensor, message.getBool());
  if (message.sensor == CHILD_RELAY && message.type == V_LIGHT && strlen(message.getString()) != 0) {
    // Change relay state
    state = message.getBool();
    digitalWrite(RELAY, state ? RELAY_ON : RELAY_OFF);
  }
}

