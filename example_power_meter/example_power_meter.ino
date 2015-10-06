#include <MySigningNone.h>
#include <MyTransportRFM69.h>
#include <MyTransportNRF24.h>
#include <MyHwATMega328.h>
#include <MySigningAtsha204Soft.h>
#include <MySigningAtsha204.h>

#include <MySensor.h>
#include <SPI.h>

#define DIGITAL_INPUT_SENSOR 3  // The digital input you attached your light sensor.  (Only 2 and 3 generates interrupt!)
#define PULSE_FACTOR 1000       // Nummber of blinks per KWH of your meeter
#define SLEEP_MODE false        // Watt-value can only be reported when sleep mode is false.
#define MAX_WATT 10000          // Max watt value to report. This filetrs outliers.
#define INTERRUPT DIGITAL_INPUT_SENSOR-2 // Usually the interrupt = pin -2 (on uno/nano anyway)
#define CHILD_ID 1              // Id of the sensor child
unsigned long SEND_FREQUENCY = 120000; // Minimum time between send (in milliseconds). We don't wnat to spam the gateway.
double ppwh = ((double)PULSE_FACTOR) / 1000; // Pulses per watt hour
#define  LED  6
boolean pcReceived = true;
volatile unsigned long pulseCount = 0;
volatile unsigned long lastBlink = 0;
volatile unsigned long watt = 0;
unsigned long oldPulseCount = 0;
unsigned long oldWatt = 0;
double oldKwh;
unsigned long lastSend;
MyMessage wattMsg(CHILD_ID, V_WATT);
MyMessage kwhMsg(CHILD_ID, V_KWH);
//MyMessage pcMsg(CHILD_ID, V_VAR1);


MyTransportRFM69 transport;
// Hardware profile
MyHwATMega328 hw;

MySensor gw(transport, hw /*, signer*/);

void setup()
{
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  gw.begin();

  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("Energy Meter", "1.0");

  // Register this device as power sensor
  gw.present(CHILD_ID, S_POWER);

  // Fetch last known pulse count value from gw
  //  gw.request(CHILD_ID, V_VAR1);

  attachInterrupt(INTERRUPT, onPulse, RISING);
  lastSend = millis();
}

void loop()
{
  gw.process();
  unsigned long now = millis();
  // Only send values at a maximum frequency or woken up from sleep
  bool sendTime = now - lastSend > SEND_FREQUENCY;
  if (pcReceived && (SLEEP_MODE || sendTime)) {
    // New watt value has been calculated
    if (!SLEEP_MODE && watt != oldWatt) {
      // Check that we dont get unresonable large watt value.
      // could hapen when long wraps or false interrupt triggered
      if (watt < ((unsigned long)MAX_WATT)) {
        gw.send(wattMsg.set(watt));  // Send watt value to gw
      }
      Serial.print("Watt:");
      Serial.println(watt);
      oldWatt = watt;
    }

    // Pulse cout has changed
    if (pulseCount != oldPulseCount) {
//      gw.send(pcMsg.set(pulseCount));  // Send pulse count value to gw
      double kwh = ((double)pulseCount / ((double)PULSE_FACTOR));
      oldPulseCount = pulseCount;
      if (kwh != oldKwh) {
        gw.send(kwhMsg.set(kwh, 4));  // Send kwh value to gw
        oldKwh = kwh;
      }
    }
    lastSend = now;
  } else if (sendTime && !pcReceived) {
    // No count received. Try requesting it again
    gw.request(CHILD_ID, V_VAR1);
  }

  if (SLEEP_MODE) {
    gw.sleep(SEND_FREQUENCY);
  }
}

void incomingMessage(const MyMessage &message) {
  if (message.type == V_VAR1) {
    pulseCount = oldPulseCount = message.getLong();
    Serial.print("Received last pulse count from gw:");
    Serial.println(pulseCount);
    pcReceived = true;
  }
}

void onPulse()
{
  if (!SLEEP_MODE) {
    unsigned long newBlink = micros();
    unsigned long interval = newBlink - lastBlink;
    if (interval < 10000L) { // Sometimes we get interrupt on RISING
      return;
    }
    watt = (3600000000.0 / interval) / ppwh;
    lastBlink = newBlink;
    digitalWrite(LED, !digitalRead(LED));
  }
  pulseCount++;
}
