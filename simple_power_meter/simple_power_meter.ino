#include <MySigningNone.h>
#include <MyTransportRFM69.h>
#include <MyTransportNRF24.h>
#include <MyHwATMega328.h>
#include <MySigningAtsha204Soft.h>
#include <MySigningAtsha204.h>

#include <MySensor.h>
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#define  LED  6
#define CHILD_POWER 1
#define CHILD_TEMPERATURE 2
#define CHILD_POWERP 3
#define ONE_WIRE_BUS 7
#define PERIOD 300000
#define BLINK_PIN 4
#define BLINK_PINP 3
#define BUTTON_PIN  4  // Arduino Digital I/O pin for button/reed switch
//RH_RF69 driver(10, 2);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

float lastTemperature  = 0;
long lastTemperatureSent = 0;
unsigned long time;
int req = 0;


MyTransportRFM69 transport;
// Hardware profile
MyHwATMega328 hw;

MySensor gw(transport, hw /*, signer*/);


MyMessage powerMessage(CHILD_POWER, V_KWH);
//MyMessage powerMessageP(CHILD_POWERP, V_KWH);
MyMessage currentPowerMessage(CHILD_POWER, V_WATT);
//MyMessage currentPowerMessageP(CHILD_POWERP, V_WATT);
MyMessage temperatureMessage(CHILD_TEMPERATURE, V_TEMP);

static unsigned long last = 0, blink = 0, blinkP = 0, count = 0;
void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  digitalWrite(BLINK_PIN, HIGH);
  //digitalWrite(BLINK_PINP, HIGH);
  gw.begin();
  sensors.begin();
  // After setting up the button, setup debouncer
  gw.sendSketchInfo("Power sensor", "1.0");
  // Register binary input sensor to gw (they will be created as child devices)
  // You can use S_DOOR, S_MOTION or S_LIGHT here depending on your usage.
  // If S_LIGHT is used, remember to update variable type you send in. See "msg" above.
  gw.present(CHILD_POWER, S_POWER);
  //  gw.present(CHILD_POWERP, S_POWER);
  gw.present(CHILD_TEMPERATURE, S_TEMP);

}

void loop() {
  static boolean ledOn = false;
  int data = digitalRead(BLINK_PIN);
  //static boolean ledOnP = false;
  //  int dataP = digitalRead(BLINK_PINP);

  gw.process();

  /// Serial.println(data);
  ///delay (1000);

  time = millis();
  if (time - blink > 50) {
    if (ledOn && data == 1) {
      blink = time;
      ledOn = false;
    } else if (!ledOn && data == 0) {
      ledOn = true;
      blink = time;
      ledBlink();
    }
  }

  unsigned long interval = time - lastTemperatureSent;

  if (interval > PERIOD) { // 1+ sec passed
    sensors.requestTemperatures();
    req = 1;
  }
  if (req == 1 && interval > PERIOD  + 100) {
    req = 0;
    float temperature = static_cast<float>(sensors.getTempCByIndex(0));
    if (lastTemperature !=   temperature && temperature  != -127) {
      gw.send(temperatureMessage.set(temperature, 1));
      // tdebug
      lastTemperature  = temperature;
      lastTemperatureSent  = time;
    }

  }
  //  ledOn=!ledOn;
  //      ledBlink();
  /*
    if (millis() - blinkP > 80) {
      if (ledOnP && dataP == 1) {
        blinkP = millis();
        ledOnP = false;
      } else if (!ledOnP && dataP == 0) {
        ledOnP = true;
        blinkP = millis();
        ledBlinkP();
      }
    }
    */
}

void ledBlink() {
  static int nBlinks = 0;
  time = millis();
  unsigned long interval = time - last;
  digitalWrite(LED, !digitalRead(LED));
  nBlinks++;
  if (interval < 0) { // millis() overflow
    last = time;
    nBlinks = 0;
  } else if (interval > PERIOD) { // 1+ sec passed
    // Blinks are 1000 per kWh, or 1 Wh each
    // One hour has 3.6M milliseconds
    long watts = nBlinks * 1 * 3.6E6 / interval;
    gw.send(currentPowerMessage.set(watts, 1));
    gw.send(powerMessage.set((double)nBlinks / 1000.0, 4));
    ///wattSend(watts);
    last = time;
    nBlinks = 0;


  }
}
/*
void ledBlinkP() {
  static int nBlinks = 0;
  unsigned long time = millis();
  unsigned long interval = time - last;
  digitalWrite(LED, !digitalRead(LED));
  nBlinks++;
  if (interval < 0) { // millis() overflow
    last = time;
    nBlinks = 0;
  } else if (interval > PERIOD) { // 1+ sec passed
    // Blinks are 1000 per kWh, or 1 Wh each
    // One hour has 3.6M milliseconds
    long watts = nBlinks * 1 * 3.6E6 / interval;
    gw.send(currentPowerMessageP.set(watts, 1));
    gw.send(powerMessageP.set((double)nBlinks / 1000.0, 4));
    ///wattSend(watts);
    last = time;
    nBlinks = 0;

    sensors.requestTemperatures();
    delay(100);
    float temperature = static_cast<float>(sensors.getTempCByIndex(0));
    if (lastTemperature!=   temperature && temperature  != -127) {
      gw.send(temperatureMessage.set(temperature,1));
      lastTemperature  = temperature;
      lastTemperatureSent  =millis();
    }

  }
}
*/
/*
static void wattSend(long watts) {
    Packet_t packet;

    packet.lang = LANG_ELECTRICITY;
    packet.mesg = MESG_ELEC_CURRENT;
    packet.data = watts;
    rf12_easySend(&packet, sizeof packet);
}

*/
