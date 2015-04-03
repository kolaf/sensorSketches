// Simple binary switch example
// Connect button or door/window reed switch between
// digitial I/O pin 3 (BUTTON_PIN below) and GND.
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

#define CHILD_TEMPERATURE 1
#define CHILD_POWER 2
#define ONE_WIRE_BUS 7
int BATTERY_SENSE_PIN  = A5;  // select the input pin for the battery sense point

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

float lastTemperature  = 0;
long lastTemperatureSent = 0;

MyTransportRFM69 transport;
// Hardware profile 
MyHwATMega328 hw;

MySensor gw(transport, hw /*, signer*/);
// Change to V_LIGHT if you use S_LIGHT in presentation below
MyMessage temperatureMessage(CHILD_TEMPERATURE, V_TEMP);
MyMessage powerMessage(CHILD_POWER, V_VOLTAGE);


void setup()
{
  analogReference(INTERNAL);
  gw.begin();
  sensors.begin();
  // After setting up the button, setup debouncer
  gw.sendSketchInfo("Temperature stable", "1.0");
  // Register binary input sensor to gw (they will be created as child devices)
  // You can use S_DOOR, S_MOTION or S_LIGHT here depending on your usage.
  // If S_LIGHT is used, remember to update variable type you send in. See "msg" above.
  gw.present(CHILD_TEMPERATURE, S_TEMP);
  gw.present(CHILD_POWER, S_POWER);
}


//  Check if digital input has changed and send in new value
void loop()
{
  // Get the update value

  gw.process();

  int sensorValue = analogRead(BATTERY_SENSE_PIN);
  Serial.println(sensorValue);

  int t  = 0;
  sensorValue = 0;
  for(int i=0;i<10;i++){
    t = analogRead(BATTERY_SENSE_PIN);
    sensorValue += t;
  Serial.println(t);
  }
  
  float batteryV  = sensorValue/10 * 0.00520430107;
  
  int batteryPcnt = sensorValue / 100;
  gw.send(powerMessage.set(batteryV, 1));
  gw.sendBatteryLevel(batteryPcnt);
  sensors.requestTemperatures();
  delay(100);
  float temperature = static_cast<float>(sensors.getTempCByIndex(0));
  if (temperature  != -127) {
    gw.send(temperatureMessage.set(temperature, 1));
    //    lastTemperature  = temperature - 1;
  }
   gw.sleep(900000);
//   gw.sleep(10000);
}


