// Simple binary switch example
// Connect button or door/window reed switch between
// digitial I/O pin 3 (BUTTON_PIN below) and GND.

#include <MySensor.h>
#include <RH_RF69.h>
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#define CHILD_TEMPERATURE 1
#define ONE_WIRE_BUS 13
RH_RF69 driver(10, 2);

//OneWire oneWire(ONE_WIRE_BUS);
//DallasTemperature sensors(&oneWire);

float lastTemperature  = 0;
long lastTemperatureSent = 0;

MySensor gw;
// Change to V_LIGHT if you use S_LIGHT in presentation below
MyMessage temperatureMessage(CHILD_TEMPERATURE, V_TEMP);


void setup()
{

  if (gw.setRadio(&driver)) {
    driver.setFrequency(868);
    driver.setTxPower(14);
    gw.begin();
  }
  // Setup the button
  pinMode(12, OUTPUT);
  // Activate internal pull-up
  digitalWrite(12, HIGH);
  //sensors.begin();
  // After setting up the button, setup debouncer
  //gw.sendSketchInfo("Temperature living room", "1.0");
  // Register binary input sensor to gw (they will be created as child devices)
  // You can use S_DOOR, S_MOTION or S_LIGHT here depending on your usage.
  // If S_LIGHT is used, remember to update variable type you send in. See "msg" above.
  //gw.present(CHILD_TEMPERATURE, S_TEMP);
}


//  Check if digital input has changed and send in new value
void loop()
{
  // Get the update value
  if (gw.process()) {
  }
  if (millis()  - lastTemperatureSent  > 1000) {
//    sensors.requestTemperatures();
//    delay(100);
    float temperature = 2.3;//static_cast<float>(sensors.getTempCByIndex(0));
    if (lastTemperature !=   temperature && temperature  != -127) {
      gw.send(temperatureMessage.set(temperature, 1));
  //    lastTemperature  = temperature - 1;
      lastTemperatureSent  = millis();
    }
  }
}


