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



void setup()
{
  gw.begin();
// Serial.begin(115200);
}


//  Check if digital input has changed and send in new value
void loop()
{
  // Get the update value

//  gw.process();

  Serial.println(" about to sleeffp" );
  gw.sleep(10000);
  Serial.println( " have slept " );
}


