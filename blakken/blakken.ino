#include <MySigningNone.h>
#include <MyTransportRFM69.h>
#include <MyTransportNRF24.h>
#include <MyHwATMega328.h>
#include <MySigningAtsha204Soft.h>
#include <MySigningAtsha204.h>
#include <SPI.h>
#include <MySensor.h>

#define DIGITAL_INPUT_DOOR 3

#define CHILD_DOOR 1

MyTransportRFM69 transport;
// Hardware profile
MyHwATMega328 hw;

MySensor gw(transport, hw /*, signer*/);

MyMessage doorMessage(CHILD_DOOR, V_TRIPPED);
int lastDoorValue = -1;

void setup()
{
  gw.begin(NULL,8);
  gw.sendSketchInfo("Blakken", "1.0");
  delay(250);
  gw.present(CHILD_DOOR, S_DOOR);
  pinMode(DIGITAL_INPUT_DOOR,INPUT_PULLUP);
}


//  Check if digital input has changed and send in new value
void loop()
{
  // Get the update value

  gw.process();
  
  int door = digitalRead(DIGITAL_INPUT_DOOR);
  if (door != lastDoorValue) {
    gw.send(doorMessage.set(door == HIGH?LOW:HIGH));
    lastDoorValue = door;
  }
  gw.sleep(DIGITAL_INPUT_DOOR - 2, CHANGE, 3600000);
}

