
// Simple binary switch example 
// Connect button or door/window reed switch between 
// digitial I/O pin 3 (BUTTON_PIN below) and GND.

#include <MySensor.h>
#include <RH_RF69.h>
#include <SPI.h>
#include <Bounce2.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Vcc.h>
const float VccExpected   = 3.0;
const float VccCorrection = 2.860/2.92;  // Measured Vcc by multimeter divided by reported Vcc
Vcc vcc(VccCorrection);

#define CHILD_ID 1
#define CHILD_TEMPERATURE 3
#define ONE_WIRE_BUS 7
#define BUTTON_PIN  4  // Arduino Digital I/O pin for button/reed switch
RH_RF69 driver(10, 2);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

float lastTemperature  = 0;
long lastTemperatureSent = 0;

MySensor gw;
Bounce debouncer = Bounce(); 
int oldValue=-1;
int x = 0;
// Change to V_LIGHT if you use S_LIGHT in presentation below
MyMessage msg(CHILD_ID,V_LIGHT);
MyMessage temperatureMessage(CHILD_TEMPERATURE, V_TEMP);
static int oldBatteryPcnt = 0;

void setup()  
{  
  
  if(gw.setRadio(&driver)) {
    driver.setFrequency(868);
    driver.setTxPower(14);
    gw.begin();
  }
 // Setup the button
  pinMode(BUTTON_PIN,INPUT);
  // Activate internal pull-up
  digitalWrite(BUTTON_PIN,HIGH);
  sensors.begin();
  // After setting up the button, setup debouncer
  gw.sendSketchInfo("Wave switch 3", "1.1");
  // Register binary input sensor to gw (they will be created as child devices)
  // You can use S_DOOR, S_MOTION or S_LIGHT here depending on your usage. 
  // If S_LIGHT is used, remember to update variable type you send in. See "msg" above.
  gw.present(CHILD_ID, S_LIGHT); 
  gw.present(CHILD_TEMPERATURE, S_TEMP);
  gw.request(2,V_LIGHT);
}


//  Check if digital input has changed and send in new value
void loop() 
{
  // Get the update value
  int value = digitalRead(BUTTON_PIN);
  if(gw.process()) {
    MyMessage message = gw.getLastMessage();
    if (message.sensor == 2)  {
      digitalWrite(9,atoi  (message.data));
      
    }
  }
  if (millis()  -lastTemperatureSent  >30000) {
    sensors.requestTemperatures();
    delay(100);
    float temperature = static_cast<float>(sensors.getTempCByIndex(0));
    if (lastTemperature!=   temperature && temperature  != -127) {
      gw.send(temperatureMessage.set(temperature,1));
      lastTemperature  = temperature;
      lastTemperatureSent  =millis();
    }
    int batteryPcnt = (int)vcc.Read_Perc(VccExpected);
    if (oldBatteryPcnt != batteryPcnt)
    {
        gw.sendBatteryLevel(batteryPcnt);
        oldBatteryPcnt = batteryPcnt;
    }
  }
    
    
  if (value != oldValue) {
    
   // Serial.println (value);
    // Send in the new value
     gw.send(msg.set(value==HIGH ? 0 : 1));
     oldValue = value;
     delay(1000);
  }
  
} 


