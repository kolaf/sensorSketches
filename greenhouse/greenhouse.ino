ddefault :

#include <MySigningNone.h>
#include <MyTransportRFM69.h>
#include <MyTransportNRF24.h>
#include <MyHwATMega328.h>
#include <MySigningAtsha204Soft.h>
#include <MySigningAtsha204.h>

#include <MySensor.h>
#include <SPI.h>
#include <DHT.h>

#define DIGITAL_INPUT_SOIL_SENSOR 2
#define DIGITAL_INPUT_DOOR 3
#define DIGITAL_INPUT_WINDOW 4
#define DHTPIN 5     // what pin we're connected to

#define CHILD_TEMPERATURE 1
#define CHILD_HUMIDITY 2
#define CHILD_DOOR 3
#define CHILD_WINDOW 4
#define CHILD_DIGITAL_SOIL 5
#define CHILD_POWER 9



#define DHTTYPE DHT22   // DHT 22  (AM2302)

int BATTERY_SENSE_PIN  = A5;  // select the input pin for the battery sense point

MyTransportRFM69 transport;
// Hardware profile
MyHwATMega328 hw;

MySensor gw(transport, hw /*, signer*/);

// Change to V_LIGHT if you use S_LIGHT in presentation below
MyMessage temperatureMessage(CHILD_TEMPERATURE, V_TEMP);
MyMessage humidityMessage(CHILD_HUMIDITY, V_HUM);
MyMessage doorMessage(CHILD_DOOR, V_TRIPPED);
MyMessage windowMessage(CHILD_WINDOW, V_TRIPPED);
MyMessage soilMessage(CHILD_DIGITAL_SOIL, V_TRIPPED);
MyMessage powerMessage(CHILD_POWER, V_VOLTAGE);

DHT dht(DHTPIN, DHTTYPE);

int lastSoilValue = -1;
int lastTemperatureValue = -1;
int lastHumidityValue = -1;
int lastDoorValue = -1;
int lastWindowValue = -1;

void setup()
{
  analogReference(INTERNAL);
  gw.begin();
  gw.sendSketchInfo("Greenhouse", "1.0");

  gw.present(CHILD_TEMPERATURE, S_TEMP);
  gw.present(CHILD_HUMIDITY, S_HUM);
  gw.present(CHILD_DOOR, S_DOOR);
  gw.present(CHILD_WINDOW, S_DOOR);
  gw.present(CHILD_DIGITAL_SOIL, S_MOTION);
  gw.present(CHILD_POWER, S_POWER);
  
  digitalWrite(DIGITAL_INPUT_DOOR,HIGH);
  digitalWrite(DIGITAL_INPUT_WINDOW,HIGH);
}


//  Check if digital input has changed and send in new value
void loop()
{
  // Get the update value

  gw.process();
  //Power reading
  int sensorValue = analogRead(BATTERY_SENSE_PIN);
  Serial.print(sensorValue);
  int t  = 0;
  sensorValue = 0;
  for (int i = 0; i < 10; i++) {
    t = analogRead(BATTERY_SENSE_PIN);
    sensorValue += t;
    Serial.println(t);
  }

  float batteryV  = sensorValue / 10 * 0.00520430107;

  int batteryPcnt = sensorValue / 100;
  gw.send(powerMessage.set(batteryV, 1));
  gw.sendBatteryLevel(batteryPcnt);

  //  soil moisture
  int soilValue = digitalRead(DIGITAL_INPUT_SOIL_SENSOR); // 1 = Not triggered, 0 = In soil with water
  if (soilValue != lastSoilValue) {
    Serial.println(soilValue);
    gw.send(soilMessage.set(soilValue == 0 ? 1 : 0)); // Send the inverse to gw as tripped should be when no water in soil
    lastSoilValue = soilValue;
  }

  float h = dht.readHumidity();
  if (h != lastHumidityValue && !isnan(h)) { // Read temperature as Celsius
    Serial.println(h);
    gw.send(humidityMessage.set(h, 1));
    lastHumidityValue = h;
  }
  
  float te = dht.readTemperature();
  if (te != lastTemperatureValue && !isnan(te)) { // Read temperature as Celsius
    Serial.println(te);
    gw.send(temperatureMessage.set(te, 1));
    lastTemperatureValue = te;
  }

  int door = digitalRead(DIGITAL_INPUT_DOOR);
  if (door != lastDoorValue) {
    gw.send(doorMessage.set(door));
    lastDoorValue = door;
  }


  int window = digitalRead(DIGITAL_INPUT_WINDOW);
  if (window != lastWindowValue) {
    gw.send(windowMessage.set(door));
    lastWindowValue = window;
  }

  gw.sleep(900000);
}

