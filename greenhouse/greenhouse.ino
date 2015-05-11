#include <MySigningNone.h>
#include <MyTransportRFM69.h>
#include <MyTransportNRF24.h>
#include <MyHwATMega328.h>
#include <MySigningAtsha204Soft.h>
#include <MySigningAtsha204.h>

#include <MySensor.h>
#include <SPI.h>
#include <DHT.h>
#include <Bounce2.h>

#define DIGITAL_INPUT_SOIL_SENSOR 4
#define DIGITAL_INPUT_DOOR 3
#define DIGITAL_INPUT_WINDOW 6
#define DHTPIN 5     // what pin we're connected to

#define CHILD_TEMPERATURE 1
#define CHILD_HUMIDITY 2
#define CHILD_DOOR 3
#define CHILD_WINDOW 4
#define CHILD_DIGITAL_SOIL 5
#define CHILD_POWER 9



#define DHTTYPE DHT22   // DHT 22  (AM2302)

int BATTERY_SENSE_PIN  = A7;  // select the input pin for the battery sense point

MyTransportRFM69 transport;
// Hardware profile
MyHwATMega328 hw;

MySensor gw(transport, hw /*, signer*/);

Bounce debouncer[2];
byte oldValue[2] = {
  -1, -1};

// Change to V_LIGHT if you use S_LIGHT in presentation below
MyMessage temperatureMessage(CHILD_TEMPERATURE, V_TEMP);
MyMessage humidityMessage(CHILD_HUMIDITY, V_HUM);
MyMessage reed[2] = {MyMessage(CHILD_DOOR, V_TRIPPED),MyMessage(CHILD_WINDOW, V_TRIPPED)};
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
  gw.begin(NULL,7);
  gw.sendSketchInfo("Greenhouse", "1.0");
  delay(250);
  gw.present(CHILD_TEMPERATURE, S_TEMP);
  delay(250);
  gw.present(CHILD_HUMIDITY, S_HUM);
  delay(250);
  gw.present(CHILD_DOOR, S_DOOR);
  delay(250);
  gw.present(CHILD_WINDOW, S_DOOR);
  delay(250);
  gw.present(CHILD_DIGITAL_SOIL, S_MOTION);
  delay(250);
  gw.present(CHILD_POWER, S_POWER);
  
  
  debouncer[0] = Bounce();
  debouncer[1] = Bounce();
  // Setup the buttons and Activate internal pull-ups
  pinMode(DIGITAL_INPUT_DOOR,INPUT_PULLUP);
  pinMode(DIGITAL_INPUT_WINDOW, INPUT_PULLUP);
  /*
  debouncer[0].attach(DIGITAL_INPUT_DOOR);
  debouncer[1].attach(DIGITAL_INPUT_WINDOW);
  debouncer[0].interval(5);
  debouncer[1].interval(5);
  */
}


//  Check if digital input has changed and send in new value
void loop()
{
  // Get the update value

  gw.process();
  delay(2000);
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
      Serial.println(" oil ");
    Serial.println(soilValue);
    gw.send(soilMessage.set(soilValue == 0 ? 1 : 0)); // Send the inverse to gw as tripped should be when no water in soil
    lastSoilValue = soilValue;
  }

  float h = dht.readHumidity();
  if (h != lastHumidityValue && !isnan(h)) { // Read temperature as Celsius
    Serial.println(" humidity ");
    Serial.println(h);
    gw.send(humidityMessage.set(h, 1));
    lastHumidityValue = h;
  }
  
  float te = dht.readTemperature();
  if (te != lastTemperatureValue && !isnan(te)) { // Read temperature as Celsius
    Serial.println(" temperature ");
    Serial.println(te);
    gw.send(temperatureMessage.set(te, 1));
    lastTemperatureValue = te;
  }
/*
for (byte i = 0; i < 1; i++)
  {
    debouncer[i].update();
    int value = debouncer[i].read();
    if (value != oldValue[i]);
      Serial.println("door");
        Serial.println(value);
    gw.send(reed[i].set(value), false); 
    oldValue[i] = value;
  }
  */
  
  int door = digitalRead(DIGITAL_INPUT_DOOR);
  if (door != lastDoorValue) {
    gw.send(reed[0].set(door));
    lastDoorValue = door;
  }


  int window = digitalRead(DIGITAL_INPUT_WINDOW);
  if (window != lastWindowValue) {
    gw.send(reed[1].set(window));
    lastWindowValue = window;
  }

  gw.sleep(DIGITAL_INPUT_DOOR - 2, CHANGE, 900000);//DIGITAL_INPUT_WINDOW - 2, CHANGE,900000);
}

