#include <MySensor.h>
#include <MyGateway.h>
#include <SPI.h>
#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 0
// Singleton instance of the radio driver

//RH_RF69 driver(15, 16); // For RF69 on PJRC breakout board with Teensy 3.1
// Class to manage message delivery and receipt, using the driver declared above
uint8_t x;
MySensor gw;
//RHReliableDatagram manager(driver, CLIENT_ADDRESS);
//RHReliableDatagram manager(driver, SERVER_ADDRESS);
void setup()
{
  Serial.begin(9600);
  gw.begin(NULL, SERVER_ADDRESS);
  Serial.println("inited");
}
uint8_t datas[] = "And hello back to you";
// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
void loop()
{
//  if (gw.manager->available())
//  {
//    // Wait for a message addressed to us from the client
//    uint8_t len = sizeof(buf);
//    uint8_t from;
//    if (gw.manager->recvfromAck(buf, &len, &from))
//    {
//      Serial.print("got request from : 0x");
//      Serial.print(from, HEX);
//      Serial.print(": ");
//      Serial.println((char*)buf);
    if (gw.process()) {
MyMessage message = gw.getLastMessage();
Serial.println  (message.sender);
//gw.serial (message);
    }
///gw.processRadioMessage();      

      // Send a reply back to the originator client
///    if (gw.manager->sendtoWait(datas, sizeof(datas), from) !=RH_ROUTER_ERROR_NONE) 
//        Serial.println("sendtoWait failed");
//He home//    }

}

