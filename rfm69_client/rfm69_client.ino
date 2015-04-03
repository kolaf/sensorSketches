#include <MySensor.h>
#include <SPI.h>
#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 0
// Singleton instance of the radio driver

//RH_RF69 driver(15, 16); // For RF69 on PJRC breakout board with Teensy 3.1
// Class to manage message delivery and receipt, using the driver declared above
uint8_t x;
MySensor gw;
//RHReliableDatagram manager(driver, CLIENT_ADDRESS);
void setup()
{
  Serial.begin(9600);
 gw.begin(NULL, CLIENT_ADDRESS);
x = 'a';
}
uint8_t datas[] = "0Hello World!";
// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
void loop()
{
  Serial.println("Sending to rf69_reliable_datagram_server");
  // Send a message to manager_server
  datas[0] = x;
       gw.send(msg.set(x));
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (gw.manager->recvfromAckTimeout(buf, &len, 2000, &from))
    {
      Serial.print("got reply from : 0x");
      x++;
      if (x >'z')x  =  'a';
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);
    }
    else
    {
      Serial.println("No reply, is rf69_reliable_datagram_server running?");
    }
  
  delay(500);
}

