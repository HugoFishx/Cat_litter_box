/*
 *  This sketch sends random data over UDP on a ESP32 device
 *
 */
#include <WiFi.h>
#include <WiFiUdp.h>

#define RXp2 3
#define TXp2 1

// WiFi network name and password:
const char * networkName = "Hugo_device";
const char * networkPswd = "19981222";

char read_buffer[255];

//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
const char * udpAddress = "192.168.0.136";
const int udpPort = 9999;

//Are we currently connected?
boolean connected = false;

//The udp library class
WiFiUDP udp;


void setup(){
  // Initilize hardware serial:
  Serial.begin(9600);
  // Connect to the WiFi network
  connectToWiFi(networkName, networkPswd);
}

void loop(){
  //only send data when connected
  if(connected && (Serial.available() > 0 )){
    //Send a packet
    String content = Serial.readString();
    if(content == "Init\r\n") {
      Serial.print("WiFi connected! IP address: ");
      Serial.println(WiFi.localIP()); 
      Serial.print("Server address: ");
      Serial.println(udpAddress);  
      return;
    }
    Serial.println(content);
    const char* msg = content.c_str();
    udp.beginPacket(udpAddress,udpPort);
    Serial.println(udp.write((const uint8_t*)msg, strlen(msg)));
    udp.endPacket();
  }
  int packetSize = udp.parsePacket();
  if(packetSize) {
    int len = udp.read(read_buffer, 255);
    if(len) {
      read_buffer[len] = 0;
    }
    Serial.println(read_buffer);
    if(read_buffer[0] == '1') {
      Serial.println("1");
    }
  }

  //Wait for 1 second
  delay(1000);
}

void connectToWiFi(const char * ssid, const char * pwd){
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);
  
  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.print(WiFi.localIP());
          Serial.print("Server address: ");
          Serial.println(udpAddress);  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          connected = true;
          break;
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          break;
      default: break;
    }
}
