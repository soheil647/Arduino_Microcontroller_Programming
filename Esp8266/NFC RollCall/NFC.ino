#include <Wire.h>
#include <NTPClient.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <time.h>
#include <ESP8266HTTPClient.h>
#include <Adafruit_PN532.h>

// If using the breakout with SPI, define the pins for SPI communication.
#define PN532_SCK  (14)
#define PN532_MOSI (13)
#define PN532_SS   (15)
#define PN532_MISO (12)


// If using the breakout or shield with I2C, define just the pins connected
// to the IRQ and reset lines.  Use the values below (2, 3) for the shield!
#define PN532_IRQ   (12)
#define PN532_RESET (15)  // Not connected by default on the NFC Shield

// Uncomment just _one_ line below depending on how your breakout or shield
// is connected to the Arduino:

// Use this line for a breakout with a SPI connection:
Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);

// Use this line for a breakout with a hardware SPI connection.  Note that
// the PN532 SCK, MOSI, and MISO pins need to be connected to the Arduino's
// hardware SPI SCK, MOSI, and MISO pins.  On an Arduino Uno these are
// SCK = 13, MOSI = 11, MISO = 12.  The SS line can be any digital IO pin.
//Adafruit_PN532 nfc(PN532_SS);

// Or use this line for a breakout or shield with an I2C connection:
//Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
// also change #define in Adafruit_PN532.cpp library file
   #define Serial SerialUSB
#endif

////Time Variables
//const char* ntpServer = "pool.ntp.org";
//const long  gmtOffset_sec = 12600;
//const int   daylightOffset_sec = 3600;
//const long utcOffsetInSeconds = 16200;
////Time_Saved
//int Time=0;
//int Time_S=1;
//int Time_M=1;
//int Time_H=1;

// Replace with your network credentials
const char* ssid     = "Sepantab";
const char* password = "alihfred";
//
//WiFiUDP ntpUDP;
//NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

void setup(void) {
  Serial.begin(115200);
  Serial.println("Hello!");
  
  //Wifi init
  Wifi_Connect_now();
  
   //Time Init
   //configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
   //timeClient.begin();

  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }
  
  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
  
  // Set the max number of retry attempts to read from a card
  // This prevents us from waiting forever for a card, which is
  // the default behaviour of the PN532.
  nfc.setPassiveActivationRetries(0xFF);
  
  // configure board to read RFID tags
  nfc.SAMConfig();
    
  Serial.println("Waiting for an ISO14443A card");
}

void loop(void) {
  boolean success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
  
  // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength);
  
  if (success) {
    Serial.println("Found a card!");
    Serial.print("UID Length: ");Serial.print(uidLength, DEC);Serial.println(" bytes");
    Serial.print("UID Value: ");
    String hex_value = "";
    for (uint8_t i=0; i < uidLength; i++) 
    {
      Serial.print(" 0x");Serial.print(uid[i], HEX);       
      //Serial.print(" ");Serial.print(uid[i], HEX);       
      hex_value += (String)uid[i];
    }

    Serial.println(", value="+hex_value);

    if(hex_value == "23173217171") {
      Serial.println("This is Soheil Key.");
      if (WiFi.status() != WL_CONNECTED) {
        POST_TEMP_DATA();
        
      }
      
    }
    else if(hex_value == "230522426") {
      Serial.println("This is Card Tag. ");
    }
    else if(hex_value == "63156295") {
      Serial.println("This is Phone Tag. ");
    }
    else
      Serial.println("I don't know.");

    Serial.println("");
    // Wait 1 second before continuing
    delay(1000);
  }
  else
  {
    // PN532 probably timed out waiting for a card
    Serial.println("Waiting for a card...");
  }
}
/*****************************************************************************************************/
void POST_TEMP_DATA() {
   HTTPClient http;  
   http.begin("http://scotech.ir/req/soheil/come");
   int httpCode = http.GET();   //Send the actual POST request
   if(httpCode>0){
    Serial.println("Request Complete ");
   }else{
    Serial.println("Error on sending request: ");
   }
   http.end();  //Free resources
}
/*******************************************Time****************************************************/
//void Get_Time(){
//   
//  if(WiFi.status() == WL_CONNECTED && Internet ) {
//    timeClient.update();
//    time_now.tm_hour = timeClient.getHours();
//    time_now.tm_min = timeClient.getMinutes();
//    time_now.tm_sec = timeClient.getSeconds();
//    
//  Serial.print(timeClient.getDay());
//  Serial.print(", ");
//  Serial.print(timeClient.getHours());
//  Serial.print(":");
//  Serial.print(timeClient.getMinutes());
//  Serial.print(":");
//  Serial.println(timeClient.getSeconds());     
//  Serial.println("NTP TIME");
//  } 
//  else {
//    if(Time>=5500*Time_S){
//      time_now.tm_sec = time_now.tm_sec + 11;
//    }
//    if(time_now.tm_sec >= 60){
//      time_now.tm_min = time_now.tm_min + 1;
//    }
//    if(time_now.tm_min >= 60){
//      time_now.tm_hour = time_now.tm_hour + 1;
//    }
//    if(time_now.tm_hour >= 23 && time_now.tm_min >= 60) Today = Today + 1;
//    if(time_now.tm_hour >= 24) time_now.tm_hour = time_now.tm_hour -24;
//    if(time_now.tm_min >= 60) time_now.tm_min = time_now.tm_min -60;
//    if(time_now.tm_sec >= 60) time_now.tm_sec = time_now.tm_sec -60;
//    Serial.print(time_now.tm_hour);
//    Serial.print(":");  
//    Serial.print(time_now.tm_min); 
//    Serial.print(":");  
//    Serial.println(time_now.tm_sec);
//    Serial.println("MY TIME");
//    Serial.println(Time);
//  } 
//}
/***********************************************************************************************/
void Wifi_Connect_now() {
  Serial.begin(115200);

  // We start by connecting to a WiFi network

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
     would try to act as both a client and an access-point and could cause
     network-issues with your other WiFi-devices on your WiFi-network. */
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
