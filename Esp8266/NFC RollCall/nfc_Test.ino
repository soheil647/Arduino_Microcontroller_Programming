
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <PN532_SPI.h>
#include <WiFiUdp.h>
#include <ESP8266HTTPClient.h>
#include <PN532.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>

//Number Cards
#define Number_Of_Cards 6

PN532_SPI pn532spi(SPI, 15);
PN532 nfc(pn532spi);
const char* ssid     = "";
const char* password = "";

//HTTP for Relays
String Http_Request_P[Number_Of_Cards] = {"http://scotech.ir/req/soheil", "http://scotech.ir/req/ali", "http://scotech.ir/req/mamad", "http://scotech.ir/req/hootan", "http://scotech.ir/req/ashkan", "http://scotech.ir/req/alireza"};

//Cards Hex Value
String Cards_ID[Number_Of_Cards] = {"23173217171", "210153160234", "66249163234", "19459153234", "828071234", "146153216234"};
String Names[Number_Of_Cards] = {"soheil", "ali", "mamad", "hootan", "ashkan", "alireza"};
int Status_ID[Number_Of_Cards] = {0};

String State = "";


void setup(void) {
  Serial.begin(115200);
  Serial.println("Hello!");

  Init_GPIO();

  //Wifi init
  Wifi_Connect();


  nfc.begin();


  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }

  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata >> 24) & 0xFF, HEX);
  Serial.print("Firmware ver. "); Serial.print((versiondata >> 16) & 0xFF, DEC);
  Serial.print('.'); Serial.println((versiondata >> 8) & 0xFF, DEC);

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
    Serial.print("UID Length: "); Serial.print(uidLength, DEC); Serial.println(" bytes");
    Serial.print("UID Value: ");
    String hex_value = "";
    for (uint8_t i = 0; i < uidLength; i++)
    {
      Serial.print(" 0x"); Serial.print(uid[i], HEX);
      //Serial.print(" ");Serial.print(uid[i], HEX);
      hex_value += (String)uid[i];
    }
    Serial.println(", value=" + hex_value);
    if (WiFi.status() != WL_CONNECTED) {
      Wifi_Try();
    }
    for (int i = 0; i < Number_Of_Cards; i++) {

      if (hex_value == Cards_ID[i]) {
        Serial.print("This Key Is For: ");
        Serial.println(Names[i]);
        Request_PA(i);
        CheckLED();
        break;
      }
      else
        Serial.println("I don't know.");
    }
  }
  else {
    // PN532 probably timed out waiting for a card
    Serial.println("Waiting for a card...");
  }
  delay(1000);
}
/*****************************************************************************************************/
void Request_PA(int ID_Number) {
  HTTPClient http;
  http.begin(Http_Request_P[ID_Number]);

  int httpCode = http.GET();
  if (httpCode > 0) {
    String Load = http.getString();
    Serial.println(Load);
    //HERE IS Parsing
    const size_t capacity = 500;
    StaticJsonDocument<capacity> doc;
    DeserializationError error = deserializeJson(doc, Load);
    if (error) {
      Serial.println(error.c_str());
      return;
    }
    else {
      String OPER = doc["action"];
      State = OPER;
      Serial.println(State);
    }
    Serial.println("Request Complete ");
  }
  else {
    Serial.println("Error on sending request: ");
    Blink_led();
  }
  http.end();  //Free resources
}
/*******************************************Time****************************************************/
void CheckLED(){
  if(State == "come"){
    Blink_led2();
  }
  if(State == "go"){
    Blink_led3();
  }
  if(State == "endDay"){
    Blink_led();
  }
}
/*******************************************Time****************************************************/
void Wifi_Connect_now() {
  Serial.begin(115200);
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
/***********************************************************************************************/
void Init_GPIO(void) {
  // Initialize the output variables as outputs
  pinMode(5, OUTPUT);
  // Set outputs to LOW
  digitalWrite(5, LOW);
}
void Blink_led(void) {
  digitalWrite(5, HIGH);
  delay(500);
  digitalWrite(5, LOW);
  delay(500);
  digitalWrite(5, HIGH);
  delay(500);
  digitalWrite(5, LOW);
  delay(500);
  digitalWrite(5, HIGH);
  delay(500);
  digitalWrite(5, LOW);
  delay(500);
  digitalWrite(5, HIGH);
  delay(500);
  digitalWrite(5, LOW);
  delay(500);
  digitalWrite(5, HIGH);
  delay(500);
  digitalWrite(5, LOW);
  delay(500);
}
void Blink_led2(void) {
  digitalWrite(5, HIGH);
  delay(1000);
  digitalWrite(5, LOW);
  delay(1000);
}
void Blink_led3(void) {
  digitalWrite(5, HIGH);
  delay(1000);
  digitalWrite(5, LOW);
  delay(1000);
  digitalWrite(5, HIGH);
  delay(1000);
  digitalWrite(5, LOW);
  delay(1000);
}
/*******************************************WIFI**************************************************/
void Wifi_Connect(void) {

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //reset settings - for testing
  //wifiManager.resetSettings();
  for (int i = 0; i <= 15; i++) {
    //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
    wifiManager.setAPCallback(configModeCallback);
    delay(1000);
  }

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("Smart_Sepantab")) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }
}
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}
/*************************************************************************************************/
void Wifi_Try(void) {
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //reset settings - for testing
  //wifiManager.resetSettings();

  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);

}
