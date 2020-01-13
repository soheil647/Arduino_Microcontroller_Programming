#define strncmp_P(s1, s2, n) strncmp((s1), (s2), (n))
#include <ArduinoJson.h>

#define Number_Of_Relays    7


// Assign output variables to GPIO pins
const int R_GPIO[Number_Of_Relays] = {PA1, PA2, PA3, PA4, PA5, PA6, PA7};

//R1 Variables
int RStartHour[Number_Of_Relays][32][3] = {0};
int RStartMin[Number_Of_Relays][32][3] = {0};
int REndHour[Number_Of_Relays][32][3] = {0};
int REndMin[Number_Of_Relays][32][3] = {0};
bool emergency[Number_Of_Relays] = {false};

//HTTP for Relays
const String Http_TodayPlan_URL[Number_Of_Relays] = {"/server/new_plan/1", "/server/new_plan/2", "/server/new_plan/3", "/server/new_plan/4", "/server/new_plan/5", "/server/new_plan/6", "/server/new_plan/7"};
const String Http_NewPlan_URL[Number_Of_Relays] = {"/server/check_new_data/1", "/server/check_new_data/2", "/server/check_new_data/3", "/server/check_new_data/4", "/server/check_new_data/5", "/server/check_new_data/6", "/server/check_new_data/7"};


//Time Variables
int Today = 0;

bool HTTP_Connect = false;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(1000);
  Toggle_LED();
  Serial1.println("ss");
  delay(2000);

}

void loop() {

  for (int i = 0; i < 15; i++) {
    Serial1.println("/server/new_plan/eu12");
    delay(1000);
    //Serial1.println("server/new_plan/eu12");
    String temp0 = Serial1.readString();
    Serial.println(temp0);
    delay(1000);
  }


  delay(1000);

}
/************************************************GPIO*****************************************/
void Init_GPIO(void) {
  // Initialize the output variables as outputs
  for (int i = 0; i <= Number_Of_Relays - 1; i++) {
    pinMode(R_GPIO[i], OUTPUT);
    // Set outputs to LOW
    digitalWrite(R_GPIO[i], LOW);
  }
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, LOW);
}
/************************************************************************/
void Toggle_LED(void) {
  if (digitalRead(PC13) == LOW) {
    digitalWrite(PC13, HIGH);
  }
  else if (digitalRead(PC13) == HIGH) {
    digitalWrite(PC13, LOW);
  }
}
/************************************************************************/
void Init_Module(void) {
  //Reset Module  AT+Z
  //  Serial.print("AT+Z\n\r");
  //  Serial1.print("AT+Z\n\r");
  //  delay(1000);
  //    //Set Module Name  AT+MID=<MID>
  //    Serial.print("AT+MID=Sepantab_Smart\n\r");
  //    Serial1.print("AT+MID=Sepantab_Smart\n\r");
  //  delay(1000);
  //      //version OF Module AT+VER
  Serial.print("AT+VER\n\r");
  Serial1.print("AT+VER\n\r");
  delay(1000);
}
/************************************************************************/
void Start_ATMode(void) {
  String StartAT;
  Serial1.print("+");
  delay(300);
  Serial1.print("+");
  delay(300);
  Serial1.print("+");
  delay(300);
  Serial1.print("a");
  delay(300);

  delay(1000);
  Serial1.println("");
}
/************************************************************************/
//AP mode AT command setting mode:
void config_WiFi(void) {
  HTTP_Connect = false;
  //1Set the AP working mode  AT+WMODE=AP\n\r
  Serial.print("AT+WMODE=AP\n\r");
  Serial1.print("AT+WMODE=AP\n\r");
  delay(1000);
  //    //2Set the SSID of the AP, Password AT+WAP=SSID, Password\n\r
  //    Serial.print("AT+WAP=Sepantab_Smart,\n\r");
  //    Serial1.print("AT+WAP=Sepantab_Smart,\n\r");
  //  delay(1000);
  //      //3Set the AP channel AT+CHANNEL=num\n\r
  //      Serial.print("AT+CHANNEL=6\n\r");
  //      Serial1.print("AT+CHANNEL=6\n\r");
  //  delay(1000);
  //        //3Set the IP address of the AP AT+LANN=Ip, Mask\n\r
  //        Serial.print("AT+LANN=192.168.4.1, 255.255.255.0\n\r");
  //        Serial1.print("AT+LANN=192.168.4.1, 255.255.255.0\n\r");
  //  delay(1000);
  //          Serial.print("WiFi Connection Completed");
  //          HTTP_Connect = true;

}
/************************************************************************//************************************************************************/
void Config_Socket(void) {
  //Set Mode Of Socket AT+WKMOD=<Mode>\n\r
  Serial.println("AT+WKMOD=HTPC\n\r");
  Serial1.println("AT+WKMOD=HTPC\n\r");
}
/************************************************************************//************************************************************************/
void Check_WiFi(void) {
  //Check Status AT+WSLK
  Serial.println("AT+WSLK\n\r");
  Serial1.println("AT+WSLK\n\r");
}
/************************************************************************//************************************************************************/
String Config_HTTP(int Relay_Number, String Action) {
  String URL = "";
  if (Action == "new_plan")
    URL = Http_NewPlan_URL[Relay_Number];
  if (Action == "Check_Plan")
    URL = Http_TodayPlan_URL[Relay_Number];

  //Set Method Of UART0 AT+HTPTP=<Method>\n\r
  Serial.println("AT+HTPTP=GET\n\r");
  Serial1.println("AT+HTPTP=GET\n\r");
  String temp0 = Serial1.readString();
  if (temp0 == "+OK") {
    //Set Server Address OF UART0 AT+HTPSV=<Address>,<Port>
    Serial.println("AT+HTPSV=\"http://sepantabiotserver.ir\",80\n\r");
    Serial1.println("AT+HTPSV=\"http://sepantabiotserver.ir\",80\n\r");
    String temp1 = Serial1.readString();
    if (temp1 == "+OK") {
      //Set URL Of UART0 AT+HTPURL=<URL>\n\r
      Serial.print("AT+HTPURL="); Serial.print(URL); Serial.println("\n\r");
      Serial1.print("AT+HTPURL="); Serial1.print(URL); Serial1.println("\n\r");
      String temp2 = Serial1.readString();
      if (temp2 == "+OK") {
        String temp3 = Serial1.readString();
        Serial.println("HTTP Request Complete");
        return temp3;
      }
      else
        return "AT+HTPURL Failed";
    }
    else
      return "AT+HTPSV Failed";
  }
  else
    return "AT+HTPTP Failed";
}
/************************************************************************//************************************************************************/
bool RelayHaveNewPlan(int Relay_Number) {

  String payload = Config_HTTP(Relay_Number, "Check_Plan");
  Serial.println(payload);
  bool is_New = false;

  if (HTTP_Connect) {

    //HERE IS Parsing
    const size_t capacity = 500;
    StaticJsonDocument<capacity> doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
      Serial.println(error.c_str());
      return false;
    }
    else {
      Today = doc["day"];
      is_New = doc["new_plan"];
      emergency[Relay_Number] = doc["emergency"];
      return is_New;
    }
  }
  else {
    Serial.println("HTTP_Connect Failed");
    return false;
  }
}
/************************************************************************//************************************************************************/
void Relay_NewPlan(int Relay_Number) {

  String payload = Config_HTTP(Relay_Number, "new_plan");
  Serial.println(payload);

  if (HTTP_Connect) { //Check for the returning code

    //HERE IS Parsing
    const size_t capacity = 500;
    StaticJsonDocument<capacity> doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
      Serial.println(error.c_str());
      return;
    }
    else {
      for (int i = 1; i <= 31; i++) {
        for (int j = 0; j <= 4; j++) {
          RStartHour[Relay_Number][i][j] = doc[String(i)][j]["startHour"];
          RStartMin[Relay_Number][i][j]  = doc[String(i)][j]["startMinute"];
          REndHour[Relay_Number][i][j]   = doc[String(i)][j]["endHour"];
          REndMin[Relay_Number][i][j]    = doc[String(i)][j]["endMinute"];
        }
      }
    }
  }
}
/************************************************************************//************************************************************************/
