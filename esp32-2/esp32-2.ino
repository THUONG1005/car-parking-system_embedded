
#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>
#include <HardwareSerial.h>
#include <esp32/rom/rtc.h>
#include <LiquidCrystal_I2C.h>
//#include <WiFiClient.h>
#include <SPI.h>
#include <MFRC522.h>
#//include <bits/stdc++.h>
using namespace std;
// #define DEBUG false
// #define DEBUG true

#define SS_PIN 5    // ESP32 pin GIOP5
#define RST_PIN 27
#define LED_BUILTIN 2  // ESP32 pin GIOP27

MFRC522 rfid(SS_PIN, RST_PIN);

const int DIR = 12;  //(direction)
const int STEP = 14;
const int steps_per_rev = 200;
const int speaker = 26;

const int cbMain = 34;
const int cb1 = 25;
const int cb2 = 33;
const int cb3 = 32;

int location_current = 10;
int location = 10;
int a0 = 0;
int a1 = 0;
int a2 = 0;

char cardID[9];

LiquidCrystal_I2C lcd(0x27, 16, 2);

/* Wifi */
//const char *wifiName = "B3 206 2.4G";
//const char *wifiPassword = "b3206k64";

const char *wifiName = "Nguyen Van Son";
const char *wifiPassword = "son1692005";
//
//const char *wifiName  = "Viettel Truong Tinh";
//const char *wifiPassword = "123456789";

/* System server */
const String serverPath = "https://vizigtalk.online:3002/api";
const String loginPath = serverPath + "/auth/login";
const String logoutPath = serverPath + "/auth/logout";
const String getLocationPath = serverPath + "/parking/get-location-by-card-code/";
const String getAccessTokenPath = serverPath + "/auth/get-access-token";
const String getRefreshTokenPath = serverPath + "/auth/get-refresh-token";

/* System account */
//const String gateEmail = "thuong@gmail.com";
//const String gatePassword = "admin";

const String gateEmail = "admin@gmail.com";
const String gatePassword = "Binh542@";

/* Auth */
String gAccessToken = "";
String gRefreshToken = "";
String stringGetLocationRequest = " ";
unsigned int gAccessTokenExpiredIn = 0;
unsigned int gRefreshTokenExpiredIn = 0;

/*  */
String loginRequest(void);
String logoutRequest(void);
String getLocationRequest(String cardCode);
String getAccessTokenRequest(void);
String getRefreshTokenRequest(void);
void connectToWifi(void);
void login(void);
void toggleLed(unsigned int delayInterval);
void printRequestLog(String title, String input, int code, String response);
void printHTTPCode(int code);
void printResetReason(RESET_REASON resetReason);


void IRAM_ATTR isr() {
  a0 = digitalRead(cb1);  //value sensor 1
  a1 = digitalRead(cb2);  //value sensor 2
  a2 = digitalRead(cb3);  //value sensor 3
  location_current = 7 - (a0 * 4 + a1 * 2 + a2);
}

void setup() {
  Serial.begin(115200);

//  Serial2.setRxBufferSize(1024);
//  Serial2.setTimeout(50);
//  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  Serial.print("\nCPU0 reset reason: ");
  printResetReason(rtc_get_reset_reason(0));
  Serial.print("\nCPU1 reset reason: ");
  printResetReason(rtc_get_reset_reason(1));

  pinMode(LED_BUILTIN, OUTPUT);

  connectToWifi();

  login();

  lcd.init();
  lcd.backlight();
  

  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(cbMain, INPUT_PULLUP);
  pinMode(speaker, OUTPUT);

  digitalWrite(speaker, HIGH);

  pinMode(cb1, INPUT);
  pinMode(cb2, INPUT);
  pinMode(cb3, INPUT);

  SPI.begin();      // init SPI bus
  rfid.PCD_Init();  // init MFRC522

  attachInterrupt(cbMain, isr, FALLING);

  // cardID[4][0] = 0x03;
  // cardID[4][1] = 0xAF;
  // cardID[4][2] = 0xF8;
  // cardID[4][3] = 0xAD;

  // cardID[2][0] = 0x73;
  // cardID[2][1] = 0x86;
  // cardID[2][2] = 0x7A;
  // cardID[2][3] = 0xA7;

  lcd.setCursor(0, 0);
  lcd.print("  PLEASE CHECK  ");

  // stringGetLocationRequest = getLocationRequest("53C0E1AD");
  // if (stringGetLocationRequest != "") {
  //   JSONVar jsonGetLocationRequest = JSON.parse(stringGetLocationRequest);

  //   int statusCode = jsonGetLocationRequest["statusCode"];
  //   if (statusCode == 200) {
  //     const char *station = jsonGetLocationRequest["data"]["slot"]["station"];
  //     unsigned int position = jsonGetLocationRequest["data"]["slot"]["position"];

  //     Serial.print("station: ");
  //     Serial.println(station);
  //     Serial.print("position: ");
  //     Serial.print(position);
  //   } else {
  //     Serial.print("Loi cmnr!");
  //   }
  // }
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    toggleLed(300);
    if (rfid.PICC_IsNewCardPresent()) {  // new tag is available
      if (rfid.PICC_ReadCardSerial()) {  // NUID has been readed
        MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
        digitalWrite(speaker, LOW);
        delay(200);
        digitalWrite(speaker, HIGH);
        Serial.print("RFID/NFC Tag Type: ");
        Serial.println(rfid.PICC_GetTypeName(piccType));

        // print UID in Serial Monitor in the hex format
        Serial.print("UID:");
        for (int i = 0; i < rfid.uid.size; i++) {
          sprintf(cardID+i*2, "%02x", rfid.uid.uidByte[i]);
        }
        for (int i = 0; i < 8; i++){
          if(cardID[i]>96&&cardID[i]<123){
            cardID[i]-=32;
          }
        }
        Serial.println();
        stringGetLocationRequest = getLocationRequest((char*)cardID);
        for(int i = 0; i++ ; i< 700){
          delay(10);
          if(stringGetLocationRequest != ""){
            break;
          }
        }
        if (stringGetLocationRequest != "") {
          JSONVar jsonGetLocationRequest = JSON.parse(stringGetLocationRequest);

          int statusCode = jsonGetLocationRequest["statusCode"];
          if (statusCode == HTTP_CODE_OK) {
            const char *station = jsonGetLocationRequest["data"]["slot"]["station"];
            unsigned int position = jsonGetLocationRequest["data"]["slot"]["position"];

            Serial.print("station: ");
            Serial.println(station);
            Serial.print("position: ");
            Serial.print(position);

            location = position;
          } else {
            Serial.print("Loi");
          }
        }
        if (location == 10) {
          lcd.setCursor(0, 0);
          lcd.print("    NOT FOUND    ");
          delay(3000);
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("  PLEASE CHECK  ");
        }
        if (location != 10) {
          lcd.setCursor(0, 0);
          lcd.print("  PLEASE WAIT!  ");
        }
        rfid.PICC_HaltA();       // halt PICC
        rfid.PCD_StopCrypto1();  // stop encryption on PCD
      }
    }
    if (location != 10) {
      if (location < 4) {
        digitalWrite(DIR, HIGH);
        while (location != location_current) {
          digitalWrite(STEP, HIGH);
          delayMicroseconds(20000);
          digitalWrite(STEP, LOW);
          delayMicroseconds(20000);
        }
      }
      if (location >= 4) {
        digitalWrite(DIR, LOW);
        while (location != location_current) {
          digitalWrite(STEP, HIGH);
          delayMicroseconds(20000);
          digitalWrite(STEP, LOW);
          delayMicroseconds(20000);
        }
      }
      location = 10;
      lcd.setCursor(0, 0);
      lcd.print("  PLEASE CHECK  ");
      delay(10);
    }
  } else {
    digitalWrite(LED_BUILTIN, HIGH);

    connectToWifi();
  }
}

void connectToWifi(void) {
  Serial.printf("\nConnecting to Wifi: %s...............\n", wifiName);

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiName, wifiPassword);

  int tryOnTimes = 5;
  do {
    unsigned long lastTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - lastTime < 10000) {
      toggleLed(100);
      tryOnTimes--;
    }
  } while (WiFi.status() != WL_CONNECTED && tryOnTimes);
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nConnected to Wifi failed!");

    ESP.restart();
  }
if (WiFi.status() == WL_CONNECTED) {
  Serial.println("\nConnected to Wifi successfully!");
  Serial.printf("\nIP address: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("\nRRSI: %i\n", WiFi.RSSI());
  
}
}
void login() {
  int maxAttempts = 5;
  int attempts = 0;

  while (attempts < maxAttempts) {
    String stringLoginResult = loginRequest();
    if (stringLoginResult != "") {
      JSONVar jsonLoginResult = JSON.parse(stringLoginResult);

      int statusCode = jsonLoginResult["statusCode"];
      if (statusCode == HTTP_CODE_OK) {  // Thành công
        String accessToken = jsonLoginResult["data"]["tokens"]["accessToken"]["value"];
        gAccessTokenExpiredIn = jsonLoginResult["data"]["tokens"]["accessToken"]["expiredIn"];
        String refreshToken = jsonLoginResult["data"]["tokens"]["refreshToken"]["value"];
        gRefreshTokenExpiredIn = jsonLoginResult["data"]["tokens"]["refreshToken"]["expiredIn"];
        gAccessToken = accessToken;
        gRefreshToken = refreshToken;
        Serial.println("Login successful!");
        return;  // Exit the function if login is successful
      } else if (statusCode == HTTP_CODE_UNAUTHORIZED)  // Xem lại đăng nhập
      {
        Serial.println("Login unsuccessful. Retrying...");
      } else {
        Serial.print("Unexpected response code: ");
        Serial.println(statusCode);
        delay(1000);  // Wait before retrying
      }
    } else {
      Serial.println("Failed to connect to the server. Retrying...");
    }

    attempts++;
    delay(1000);  // Wait before the next attempt
  }

  Serial.println("Max login attempts reached. Exiting program.");
  ESP.restart();  // Restart the ESP32 if login fails after multiple attempts
}

void toggleLed(unsigned int delayInterval) {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  delay(delayInterval);
}

String loginRequest(void) {
  HTTPClient http;
  int responseCode;
  int tryOnTimes = 5;
  JSONVar jsonBody;
  jsonBody["email"] = gateEmail;
  jsonBody["password"] = gatePassword;
  String stringBody = JSON.stringify(jsonBody);

  http.begin(loginPath);
  http.addHeader("Content-Type", "application/json");

  do {
    responseCode = http.PUT(stringBody);
   if (responseCode >= 0) break;
    tryOnTimes--;
  } while (tryOnTimes > 0);
  if (responseCode < 0) {
    Serial.println("Failed to connect or request unsuccessful loginRequest");
    return "";
  }
  String responseBody = http.getString();
  http.end();

  printRequestLog("LOGIN_REQUEST", stringBody, responseCode, responseBody);

  return responseBody;
}

String logoutRequest(void) {
  HTTPClient http;
  int responseCode;
  int tryOnTimes = 5;

  http.begin(logoutPath);
  http.addHeader("Authorization", "Bearer " + gAccessToken);

  do {
    responseCode = http.GET();
    if (responseCode >= 0) break;
     tryOnTimes--;
  } while (tryOnTimes > 0);

  if (responseCode < 0) {
    Serial.println("Failed to connect or request unsuccessful logoutRequest");
    return "";
  }

  String responseBody = http.getString();
  http.end();

  printRequestLog("LOGOUT_REQUEST", "accessToken: " + gAccessToken, responseCode, responseBody);

  return responseBody;
}

String getLocationRequest(String cardCode) {
  HTTPClient http;
  int responseCode;
  int tryOnTimes = 5;

  http.begin(getLocationPath + cardCode);
  http.addHeader("Authorization", "Bearer " + gAccessToken);

  do {
    responseCode = http.GET();
    if (responseCode >= 0) break;
    tryOnTimes--;
  } while (tryOnTimes > 0);

  if (responseCode < 0) {
    Serial.println("Failed to connect or request unsuccessful getLocationRequest");
    return "";
  }
   
  String responseBody = http.getString();
  http.end();

  printRequestLog("\nGET_LOCATION_REQUEST", "accessToken: " + gAccessToken + "\ncardCode: " + cardCode, responseCode, responseBody);

  return responseBody;
}

String getAccessTokenRequest(void) {
  HTTPClient http;
  int responseCode;
  int tryOnTimes = 5;

  http.begin(getAccessTokenPath);
  http.addHeader("Authorization", "Bearer " + gRefreshToken);

  do {
    responseCode = http.GET();
    if (responseCode >= 0) break;
    tryOnTimes--;
  } while (tryOnTimes > 0);

  if (responseCode < 0) {
    Serial.println("Failed to connect or request unsuccessful getAccessTokenRequest");
    return "";
  }

  String responseBody = http.getString();
  http.end();

  printRequestLog("GET_ACCESS_TOKEN_REQUEST", "refreshToken: " + gRefreshToken, responseCode, responseBody);

  return responseBody;
}

String getRefreshTokenRequest(void) {
  HTTPClient http;
  int responseCode;
  int tryOnTimes = 5;

  http.begin(getRefreshTokenPath);
  http.addHeader("Authorization", "Bearer " + gRefreshToken);

  do {
    responseCode = http.GET();
   if (responseCode >= 0) break;
    tryOnTimes--;
  } while (tryOnTimes > 0);

  if (responseCode < 0) {
    Serial.println("Failed to connect or request unsuccessful getRefreshTokenRequest");
    return "";
  }

  String responseBody = http.getString();
  http.end();

  printRequestLog("GET_REFRESH_TOKEN_REQUEST", "refreshToken: " + gRefreshToken, responseCode, responseBody);

  return responseBody;
}

void printRequestLog(String title, String input, int code, String response) {
  Serial.println("\n================" + title + "================");
  Serial.println("\nInput:");
  Serial.println(input);
  Serial.println("\nCode:");
  Serial.print(code);
  Serial.print(" - ");
  printHTTPCode(code);
  Serial.println("\nResponse:");
  Serial.println(response);
  Serial.println("\n=============================================");
}

void printHTTPCode(int code) {
  switch (code) {
    case -1: Serial.println("CONNECTION_REFUSED"); break;
    case -2: Serial.println("SEND_HEADER_FAILED"); break;
    case -3: Serial.println("SEND_PAYLOAD_FAILED"); break;
    case -4: Serial.println("NOT_CONNECTED"); break;
    case -5: Serial.println("CONNECTION_LOST"); break;
    case -6: Serial.println("NO_STREAM"); break;
    case -7: Serial.println("NO_HTTP_SERVER"); break;
    case -8: Serial.println("TOO_LESS_RAM"); break;
    case -9: Serial.println("ENCODING"); break;
    case -10: Serial.println("STREAM_WRITE"); break;
    case -11: Serial.println("READ_TIMEOUT"); break;
    case 200: Serial.println("OK"); break;
    case 201: Serial.println("CREATED"); break;
    case 400: Serial.println("BAD_REQUEST"); break;
    case 401: Serial.println("UNAUTHORIZED"); break;
    case 403: Serial.println("FORBIDDEN"); break;
    case 404: Serial.println("NOT_FOUND"); break;
    case 500: Serial.println("INTERNAL_SERVER_ERROR"); break;
  }
}

void printResetReason(RESET_REASON resetReason) {
  switch (resetReason) {
    case 1: Serial.println("POWERON_RESET"); break;           /**<1,  Vbat power on reset*/
    case 3: Serial.println("SW_RESET"); break;                /**<3,  Software reset digital core*/
    case 4: Serial.println("OWDT_RESET"); break;              /**<4,  Legacy watch dog reset digital core*/
    case 5: Serial.println("DEEPSLEEP_RESET"); break;         /**<5,  Deep Sleep reset digital core*/
    case 6: Serial.println("SDIO_RESET"); break;              /**<6,  Reset by SLC module, reset digital core*/
    case 7: Serial.println("TG0WDT_SYS_RESET"); break;        /**<7,  Timer Group0 Watch dog reset digital core*/
    case 8: Serial.println("TG1WDT_SYS_RESET"); break;        /**<8,  Timer Group1 Watch dog reset digital core*/
    case 9: Serial.println("RTCWDT_SYS_RESET"); break;        /**<9,  RTC Watch dog Reset digital core*/
    case 10: Serial.println("INTRUSION_RESET"); break;        /**<10, Instrusion tested to reset CPU*/
    case 11: Serial.println("TGWDT_CPU_RESET"); break;        /**<11, Time Group reset CPU*/
    case 12: Serial.println("SW_CPU_RESET"); break;           /**<12, Software reset CPU*/
    case 13: Serial.println("RTCWDT_CPU_RESET"); break;       /**<13, RTC Watch dog Reset CPU*/
    case 14: Serial.println("EXT_CPU_RESET"); break;          /**<14, for APP CPU, reseted by PRO CPU*/
    case 15: Serial.println("RTCWDT_BROWN_OUT_RESET"); break; /**<15, Reset when the vdd voltage is not stable*/
    case 16: Serial.println("RTCWDT_RTC_RESET"); break;       /**<16, RTC Watch dog reset digital core and rtc module*/
    default: Serial.println("NO_MEAN");
  }
}
