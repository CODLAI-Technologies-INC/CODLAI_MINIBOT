/*
 * English: Email Sender Example
 * Turkish: E-posta Gönderici Örneği
 */
#define USE_EMAIL
#define USE_WIFI
#include "MINIBOT.h"

MINIBOT minibot;

// WiFi Credentials
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// Email Credentials
#define SMTP_HOST "smtp.gmail.com"
#define SMTP_PORT 465
#define AUTHOR_EMAIL "YOUR_EMAIL@gmail.com"
#define AUTHOR_PASSWORD "YOUR_APP_PASSWORD"
#define RECIPIENT_EMAIL "RECIPIENT_EMAIL@example.com"

void setup() {
  minibot.serialStart(115200);
  minibot.begin();
  
  Serial.println("Email Sender Example");

  minibot.wifiStartAndConnect(WIFI_SSID, WIFI_PASSWORD);
  
  if(minibot.wifiConnectionControl()) {
      Serial.println("Sending Email...");
      minibot.sendEmail(SMTP_HOST, SMTP_PORT, AUTHOR_EMAIL, AUTHOR_PASSWORD, RECIPIENT_EMAIL, "Test Subject", "Hello from MINIBOT!");
  }
}

void loop() {
  
}
