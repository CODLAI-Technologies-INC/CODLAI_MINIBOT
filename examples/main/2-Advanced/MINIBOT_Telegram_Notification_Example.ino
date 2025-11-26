/*
 * English: Telegram Notification Example
 * Turkish: Telegram Bildirim Örneği
 */
#define USE_TELEGRAM
#define USE_WIFI
#include "MINIBOT.h"

MINIBOT minibot;

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// Telegram Bot Token (Get from @BotFather)
#define BOT_TOKEN "YOUR_BOT_TOKEN"
// Your Chat ID (Get from @userinfobot)
#define CHAT_ID "YOUR_CHAT_ID"

void setup() {
  minibot.serialStart(115200);
  minibot.begin();
  
  Serial.println("Telegram Notification Example");

  minibot.wifiStartAndConnect(WIFI_SSID, WIFI_PASSWORD);
}

void loop() {
  if(minibot.button1Read()) { // Assuming button1 is available
      Serial.println("Button Pressed! Sending Telegram message...");
      minibot.sendTelegram(BOT_TOKEN, CHAT_ID, "MINIBOT: Button Pressed!");
      delay(5000); // Debounce
  }
}
