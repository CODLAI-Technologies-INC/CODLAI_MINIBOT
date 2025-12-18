/*
 * English: Wikipedia Search Example
 * Turkish: Wikipedia Arama Örneği
 */
#define USE_WIKIPEDIA
#include "MINIBOT.h"

MINIBOT minibot;

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

void setup() {
  minibot.serialStart(115200);
  minibot.begin();
  
  Serial.println("Wikipedia Search Example");

  minibot.wifiStartAndConnect(WIFI_SSID, WIFI_PASSWORD);
  
  if(minibot.wifiConnectionControl()) {
      Serial.println("Searching for 'Robot'...");
      String summary = minibot.getWikipedia("Robot", "en");
      Serial.println("Summary:");
      Serial.println(summary);
  }
}

void loop() {
  
}
