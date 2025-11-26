/*
 * English: Weather Info Example
 * Turkish: Hava Durumu Bilgisi Örneği
 */
#define USE_WEATHER
#define USE_WIFI
#include "MINIBOT.h"

MINIBOT minibot;

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// OpenWeatherMap API Key (Optional, leave empty to use wttr.in)
#define API_KEY "" 
#define CITY "Istanbul"

void setup() {
  minibot.serialStart(115200);
  minibot.begin();
  
  Serial.println("Weather Info Example");

  minibot.wifiStartAndConnect(WIFI_SSID, WIFI_PASSWORD);
}

void loop() {
  if(minibot.wifiConnectionControl()) {
      String weather = minibot.getWeather(CITY, API_KEY);
      Serial.println("Weather in " + String(CITY) + ": " + weather);
  }
  delay(60000); // Update every minute
}
