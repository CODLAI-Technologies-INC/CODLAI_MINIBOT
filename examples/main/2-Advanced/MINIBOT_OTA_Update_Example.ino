/*
  CODLAI MINIBOT - OTA Update Example (ESP8266)
  -------------------------------------------------
  TR: OTA kullanmak icin once WiFi baglantisi kurun ve loop icinde otaHandle() cagirin.
  EN: Connect to WiFi first and call otaHandle() continuously in loop.

  Not: USE_OTA ve USE_WIFI define'lari kutuphane dahil edilmeden once yazilmalidir.
*/

#define USE_WIFI
#define USE_OTA

#include <MINIBOT.h>

MINIBOT minibot;

const char *WIFI_SSID = "YOUR_WIFI_SSID";
const char *WIFI_PASS = "YOUR_WIFI_PASSWORD";

const char *OTA_HOST = "MINIBOT-OTA"; // Cihaz adi (opsiyonel)
const char *OTA_PASS = "1234";        // Sifre (varsayilan 1234)

void setup()
{
  minibot.serialStart(115200);
  minibot.begin();

  minibot.wifiStartAndConnect(WIFI_SSID, WIFI_PASS);
  minibot.otaBegin(OTA_HOST, OTA_PASS, 3232);
}

void loop()
{
  minibot.otaHandle();
  delay(10);
}
