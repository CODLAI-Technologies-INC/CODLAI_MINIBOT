/*
  CODLAI MINIBOT - NTP Time Advanced Example

  TR:
  - Bu örnek, NTP ile saat senkronizasyonunu gosterir.
  - WiFi baglantisi kurulduktan sonra `ntpBegin()` cagrilmalidir.
  - Ornek olarak son senkron zamanini EEPROM'a CRC'li record olarak kaydediyoruz.

  EN:
  - This example demonstrates time synchronization via NTP.
  - Call `ntpBegin()` AFTER connecting to WiFi.
  - As an example, we store the last sync time into EEPROM as a CRC-protected record.
*/

#define USE_WIFI
#include <MINIBOT.h>

MINIBOT minibot;

// TR/EN: Fill in your WiFi credentials
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

// TR/EN: Turkey is UTC+3, no DST (usually)
static const int TIMEZONE_HOURS = 3;

// TR/EN: EEPROM address where we store our record
static const int EEPROM_ADDR_LAST_SYNC = 200;

struct LastSyncRecord
{
  uint32_t lastEpoch;
};

void setup()
{
  minibot.serialStart(115200);
  delay(200);

  // TR/EN: Connect WiFi
  minibot.wifiStartAndConnect(WIFI_SSID, WIFI_PASS);

  if (!minibot.wifiConnectionControl())
  {
    Serial.println("[WiFi] Not connected");
    return;
  }

  // TR/EN: NTP setup (single-call, recommended)
  bool ok = minibot.ntpBegin(TIMEZONE_HOURS);
  Serial.println(ok ? "[NTP] Synced" : "[NTP] Sync failed");

  Serial.print("Epoch: ");
  Serial.println((unsigned long)minibot.ntpGetEpoch());

  Serial.print("DateTime: ");
  Serial.println(minibot.ntpGetDateTimeString());

  // TR/EN: Store last sync time as a CRC-protected record
  minibot.eepromBegin(512);

  LastSyncRecord out;
  out.lastEpoch = (uint32_t)minibot.ntpGetEpoch();

  bool w = minibot.eepromWriteRecord(EEPROM_ADDR_LAST_SYNC, (const uint8_t *)&out, (uint16_t)sizeof(out), 1);
  Serial.println(w ? "[EEPROM] Record written" : "[EEPROM] Record write failed");

  LastSyncRecord in;
  uint16_t len = 0;
  uint16_t ver = 0;
  bool r = minibot.eepromReadRecord(EEPROM_ADDR_LAST_SYNC, (uint8_t *)&in, (uint16_t)sizeof(in), &len, &ver);

  if (r)
  {
    Serial.print("[EEPROM] Record ok. ver=");
    Serial.print(ver);
    Serial.print(" len=");
    Serial.print(len);
    Serial.print(" lastEpoch=");
    Serial.println((unsigned long)in.lastEpoch);
  }
  else
  {
    Serial.println("[EEPROM] Record invalid (magic/version/len/crc mismatch)");
  }
}

void loop()
{
}
