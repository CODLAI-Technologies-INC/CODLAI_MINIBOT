// TR: KABLOSUZ AKILLI EV FIKRI - OTOMATIK VANTILATOR. Bu MINIBOT'un kendi
// sicaklik sensoru YOK - bunun yerine, ayni odadaki bir IOTBOT'un
// yayinladigi (broadcast) DHT sicaklik verisini ESP-NOW ile dinler ve
// sicaklik yukselince role modulune bagli GERCEK bir vantilatoru/fani
// otomatik acar. Once IOTBOT_ESPNOW_Temperature_Broadcast_Example.ino
// dosyasini bir IOTBOT'a yukleyin, sonra bu kodu bir MINIBOT'a yukleyin -
// IOTBOT'un DHT sensorunu elinizle isitinca MINIBOT'un rolesi (ve
// baglıysa vantilatorunuz) otomatik acilacak!
// EN: A WIRELESS SMART HOME IDEA - AUTOMATIC FAN. This MINIBOT has NO
// temperature sensor of its own - instead, it listens over ESP-NOW to
// the DHT temperature data broadcast by an IOTBOT in the same room, and
// automatically turns on a REAL fan (wired to its relay module) when it
// gets hot. First upload IOTBOT_ESPNOW_Temperature_Broadcast_Example.ino
// to an IOTBOT, then upload this code to a MINIBOT - warm up the
// IOTBOT's DHT sensor with your hand and watch the MINIBOT's relay (and
// your fan, if wired) turn on!
//
// Baglanti / Wiring: Role modulunu P soketlerinden BIRINE takin ve
// asagidaki RELAY_PIN degerini o soketin sinyaline gore ayarlayin. /
// Plug the relay module into ONE of the P sockets and set RELAY_PIN
// below to match that socket's signal.

#define USE_ESPNOW
#include <MINIBOT.h>

MINIBOT minibot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define RELAY_PIN IO12 // Role modulunun bagli oldugu pin / Pin the relay module is connected to
// Desteklenen pinler: IO4 - IO5 - IO12 - IO13 - IO14
// Supported pins: IO4 - IO5 - IO12 - IO13 - IO14

// Bu esigin UZERINDEKI degerler "sicak" sayilir - ortaminiza gore ayarlayin.
// Values ABOVE this threshold count as "hot" - adjust to your environment.
constexpr int kHotThresholdC = 28;

namespace {
  bool fanOn = false;

  void say(const char* turkishText, const char* englishText) {
    minibot.serialWrite(turkish ? turkishText : englishText);
  }
}

void setup() {
  minibot.begin();
  minibot.serialStart(115200);
  minibot.initESPNow();
  minibot.startListening(); // Gelen IOTBOT yayinini minibot.receivedData'ya yazar.
  minibot.moduleRelayWrite(RELAY_PIN, false);

  say("Otomatik vantilator hazir - IOTBOT'tan sicaklik verisi bekleniyor...",
      "Automatic fan ready - waiting for temperature data from IOTBOT...");
}

void loop() {
  if (minibot.newData) {
    minibot.newData = false;

    // Sadece IOTBOT sicaklik yayinindan (deviceType 11) gelen veriyi kabul
    // ediyoruz. / Only treat data from an IOTBOT temperature broadcast
    // (deviceType 11) as a temperature reading.
    if (minibot.receivedData.deviceType == 11) {
      int tempC = minibot.receivedData.axis1;
      bool shouldBeOn = tempC > kHotThresholdC;

      if (shouldBeOn != fanOn) {
        fanOn = shouldBeOn;
        minibot.moduleRelayWrite(RELAY_PIN, fanOn);
        minibot.ledWrite(fanOn);
        Serial.print(turkish ? "Sicaklik: " : "Temperature: ");
        Serial.print(tempC);
        Serial.print(" C -> ");
        Serial.println(fanOn ? (turkish ? "VANTILATOR ACILDI (sicak)" : "FAN ON (hot)")
                              : (turkish ? "VANTILATOR KAPANDI (serin)" : "FAN OFF (cool)"));
      }
    }
  }
}
