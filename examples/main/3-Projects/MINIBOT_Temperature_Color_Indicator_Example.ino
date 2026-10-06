// TR: GERCEK PROJE - Renkli Sicaklik Gostergesi. DHT sensoru odanin
// sicakligini olcer, akilli LED'ler de sicakligi RENK ile gosterir: soguksa
// MAVI, rahatsa YESIL, sicaksa KIRMIZI - aradaki degerlerde renkler yavasca
// birbirine karisir. Sicaklik uyari sinirini gecerse buzzer her saniye
// uyari sesi verir. Her 2 saniyede bir Seri Port'a sicaklik, nem ve renk
// raporu yazilir. Sensoru elinizle isitip renklerin degisimini izleyin!
// EN: A REAL PROJECT - Color Temperature Indicator. The DHT sensor measures
// the room temperature and the smart LEDs show it as a COLOR: BLUE when
// cold, GREEN when comfortable, RED when hot - in between, the colors
// blend smoothly. If the temperature passes the warning limit, the buzzer
// beeps every second. Every 2 seconds a temperature, humidity and color
// report is printed to Serial. Warm the sensor with your hand and watch
// the colors change!
//
// Baglanti / Wiring: DHT sensorunu IO12'ye, akilli LED modulunu Port A'ya
// (IO14) takin. / Plug the DHT sensor into IO12 and the smart LED module
// into Port A (IO14).

#define USE_DHT
#define USE_NEOPIXEL
#include <MINIBOT.h>

MINIBOT minibot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define DHT_PIN IO12       // DHT sensoru / DHT sensor
#define SMART_LED_PIN IO14 // Port A

namespace {
  constexpr int kColdC = 18;              // Bu ve alti tam MAVI / at or below: full BLUE
  constexpr int kComfortC = 24;           // Tam YESIL / full GREEN
  constexpr int kHotC = 30;               // Bu ve ustu tam KIRMIZI / at or above: full RED
  constexpr int kWarningC = 32;           // Bunun ustunde buzzer uyarisi / above this: buzzer warning
  constexpr uint32_t kReadEveryMs = 2000; // DHT11 yavas bir sensordur / DHT11 is a slow sensor
  constexpr uint32_t kBeepEveryMs = 1000;

  int lastTempC = -999;                   // -999 = okunamadi / could not read
  uint32_t lastReadMs = 0, lastBeepMs = 0;

  // Sicakligi renge cevir: mavi -> yesil -> kirmizi. / Turn a temperature into a color: blue -> green -> red.
  void tempToColor(int t, int &r, int &g, int &b) {
    t = constrain(t, kColdC, kHotC);
    if (t <= kComfortC) {                  // Mavi -> yesil / blue -> green
      g = map(t, kColdC, kComfortC, 0, 255);
      b = 255 - g;
      r = 0;
    } else {                               // Yesil -> kirmizi / green -> red
      r = map(t, kComfortC, kHotC, 0, 255);
      g = 255 - r;
      b = 0;
    }
  }

  const char *colorName(int t) {
    if (t <= kColdC + 2) return turkish ? "MAVI (soguk)" : "BLUE (cold)";
    if (t < kComfortC - 1) return turkish ? "MAVI-YESIL (serin)" : "BLUE-GREEN (cool)";
    if (t <= kComfortC + 1) return turkish ? "YESIL (rahat)" : "GREEN (comfortable)";
    if (t < kHotC - 1) return turkish ? "SARI-TURUNCU (ilik)" : "YELLOW-ORANGE (warm)";
    return turkish ? "KIRMIZI (sicak)" : "RED (hot)";
  }
}

void setup() {
  minibot.begin();
  minibot.serialStart(115200);
  minibot.moduleSmartLEDPrepare(SMART_LED_PIN);
  minibot.moduleSmartLEDSetBrightness(60); // Goz almasin / not too bright
  minibot.serialWrite(turkish ? "Renkli sicaklik gostergesi hazir." : "Color temperature indicator ready.");
}

void loop() {
  uint32_t now = millis();

  // 1) Her 2 saniyede bir oku, rengi guncelle, rapor yaz.
  // 1) Every 2 seconds: read, update the color, print a report.
  if (now - lastReadMs >= kReadEveryMs) {
    lastReadMs = now;
    lastTempC = minibot.moduleDhtTempReadC(DHT_PIN);
    int humidity = minibot.moduleDhtHumRead(DHT_PIN);

    if (lastTempC == -999) {
      minibot.moduleSmartLEDClear();
      minibot.serialWrite(turkish ? "Sensor okunamadi - baglantiyi kontrol edin (IO12)."
                                  : "Sensor read failed - check the wiring (IO12).");
    } else {
      int r, g, b;
      tempToColor(lastTempC, r, g, b);
      minibot.moduleSmartLEDFill(r, g, b);
      String report = (turkish ? "Sicaklik: " : "Temperature: ") + String(lastTempC) + " C | " +
                      (turkish ? "Nem: %" : "Humidity: %") + String(humidity) + " | " +
                      (turkish ? "Renk: " : "Color: ") + colorName(lastTempC);
      if (lastTempC > kWarningC) report += turkish ? "  !!! COK SICAK !!!" : "  !!! TOO HOT !!!";
      minibot.serialWrite(report);
    }
  }

  // 2) Sinir asildiysa her saniye uyari sesi (arka planda calar).
  // 2) If the limit is passed, a warning beep every second (plays in the background).
  if (lastTempC != -999 && lastTempC > kWarningC && now - lastBeepMs >= kBeepEveryMs) {
    lastBeepMs = now;
    minibot.buzzerPlay(2500, 150);
  }

  delay(20);
}
