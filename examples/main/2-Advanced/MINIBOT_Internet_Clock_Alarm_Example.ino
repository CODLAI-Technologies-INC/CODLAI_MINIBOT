// TR: INTERNET SAATI - Saat ve Alarm. MINIBOT WiFi'ye baglanip saati
// internetten (NTP) ceker ve her saniye Seri Port'a saat, tarih ve gunu yazar.
//  - "Saat 07:30 OLUNCA" alarm melodisi BIR KEZ calar (ntpTimeReached).
//  - "Saat 20:00 ile 07:00 ARASINDA ISE" mavi LED yanar (ntpTimeIsBetween).
//  - "Saat 12:00 ISE" (o dakika boyunca) Seri Port'a "Ogle vakti" yazilir
//    (ntpTimeIs).
//  - Butona basinca saat hemen GUNCELLENIR (ntpUpdate).
// Bu fonksiyonlar editordeki "Internet saatini kullan / guncelle", "saat ...
// ise", "saat ... olunca" bloklarinin karsiligidir.
// EN: INTERNET TIME - Clock and Alarm. The MINIBOT connects to WiFi, gets the
// time from the internet (NTP) and prints time, date and weekday to the Serial
// Port every second.
//  - "WHEN it is 07:30" the alarm melody plays ONCE (ntpTimeReached).
//  - "IF the time is BETWEEN 20:00 and 07:00" the blue LED is on
//    (ntpTimeIsBetween).
//  - "IF it is 12:00" (during that minute) "Noon" is printed (ntpTimeIs).
//  - Pressing the button UPDATES the time right away (ntpUpdate).
// These functions are what the editor's "use / update internet time", "if
// time is ...", "when time is ..." blocks call.
//
// Baglanti / Wiring: Ek modul GEREKMEZ. Asagiya WiFi adinizi ve sifrenizi
// yazin. / NO extra module needed. Fill in your WiFi name and password below.

#define USE_WIFI
#include <MINIBOT.h>

MINIBOT minibot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

namespace {
  constexpr int kTimezoneHours = 3;                 // Turkiye UTC+3 / Turkey UTC+3
  constexpr int kAlarmHour = 7, kAlarmMinute = 30;  // Alarm saati / alarm time
  const char *kDaysTr[] = {"", "Pazartesi", "Sali", "Carsamba", "Persembe", "Cuma", "Cumartesi", "Pazar"};
  const char *kDaysEn[] = {"", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};

  uint32_t lastPrintMs = 0;
  bool buttonWasDown = false;
}

void setup() {
  minibot.begin();
  minibot.serialStart(115200);
  Serial.println(turkish ? "WiFi'ye baglaniyor..." : "Connecting to WiFi...");
  minibot.wifiStartAndConnect(WIFI_SSID, WIFI_PASS);
  if (!minibot.wifiConnectionControl()) {
    Serial.println(turkish ? "WiFi YOK - ad/sifreyi kontrol edin." : "NO WiFi - check name/password.");
    return;
  }

  // "Internet saatini kullan" / "use internet time"
  bool ok = minibot.ntpBegin(kTimezoneHours);
  Serial.println(ok ? (turkish ? "Saat alindi." : "Time received.")
                    : (turkish ? "Saat alinamadi, arka planda denenecek." : "No time yet, retrying in the background."));
}

void loop() {
  // Buton = saati hemen guncelle / button = update the time now
  bool buttonDown = !minibot.button1Read(); // basiliyken LOW (false) / LOW (false) while pressed
  if (buttonDown && !buttonWasDown) {
    Serial.println(turkish ? "Saat guncelleniyor..." : "Updating time...");
    minibot.ntpUpdate(); // "Internet saatini guncelle" / "update internet time"
  }
  buttonWasDown = buttonDown;

  // "Saat 07:30 olunca" - o dakikada SADECE BIR KEZ true / true only ONCE in that minute
  if (minibot.ntpTimeReached(kAlarmHour, kAlarmMinute)) {
    Serial.println(turkish ? "ALARM! Gunaydin!" : "ALARM! Good morning!");
    minibot.buzzerPlayMelody(5);
  }

  // "Saat 20:00 ile 07:00 arasinda ise" / "if the time is between 20:00 and 07:00"
  minibot.ledWrite(minibot.ntpTimeIsBetween(20, 0, 7, 0));

  if (millis() - lastPrintMs >= 1000) {
    lastPrintMs = millis();
    int wd = minibot.ntpGetWeekday();
    Serial.print(minibot.ntpGetTimeString());
    Serial.print("  ");
    Serial.print(minibot.ntpGetDateString());
    Serial.print("  ");
    Serial.println(wd > 0 ? (turkish ? kDaysTr[wd] : kDaysEn[wd]) : "-");

    // "Saat 12:00 ise" - o dakika boyunca her saniye true / "if it is 12:00" - true every second of that minute
    if (minibot.ntpTimeIs(12, 0)) {
      Serial.println(turkish ? "  -> Ogle vakti!" : "  -> Noon!");
    }
  }
  delay(10);
}
