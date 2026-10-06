// TR: GERCEK PROJE - Kablosuz Deprem Uyari Agi (MINIBOT ALICI). Bu MINIBOT
// kendi sensoru olmadan, IOTBOT'un ESP-NOW ile yayinladigi "deprem"
// mesajini dinler. "deprem = 1" gelince buzzer iki tonlu siren calar, mavi
// LED yanip soner ve (takiliysa) akilli LED kirmizi flas yapar. "deprem =
// 0" gelince hepsi durur. Buton 1 SADECE bu karttaki sesi susturur (isik
// yanip sonmeye devam eder). Once IOTBOT'a
// IOTBOT_ESPNOW_Earthquake_Alert_Sender_Example.ino dosyasini, istersen bir
// ROLEBOT'a da ROLEBOT_ESPNOW_Earthquake_Alert_Receiver_Example.ino
// dosyasini yukleyin.
// EN: A REAL PROJECT - Wireless Earthquake Alert Network (MINIBOT RECEIVER).
// This MINIBOT has no sensor of its own - it listens for the "deprem"
// message the IOTBOT broadcasts over ESP-NOW. On "deprem = 1" the buzzer
// plays a two-tone siren, the blue LED blinks and (if plugged in) the smart
// LED flashes red. On "deprem = 0" everything stops. Button 1 silences ONLY
// this board's sound (the light keeps blinking). First upload
// IOTBOT_ESPNOW_Earthquake_Alert_Sender_Example.ino to an IOTBOT, and
// optionally ROLEBOT_ESPNOW_Earthquake_Alert_Receiver_Example.ino to a ROLEBOT.
//
// Baglanti / Wiring: Akilli LED modulunu (istege bagli) Port A'ya (IO14)
// takin. Akilli LED'iniz yoksa asagidaki "#define USE_NEOPIXEL" satirini
// silin. / Plug the smart LED module (optional) into Port A (IO14). If you do
// not have a smart LED, delete the "#define USE_NEOPIXEL" line below.

#define USE_ESPNOW
#define USE_NEOPIXEL // Akilli LED yoksa bu satiri silin / delete this line if you have no smart LED
#include <MINIBOT.h>

MINIBOT minibot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define SMART_LED_PIN IO14 // Port A

namespace {
  constexpr int kEspNowChannel = 1;      // IOTBOT ile AYNI kanal / SAME channel as the IOTBOT
  constexpr uint32_t kStepMs = 300;      // Siren/flas adim suresi / siren/flash step time

  bool alarmOn = false;
  bool silenced = false;                 // Buton 1 ile susturuldu mu / silenced with Button 1?
  bool phase = false;
  uint32_t lastStepMs = 0;
  bool lastButton = true;                // HIGH = birakilmis / released

  void say(const char *turkishText, const char *englishText) {
    minibot.serialWrite(turkish ? turkishText : englishText);
  }

  void allOff() {
    minibot.ledWrite(false);
#if defined(USE_NEOPIXEL)
    minibot.moduleSmartLEDClear();
#endif
  }
}

void setup() {
  minibot.begin();
  minibot.serialStart(115200);
  minibot.espNowBegin(kEspNowChannel);
#if defined(USE_NEOPIXEL)
  minibot.moduleSmartLEDPrepare(SMART_LED_PIN);
#endif
  allOff();
  say("Deprem alicisi hazir - IOTBOT'tan uyari bekleniyor...",
      "Earthquake receiver ready - waiting for an alert from the IOTBOT...");
}

void loop() {
  // 1) Gelen mesaj. NOT: espNowReadName() mesaji "okundu" isaretler; bu yuzden
  // sayiyi ONCE receivedData.value'dan aliyoruz, adi SONRA okuyoruz.
  // 1) Incoming message. NOTE: espNowReadName() marks the message as read, so
  // we take the number FIRST from receivedData.value and read the name AFTER.
  if (minibot.espNowAvailable()) {
    minibot.espNowReadText();                    // Metin mesajiysa at / drop it if it is a text message
    float value = minibot.receivedData.value;
    String name = minibot.espNowReadName();
    if (name == "deprem") {
      bool danger = value > 0.5f;
      if (danger && !alarmOn) {                  // Tekrar eden "1"ler susturmayi bozmasin / repeated 1s must not undo silencing
        alarmOn = true;
        silenced = false;
        say("!!! DEPREM UYARISI !!! (Buton 1 = sesi kapat)", "!!! EARTHQUAKE ALERT !!! (Button 1 = mute)");
      } else if (!danger && alarmOn) {
        alarmOn = false;
        allOff();
        say("Tehlike gecti - alarm durdu.", "All clear - alarm stopped.");
      }
    }
  }

  // 2) Buton 1: sadece bu karttaki sesi sustur. / Button 1: mute only this board.
  bool button = minibot.button1Read(); // false = basili / pressed
  if (lastButton && !button && alarmOn && !silenced) {
    silenced = true;
    say("Ses kapatildi (isik devam ediyor).", "Sound muted (light keeps blinking).");
  }
  lastButton = button;

  // 3) Siren + yanip sonme, beklemeden (millis). / Siren + blinking without blocking (millis).
  if (alarmOn && millis() - lastStepMs >= kStepMs) {
    lastStepMs = millis();
    phase = !phase;
    minibot.ledWrite(phase);
    if (!silenced) minibot.buzzerPlay(phase ? 1400 : 900, kStepMs - 20); // tone() arka planda calar / tone() plays in the background
#if defined(USE_NEOPIXEL)
    if (phase) minibot.moduleSmartLEDFill(255, 0, 0);
    else minibot.moduleSmartLEDClear();
#endif
  }

  delay(10);
}
