/*
 * TR: GERÇEK PROJE - Kablosuz Deprem Uyarı Ağı (MINIBOT ALICI)
 *  - Bu MINIBOT'un kendi sensörü yoktur; IOTBOT'un ESP-NOW ile yayınladığı "deprem"
 *    mesajını dinler. "deprem = 1" gelince buzzer iki tonlu siren çalar, mavi LED
 *    yanıp söner ve (takılıysa) akıllı LED kırmızı flaş yapar. "deprem = 0" gelince
 *    hepsi durur.
 *  - B1 butonu SADECE bu karttaki sesi susturur (ışık yanıp sönmeye devam eder).
 *  - Önce IOTBOT'a IOTBOT_ESPNOW_Earthquake_Alert_Sender_Example.ino dosyasını,
 *    isterseniz bir ROLEBOT'a da ROLEBOT_ESPNOW_Earthquake_Alert_Receiver_Example.ino
 *    dosyasını yükleyin.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help     -> komut listesi
 *      sustur / mute     -> sesi kapat (B1 gibi)
 *      test              -> IOTBOT olmadan alarmı dene (tekrar "test" = durdur)
 *      durum  / status   -> alarm durumu
 *      dil    / lang     -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Wireless Earthquake Alert Network (MINIBOT RECEIVER)
 *  - This MINIBOT has no sensor of its own - it listens for the "deprem" message the
 *    IOTBOT broadcasts over ESP-NOW. On "deprem = 1" the buzzer plays a two-tone
 *    siren, the blue LED blinks and (if plugged in) the smart LED flashes red. On
 *    "deprem = 0" everything stops.
 *  - The B1 button silences ONLY this board's sound (the light keeps blinking).
 *  - First upload IOTBOT_ESPNOW_Earthquake_Alert_Sender_Example.ino to an IOTBOT, and
 *    optionally ROLEBOT_ESPNOW_Earthquake_Alert_Receiver_Example.ino to a ROLEBOT.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim   -> command list
 *      mute   / sustur   -> silence the sound (like B1)
 *      test              -> try the alarm without an IOTBOT ("test" again = stop)
 *      status / durum    -> alarm state
 *      lang   / dil      -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Akıllı LED modülünü (isteğe bağlı) Port A'ya (IO14) takın.
 * Akıllı LED'iniz yoksa aşağıdaki "#define USE_NEOPIXEL" satırını silin. / Plug the
 * smart LED module (optional) into Port A (IO14). If you do not have a smart LED,
 * delete the "#define USE_NEOPIXEL" line below.
 */

#define USE_ESPNOW
#define USE_NEOPIXEL // Akıllı LED yoksa bu satırı silin / delete this line if you have no smart LED
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

#define SMART_LED_PIN IO14 // Port A

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

const int kEspNowChannel = 1;   // IOTBOT ile AYNI kanal / SAME channel as the IOTBOT
const uint32_t kStepMs = 300;   // Siren/flaş adım süresi / siren/flash step time

bool alarmOn = false;
bool silenced = false;          // Bu kartta ses kapatıldı mı / sound muted on this board?
bool testMode = false;          // "test" komutuyla başlatıldı mı / started by the "test" command?
bool phase = false;
uint32_t lastStepMs = 0;

// ---------------------------------------------------------------------------
// B1 butonu (GPIO0) basılıyken LOW okunur: button1Read() == false -> basılı.
// Sadece basıldığı anı yakalar (40 ms titreşim filtresi).
// B1 (GPIO0) reads LOW while pressed: button1Read() == false -> pressed.
// Catches only the moment of the press (40 ms debounce).
// ---------------------------------------------------------------------------
bool lastB1 = false;
uint32_t lastB1ChangeMs = 0;

bool b1Pressed() {
  bool down = !minibot.button1Read();
  bool pressed = down && !lastB1 && millis() - lastB1ChangeMs > 40;
  if (down != lastB1) lastB1ChangeMs = millis();
  lastB1 = down;
  return pressed;
}

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "SUSTUR" -> "sustur"
// Lower-cases and simplifies Turkish letters
String normalizeCommand(String s) {
  s.trim();
  s.replace("İ", "i"); s.replace("I", "i"); s.replace("ı", "i");
  s.replace("Ş", "s"); s.replace("ş", "s");
  s.replace("Ğ", "g"); s.replace("ğ", "g");
  s.replace("Ü", "u"); s.replace("ü", "u");
  s.replace("Ö", "o"); s.replace("ö", "o");
  s.replace("Ç", "c"); s.replace("ç", "c");
  s.toLowerCase();
  return s;
}

bool readCommand(String &cmd) {
  while (Serial.available() > 0) {
    char c = Serial.read();
    lastCharMs = millis();
    if (c == '\n' || c == '\r') {
      if (cmdBuffer.length() == 0) continue;
      cmd = normalizeCommand(cmdBuffer);
      cmdBuffer = "";
      return true;
    }
    if (cmdBuffer.length() < 40) cmdBuffer += c;
  }
  // "Satır sonu yok" seçiliyse: 150 ms sessizlikten sonra komutu kabul et.
  // "No line ending" selected: accept the command after 150 ms of silence.
  if (cmdBuffer.length() > 0 && millis() - lastCharMs > 150) {
    cmd = normalizeCommand(cmdBuffer);
    cmdBuffer = "";
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Alarm ve mesajlar / Alarm and messages
// ---------------------------------------------------------------------------
void allOff() {
  minibot.ledWrite(false);
#if defined(USE_NEOPIXEL)
  minibot.moduleSmartLEDClear();
#endif
}

void startAlarm() {
  alarmOn = true;
  silenced = false;
  minibot.serialWrite(L("!!! DEPREM UYARISI !!! (B1 = sesi kapat)", "!!! EARTHQUAKE ALERT !!! (B1 = mute)"));
}

void stopAlarm() {
  alarmOn = false;
  testMode = false;
  allOff();
  minibot.serialWrite(L("Tehlike geçti - alarm durdu.", "All clear - alarm stopped."));
}

void mute() {
  if (alarmOn && !silenced) {
    silenced = true;
    minibot.serialWrite(L("Ses kapatıldı (ışık devam ediyor).", "Sound muted (light keeps blinking)."));
  } else if (!alarmOn) {
    minibot.serialWrite(L("Çalan bir alarm yok.", "No alarm is sounding."));
  }
}

void printHelp() {
  minibot.serialWrite(L("---- DEPREM ALICISI - Komutlar ----", "---- EARTHQUAKE RECEIVER - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  sustur        : sesi kapat", "  mute          : silence the sound"));
  minibot.serialWrite(L("  test          : alarmı dene / durdur", "  test          : try / stop the alarm"));
  minibot.serialWrite(L("  durum         : alarm durumu", "  status        : alarm state"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu     : sesi kapat", "  B1 button     : mute"));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "sustur" || cmd == "mute" || cmd == "sessiz") {
    mute();
  } else if (cmd == "test") {
    if (alarmOn && testMode) {
      stopAlarm();
    } else if (!alarmOn) {
      testMode = true;
      minibot.serialWrite(L("TEST alarmı (durdurmak için tekrar \"test\"):", "TEST alarm (type \"test\" again to stop):"));
      startAlarm();
    }
  } else if (cmd == "durum" || cmd == "status") {
    if (!alarmOn) minibot.serialWrite(L("Durum: sakin, IOTBOT dinleniyor.", "Status: calm, listening to the IOTBOT."));
    else minibot.serialWrite(String(L("Durum: ALARM", "Status: ALARM")) + (testMode ? L(" (test)", " (test)") : "") +
                             (silenced ? L(", ses kapalı", ", muted") : ""));
  } else if (cmd == "dil" || cmd == "lang" || cmd == "language") {
    turkish = !turkish;
    minibot.serialWrite(L("Dil: Türkçe", "Language: English"));
    printHelp();
  } else {
    minibot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  minibot.begin();             // MINIBOT başlatılıyor / Initialize MINIBOT
  minibot.serialStart(115200); // Seri haberleşme / Serial communication
  minibot.espNowBegin(kEspNowChannel);
#if defined(USE_NEOPIXEL)
  minibot.moduleSmartLEDPrepare(SMART_LED_PIN);
#endif
  allOff();
  minibot.serialWrite(L("Deprem alıcısı hazır - IOTBOT'tan uyarı bekleniyor...",
                        "Earthquake receiver ready - waiting for an alert from the IOTBOT..."));
  printHelp();
}

void loop() {
  // 1) Gelen mesaj. NOT: espNowReadName() mesajı "okundu" işaretler; bu yüzden
  //    sayıyı ÖNCE receivedData.value'dan alıyoruz, adı SONRA okuyoruz.
  // 1) Incoming message. NOTE: espNowReadName() marks the message as read, so
  //    we take the number FIRST from receivedData.value and read the name AFTER.
  if (minibot.espNowAvailable()) {
    minibot.espNowReadText();                // Metin mesajıysa at / drop it if it is a text message
    float value = minibot.receivedData.value;
    String name = minibot.espNowReadName();
    if (name == "deprem") {
      bool danger = value > 0.5f;
      if (danger && !alarmOn) {              // Tekrar eden "1"ler susturmayı bozmasın / repeated 1s must not undo muting
        testMode = false;
        startAlarm();
      } else if (danger && testMode) {
        testMode = false;                    // Gerçek uyarı testi devralır / a real alert takes over the test
      } else if (!danger && alarmOn && !testMode) {
        stopAlarm();
      }
    }
  }

  // 2) B1: sadece bu karttaki sesi sustur / B1: mute only this board
  if (b1Pressed()) mute();

  // 3) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 4) Siren + yanıp sönme, beklemeden (millis) / siren + blinking without blocking (millis)
  if (alarmOn && millis() - lastStepMs >= kStepMs) {
    lastStepMs = millis();
    phase = !phase;
    minibot.ledWrite(phase);
    if (!silenced) minibot.buzzerPlay(phase ? 1400 : 900, kStepMs - 20); // tone() arka planda çalar / plays in the background
#if defined(USE_NEOPIXEL)
    if (phase) minibot.moduleSmartLEDFill(255, 0, 0);
    else minibot.moduleSmartLEDClear();
#endif
  }
}
