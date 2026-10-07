/*
 * TR: KABLOSUZ YAZILIM GÜNCELLEME (OTA - Over The Air)
 *  - MINIBOT WiFi'ye bağlanır ve OTA'yı başlatır. Bundan sonra yeni kodu USB kablosu
 *    OLMADAN, aynı ağdaki bilgisayardan yükleyebilirsiniz: Arduino IDE'de
 *    Araçlar > Port listesinde "MINIBOT-OTA" ağ portunu seçin (şifre: 1234).
 *  - Kodun çalıştığını görmek için mavi LED saniyede bir yanıp söner. Yanıp sönme
 *    hızını (BLINK_MS) değiştirip OTA ile yükleyin: değişikliği hemen görürsünüz!
 *  - OTA kullanmak için önce WiFi bağlantısı kurulmalı ve loop() içinde otaHandle()
 *    sürekli çağrılmalıdır; loop()'u uzun delay()'lerle bekletmeyin.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help     -> komut listesi
 *      durum  / status   -> WiFi, IP adresi ve OTA adı
 *      dil    / lang     -> dili değiştir (Türkçe <-> English)
 *
 * EN: WIRELESS SOFTWARE UPDATE (OTA - Over The Air)
 *  - MINIBOT joins WiFi and starts OTA. From then on you can upload new code WITHOUT
 *    a USB cable, from a computer on the same network: in Arduino IDE pick the
 *    "MINIBOT-OTA" network port under Tools > Port (password: 1234).
 *  - The blue LED blinks once per second to show the code is running. Change the
 *    blink speed (BLINK_MS) and upload over OTA: you see the change right away!
 *  - To use OTA, WiFi must be connected first and otaHandle() must be called all the
 *    time in loop(); don't block loop() with long delay()s.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim   -> command list
 *      status / durum    -> WiFi, IP address and OTA name
 *      lang   / dil      -> switch language (Turkish <-> English)
 *
 * NOT / NOTE: USE_OTA ve USE_WIFI satırları #include'dan ÖNCE yazılmalıdır.
 *             The USE_OTA and USE_WIFI lines must come BEFORE the #include.
 */

#define USE_WIFI
#define USE_OTA
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

const char *WIFI_SSID = "YOUR_WIFI_SSID";
const char *WIFI_PASS = "YOUR_WIFI_PASSWORD";

const char *OTA_HOST = "MINIBOT-OTA"; // Cihaz adı (ağ portunda görünür) / device name (shown as the network port)
const char *OTA_PASS = "1234";        // OTA şifresi (varsayılan 1234) / OTA password (default 1234)

#define BLINK_MS 1000 // LED yanıp sönme süresi (ms) / LED blink time (ms)

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

uint32_t lastBlinkMs = 0;
bool ledState = false;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir / lower-cases and simplifies Turkish letters
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
// Mesajlar / Messages
// ---------------------------------------------------------------------------
void printStatus() {
  if (WiFi.status() == WL_CONNECTED) {
    minibot.serialWrite(String(L("WiFi bağlı. IP adresi: ", "WiFi connected. IP address: ")) + WiFi.localIP().toString());
    minibot.serialWrite(String(L("OTA adı: ", "OTA name: ")) + OTA_HOST + L("  (Arduino IDE > Araçlar > Port)", "  (Arduino IDE > Tools > Port)"));
  } else {
    minibot.serialWrite(L("WiFi bağlı DEĞİL - OTA çalışmaz. SSID/şifreyi kontrol edin.", "WiFi NOT connected - OTA won't work. Check SSID/password."));
  }
}

void printHelp() {
  minibot.serialWrite(L("---- OTA GÜNCELLEME - Komutlar ----", "---- OTA UPDATE - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  durum         : WiFi, IP, OTA adı", "  status        : WiFi, IP, OTA name"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "durum" || cmd == "status") {
    printStatus();
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
  minibot.serialWrite(L("WiFi'ye bağlanılıyor...", "Connecting to WiFi..."));

  minibot.wifiStartAndConnect(WIFI_SSID, WIFI_PASS); // Önce WiFi / WiFi first
  minibot.otaBegin(OTA_HOST, OTA_PASS, 8266);        // Sonra OTA (8266 = ESP8266 OTA portu) / then OTA (8266 = ESP8266 OTA port)

  printStatus();
  printHelp();
}

void loop() {
  minibot.otaHandle(); // OTA isteklerini dinle - her döngüde çağrılmalı / listen for OTA requests - call every loop

  // Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // "Kod çalışıyor" göstergesi: LED yanıp söner (delay yok) / "code is running" sign: LED blinks (no delay)
  if (millis() - lastBlinkMs >= BLINK_MS) {
    lastBlinkMs = millis();
    ledState = !ledState;
    minibot.ledWrite(ledState);
  }
}
