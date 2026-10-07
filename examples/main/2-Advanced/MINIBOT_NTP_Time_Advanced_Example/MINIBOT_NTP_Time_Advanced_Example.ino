/*
 * TR: NTP SAAT SENKRONİZASYONU - İleri seviye örnek
 *  - MINIBOT WiFi'ye bağlanır, saati internetten (NTP) alır ve tarih/saati yazar.
 *  - Son senkron zamanını EEPROM'a CRC korumalı bir "kayıt" (record) olarak saklar.
 *    Açılışta önce ÖNCEKİ kaydı okur: kartı yeniden başlatınca bir önceki senkron
 *    zamanının hatırlandığını görürsünüz.
 *  - ntpBegin() WiFi bağlantısından SONRA çağrılmalıdır.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim   / help     -> komut listesi
 *      saat     / time     -> şu anki tarih ve saat
 *      senkron  / sync     -> saati yeniden al ve EEPROM'a kaydet
 *      kayit    / record   -> EEPROM'daki son senkron kaydını oku
 *      dil      / lang     -> dili değiştir (Türkçe <-> English)
 *
 * EN: NTP TIME SYNCHRONIZATION - Advanced example
 *  - MINIBOT joins WiFi, gets the time from the internet (NTP) and prints date/time.
 *  - It stores the last sync time in EEPROM as a CRC-protected "record". At startup it
 *    first reads the PREVIOUS record: restart the board and you see the previous sync
 *    time was remembered.
 *  - Call ntpBegin() AFTER connecting to WiFi.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help    / yardim    -> command list
 *      time    / saat      -> current date and time
 *      sync    / senkron   -> get the time again and save it to EEPROM
 *      record  / kayit     -> read the last sync record from EEPROM
 *      lang    / dil       -> switch language (Turkish <-> English)
 */

#define USE_WIFI
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

// WiFi bilgilerinizi yazın / fill in your WiFi credentials
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

// Türkiye UTC+3, yaz saati uygulaması yok / Turkey is UTC+3, no DST
static const int TIMEZONE_HOURS = 3;

// Kaydın saklandığı EEPROM adresi / EEPROM address where we store our record
static const int EEPROM_ADDR_LAST_SYNC = 200;

struct LastSyncRecord {
  uint32_t lastEpoch; // 1970'ten beri geçen saniye (Unix zamanı) / seconds since 1970 (Unix time)
};

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "KAYIT" -> "kayit"
// Lower-cases and simplifies Turkish letters: "KAYIT" -> "kayit"
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
// Saat ve EEPROM kaydı / Time and EEPROM record
// ---------------------------------------------------------------------------
void printTime() {
  if (!minibot.ntpIsTimeValid()) {
    minibot.serialWrite(L("Saat henüz geçerli değil (WiFi/İnternet?)", "Time not valid yet (WiFi/Internet?)"));
    return;
  }
  minibot.serialWrite(String(L("Epoch (Unix zamanı): ", "Epoch (Unix time): ")) + (unsigned long)minibot.ntpGetEpoch());
  minibot.serialWrite(String(L("Tarih/Saat: ", "Date/Time: ")) + minibot.ntpGetDateTimeString());
}

// EEPROM'daki kaydı okuyup yazar; geçerliyse true / reads and prints the record; true if valid
bool printRecord() {
  LastSyncRecord in;
  uint16_t len = 0, ver = 0;
  bool r = minibot.eepromReadRecord(EEPROM_ADDR_LAST_SYNC, (uint8_t *)&in, (uint16_t)sizeof(in), &len, &ver);
  if (r) {
    minibot.serialWrite(String(L("[EEPROM] Kayıt geçerli. sürüm=", "[EEPROM] Record ok. ver=")) + ver + L(" uzunluk=", " len=") + len +
                        L(" son senkron epoch=", " lastEpoch=") + (unsigned long)in.lastEpoch);
  } else {
    minibot.serialWrite(L("[EEPROM] Kayıt yok/geçersiz (sihirli sayı/sürüm/uzunluk/CRC uymadı)",
                          "[EEPROM] Record invalid (magic/version/len/crc mismatch)"));
  }
  return r;
}

// Şu anki zamanı CRC korumalı kayıt olarak sakla / store the current time as a CRC-protected record
void saveSyncTime() {
  if (!minibot.ntpIsTimeValid()) {
    minibot.serialWrite(L("[EEPROM] Saat geçerli değil, kaydedilmedi.", "[EEPROM] Time not valid, not saved."));
    return;
  }
  LastSyncRecord out;
  out.lastEpoch = (uint32_t)minibot.ntpGetEpoch();
  bool w = minibot.eepromWriteRecord(EEPROM_ADDR_LAST_SYNC, (const uint8_t *)&out, (uint16_t)sizeof(out), 1);
  minibot.serialWrite(w ? L("[EEPROM] Kayıt yazıldı.", "[EEPROM] Record written.") : L("[EEPROM] Kayıt yazılamadı.", "[EEPROM] Record write failed."));
}

// ---------------------------------------------------------------------------
// Mesajlar ve komutlar / Messages and commands
// ---------------------------------------------------------------------------
void printHelp() {
  minibot.serialWrite(L("---- NTP SAAT - Komutlar ----", "---- NTP TIME - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  saat          : tarih ve saat", "  time          : date and time"));
  minibot.serialWrite(L("  senkron       : saati al ve kaydet", "  sync          : get the time and save it"));
  minibot.serialWrite(L("  kayit         : EEPROM kaydını oku", "  record        : read the EEPROM record"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "saat" || cmd == "time") {
    printTime();
  } else if (cmd == "senkron" || cmd == "sync" || cmd == "guncelle" || cmd == "update") {
    if (WiFi.status() != WL_CONNECTED) {
      minibot.serialWrite(L("[WiFi] Bağlı değil.", "[WiFi] Not connected."));
      return;
    }
    bool ok = minibot.ntpUpdate(); // Son ayarlarla yeniden senkron (10 sn'ye kadar) / re-sync with the last settings (up to 10 s)
    minibot.serialWrite(ok ? L("[NTP] Senkron tamam", "[NTP] Synced") : L("[NTP] Senkron başarısız", "[NTP] Sync failed"));
    printTime();
    saveSyncTime();
  } else if (cmd == "kayit" || cmd == "record") {
    printRecord();
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
  delay(200);                  // Seri monitörün açılması için kısa bekleme / short wait for the serial monitor

  // 1) Önceki açılıştan kalan kaydı oku / read the record left from the previous boot
  minibot.eepromBegin(512);
  minibot.serialWrite(L("Önceki açılıştan kalan kayıt:", "Record from the previous boot:"));
  printRecord();

  // 2) WiFi'ye bağlan / connect WiFi
  minibot.wifiStartAndConnect(WIFI_SSID, WIFI_PASS);
  if (!minibot.wifiConnectionControl()) {
    minibot.serialWrite(L("[WiFi] Bağlı değil - SSID/şifreyi kontrol edin.", "[WiFi] Not connected - check SSID/password."));
    printHelp();
    return;
  }

  // 3) NTP kurulumu (tek satır, önerilen) / NTP setup (single call, recommended)
  bool ok = minibot.ntpBegin(TIMEZONE_HOURS);
  minibot.serialWrite(ok ? L("[NTP] Senkron tamam", "[NTP] Synced") : L("[NTP] Senkron başarısız", "[NTP] Sync failed"));
  printTime();

  // 4) Yeni senkron zamanını kaydet ve geri oku / save the new sync time and read it back
  saveSyncTime();
  printRecord();
  printHelp();
}

void loop() {
  // Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);
}
