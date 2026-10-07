/*
 * TR: İNTERNET SAATİ - Saat ve Alarm
 *  - MINIBOT WiFi'ye bağlanıp saati internetten (NTP) çeker ve her saniye seri porta
 *    saat, tarih ve günü yazar.
 *     - "Saat 07:30 OLUNCA" alarm BİR KEZ çalar (ntpTimeReached): 30 sn boyunca
 *       bip-bip-bip sesi ve yanıp sönen LED. B1 ya da "sustur" alarmı susturur.
 *     - "Saat 20:00 ile 07:00 ARASINDA İSE" mavi LED yanar (ntpTimeIsBetween).
 *     - "Saat 12:00 İSE" (o dakika boyunca) seri porta "Öğle vakti" yazılır (ntpTimeIs).
 *     - B1 butonuna basınca (alarm çalmıyorken) saat hemen GÜNCELLENİR (ntpUpdate).
 *    Bu fonksiyonlar editördeki "İnternet saatini kullan / güncelle", "saat ... ise",
 *    "saat ... olunca" bloklarının karşılığıdır.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim      / help         -> komut listesi
 *      alarm 06:45                -> alarm saatini değiştir ("alarm 6 45" de olur)
 *      test                       -> alarmı şimdi çaldır
 *      sustur      / stop         -> çalan alarmı sustur
 *      guncelle    / update       -> saati internetten yeniden al
 *      saat        / time         -> saati şimdi yaz
 *      sessiz      / quiet        -> her saniye saat yazmayı kapat/aç
 *      dil         / lang         -> dili değiştir (Türkçe <-> English)
 *
 * EN: INTERNET TIME - Clock and Alarm
 *  - The MINIBOT connects to WiFi, gets the time from the internet (NTP) and prints
 *    time, date and weekday every second.
 *     - "WHEN it is 07:30" the alarm rings ONCE (ntpTimeReached): beep-beep-beep and a
 *       blinking LED for 30 s. B1 or "stop" silences it.
 *     - "IF the time is BETWEEN 20:00 and 07:00" the blue LED is on (ntpTimeIsBetween).
 *     - "IF it is 12:00" (during that minute) "Noon" is printed (ntpTimeIs).
 *     - Pressing B1 (when no alarm is ringing) UPDATES the time right away (ntpUpdate).
 *    These functions are what the editor's "use / update internet time", "if time
 *    is ...", "when time is ..." blocks call.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help        / yardim       -> command list
 *      alarm 06:45                -> change the alarm time ("alarm 6 45" works too)
 *      test                       -> ring the alarm now
 *      stop        / sustur       -> silence a ringing alarm
 *      update      / guncelle     -> get the time from the internet again
 *      time        / saat         -> print the time now
 *      quiet       / sessiz       -> stop/start printing the time every second
 *      lang        / dil          -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. Aşağıya WiFi adınızı ve şifrenizi yazın.
 * NO extra module needed. Fill in your WiFi name and password below.
 */

#define USE_WIFI
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

const int kTimezoneHours = 3;             // Türkiye UTC+3 / Turkey UTC+3
int alarmHour = 7, alarmMinute = 30;      // Alarm saati / alarm time
const uint32_t kAlarmLengthMs = 30000;    // Alarm en fazla 30 sn çalar / the alarm rings for at most 30 s
const char *kDaysTr[] = {"", "Pazartesi", "Salı", "Çarşamba", "Perşembe", "Cuma", "Cumartesi", "Pazar"};
const char *kDaysEn[] = {"", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};

bool printEverySecond = true;
uint32_t lastPrintMs = 0;
bool ringing = false;                     // Alarm çalıyor mu / alarm ringing?
uint32_t ringStartMs = 0;
uint32_t lastBeepMs = 0;
int beepStep = 0;

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

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "GÜNCELLE" -> "guncelle"
// Lower-cases and simplifies Turkish letters: "GÜNCELLE" -> "guncelle"
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
// Saat, alarm ve mesajlar / Clock, alarm and messages
// ---------------------------------------------------------------------------
void printTime() {
  int wd = minibot.ntpGetWeekday(); // 1 = Pazartesi ... 7 = Pazar, -1 = saat yok / -1 = no time
  if (wd < 1) {
    minibot.serialWrite(L("Saat henüz alınamadı (WiFi/İnternet?)", "No time yet (WiFi/Internet?)"));
    return;
  }
  minibot.serialWrite(minibot.ntpGetTimeString() + "  " + minibot.ntpGetDateString() + "  " + (turkish ? kDaysTr[wd] : kDaysEn[wd]));
}

void startAlarm() {
  ringing = true;
  ringStartMs = millis();
  beepStep = 0;
  minibot.serialWrite(L("ALARM! Günaydın! (B1 ya da \"sustur\" = sustur)", "ALARM! Good morning! (B1 or \"stop\" = silence)"));
}

void stopAlarm() {
  ringing = false;
  minibot.serialWrite(L("Alarm susturuldu.", "Alarm silenced."));
}

void updateTime() {
  minibot.serialWrite(L("Saat güncelleniyor...", "Updating time..."));
  bool ok = minibot.ntpUpdate(); // "İnternet saatini güncelle" / "update internet time"
  minibot.serialWrite(ok ? L("Saat güncellendi.", "Time updated.") : L("Saat alınamadı.", "Could not get the time."));
}

void printAlarmTime() {
  char text[6];
  snprintf(text, sizeof(text), "%02d:%02d", alarmHour, alarmMinute);
  minibot.serialWrite(String(L("Alarm saati: ", "Alarm time: ")) + text);
}

void printHelp() {
  minibot.serialWrite(L("---- İNTERNET SAATİ - Komutlar ----", "---- INTERNET CLOCK - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  alarm 06:45   : alarm saatini değiştir", "  alarm 06:45   : change the alarm time"));
  minibot.serialWrite(L("  test          : alarmı şimdi çaldır", "  test          : ring the alarm now"));
  minibot.serialWrite(L("  sustur        : alarmı sustur", "  stop          : silence the alarm"));
  minibot.serialWrite(L("  guncelle      : saati yeniden al", "  update        : get the time again"));
  minibot.serialWrite(L("  saat          : saati yaz", "  time          : print the time"));
  minibot.serialWrite(L("  sessiz        : saniyelik yazmayı kapat/aç", "  quiet         : per-second printing off/on"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu     : alarmı sustur / saati güncelle", "  B1 button     : silence alarm / update time"));
  printAlarmTime();
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  String arg = (space < 0) ? "" : cmd.substring(space + 1);
  int h, m;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "alarm" && (sscanf(arg.c_str(), "%d:%d", &h, &m) == 2 || sscanf(arg.c_str(), "%d %d", &h, &m) == 2)) {
    if (h < 0 || h > 23 || m < 0 || m > 59) {
      minibot.serialWrite(L("Geçersiz saat. Örnek: alarm 06:45", "Invalid time. Example: alarm 06:45"));
      return;
    }
    alarmHour = h;
    alarmMinute = m;
    printAlarmTime();
  } else if (word == "test") {
    startAlarm();
  } else if (word == "sustur" || word == "stop") {
    if (ringing) stopAlarm();
    else minibot.serialWrite(L("Çalan bir alarm yok.", "No alarm is ringing."));
  } else if (word == "guncelle" || word == "update") {
    updateTime();
  } else if (word == "saat" || word == "time") {
    printTime();
  } else if (word == "sessiz" || word == "quiet") {
    printEverySecond = !printEverySecond;
    minibot.serialWrite(printEverySecond ? L("Her saniye saat yazılacak.", "Printing the time every second.")
                                         : L("Saniyelik yazma kapalı (\"saat\" ile sorabilirsiniz).", "Per-second printing off (ask with \"time\")."));
  } else if (word == "dil" || word == "lang" || word == "language") {
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
  minibot.wifiStartAndConnect(WIFI_SSID, WIFI_PASS);
  if (minibot.wifiConnectionControl()) {
    // "İnternet saatini kullan" / "use internet time"
    bool ok = minibot.ntpBegin(kTimezoneHours);
    minibot.serialWrite(ok ? L("Saat alındı.", "Time received.")
                           : L("Saat alınamadı, arka planda denenecek.", "No time yet, retrying in the background."));
  } else {
    minibot.serialWrite(L("WiFi YOK - ad/şifreyi kontrol edin.", "NO WiFi - check name/password."));
  }
  printHelp();
}

void loop() {
  // 1) B1: alarm çalıyorsa sustur, yoksa saati güncelle / B1: silence a ringing alarm, else update the time
  if (b1Pressed()) {
    if (ringing) stopAlarm();
    else updateTime();
  }

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) "Saat 07:30 olunca" - o dakikada SADECE BİR KEZ true / true only ONCE in that minute
  if (minibot.ntpTimeReached(alarmHour, alarmMinute)) startAlarm();

  // 4) Alarm çalıyorsa: bip-bip-bip ... mola (beklemeden), 30 sn sonra kendiliğinden biter.
  // 4) While ringing: beep-beep-beep ... pause (non-blocking), stops by itself after 30 s.
  if (ringing) {
    if (millis() - ringStartMs >= kAlarmLengthMs) {
      ringing = false;
      minibot.serialWrite(L("Alarm bitti.", "Alarm finished."));
    } else if (millis() - lastBeepMs >= 200) {
      lastBeepMs = millis();
      beepStep = (beepStep + 1) % 8;          // 0,2,4 = bip / beep; diğerleri sessiz / others silent
      bool beepNow = (beepStep % 2 == 0) && beepStep <= 4;
      if (beepNow) minibot.buzzerPlay(2000, 120);
      minibot.ledWrite(beepNow);
    }
  } else {
    // 5) "Saat 20:00 ile 07:00 arasında ise" LED yanar / "if the time is between 20:00 and 07:00" LED on
    minibot.ledWrite(minibot.ntpTimeIsBetween(20, 0, 7, 0));
  }

  // 6) Her saniye saati yaz / print the time every second
  if (millis() - lastPrintMs >= 1000) {
    lastPrintMs = millis();
    if (printEverySecond) printTime();
    // "Saat 12:00 ise" - o dakika boyunca her saniye true / "if it is 12:00" - true every second of that minute
    if (printEverySecond && minibot.ntpTimeIs(12, 0)) {
      minibot.serialWrite(L("  -> Öğle vakti!", "  -> Noon!"));
    }
  }
}
