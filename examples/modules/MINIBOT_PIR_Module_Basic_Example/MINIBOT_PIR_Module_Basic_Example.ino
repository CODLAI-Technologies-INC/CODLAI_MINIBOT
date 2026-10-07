/*
 * TR: PIR HAREKET SENSÖRÜ MODÜLÜ
 *  - PIR sensörü önünde bir canlı hareket edince "Hareket algılandı!", hareket
 *    bitince "Hareket yok" yazar. Mesaj sadece durum DEĞİŞİNCE yazılır.
 *    Hareket varken mavi LED yanar.
 *  - İpucu: PIR açıldıktan sonra ~30-60 sn ortama alışır; o sırada yanlış algılama
 *    olabilir. Hareket bittikten sonra da çıkış birkaç saniye "var" kalabilir.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help     -> komut listesi
 *      oku    / read     -> şu anki durumu yaz
 *      sayac  / count    -> kaç kez hareket algılandı
 *      ses    / sound    -> harekette bip sesini aç/kapat
 *      dil    / lang     -> dili değiştir (Türkçe <-> English)
 *
 * EN: PIR MOTION SENSOR MODULE
 *  - When something alive moves in front of the PIR sensor it prints "Motion
 *    detected!", and "No motion" when it stops. Messages are printed only when the
 *    state CHANGES. The blue LED is on while there is motion.
 *  - Tip: after power-up the PIR needs ~30-60 s to settle; false triggers may happen
 *    then. Its output can also stay "on" for a few seconds after motion stops.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help  / yardim    -> command list
 *      read  / oku       -> print the current state
 *      count / sayac     -> how many times motion was detected
 *      sound / ses       -> beep on motion on/off
 *      lang  / dil       -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: PIR modülünü IO12'ye bağlı sokete takın.
 * Desteklenen pinler / Supported pins: IO4 - IO5 - IO12 - IO13 - IO14
 * (IO5 kartın buzzer'ını da sürer / IO5 also drives the board's buzzer.)
 */

#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

#define SENSOR_PIN IO12 // Sensörün bağlı olduğu pin / Pin the sensor is connected to

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

bool motion = false;         // Son durum / last state
bool firstReading = true;
uint32_t motionCount = 0;    // Hareket sayısı / motion count
bool soundOn = false;        // Bip sesi (başlangıçta kapalı) / beep (off at start)
uint32_t lastReadMs = 0;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "SAYAÇ" -> "sayac"
// Lower-cases and simplifies Turkish letters: "SAYAÇ" -> "sayac"
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
void printState() {
  minibot.serialWrite(motion ? L("Hareket algılandı!", "Motion detected!") : L("Hareket yok.", "No motion."));
}

void printHelp() {
  minibot.serialWrite(L("---- PIR SENSÖRÜ - Komutlar ----", "---- PIR SENSOR - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  oku           : şu anki durum", "  read          : current state"));
  minibot.serialWrite(L("  sayac         : hareket sayısı", "  count         : motion count"));
  minibot.serialWrite(L("  ses           : bip sesini aç/kapat", "  sound         : beep on/off"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "oku" || cmd == "read") {
    printState();
  } else if (cmd == "sayac" || cmd == "count") {
    minibot.serialWrite(String(L("Hareket sayısı: ", "Motion count: ")) + motionCount);
  } else if (cmd == "ses" || cmd == "sound") {
    soundOn = !soundOn;
    minibot.serialWrite(soundOn ? L("Bip sesi AÇIK", "Beep ON") : L("Bip sesi KAPALI", "Beep OFF"));
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
  minibot.playIntro();         // Mavi LED 3 kez yanıp söner / the blue LED blinks 3 times
  minibot.serialWrite(L("PIR hareket sensörü testi başladı.", "PIR motion sensor test started."));
  printHelp();
}

void loop() {
  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) Sensörü 100 ms'de bir oku; sadece durum değişince yaz.
  // 2) Read the sensor every 100 ms; print only when the state changes.
  if (millis() - lastReadMs >= 100) {
    lastReadMs = millis();
    bool now = minibot.moduleMotionRead(SENSOR_PIN); // true = hareket var / motion
    if (now != motion || firstReading) {
      motion = now;
      if (motion && !firstReading) motionCount++;
      firstReading = false;
      minibot.ledWrite(motion);
      if (motion && soundOn) minibot.buzzerPlay(2000, 80);
      printState();
    }
  }
}
