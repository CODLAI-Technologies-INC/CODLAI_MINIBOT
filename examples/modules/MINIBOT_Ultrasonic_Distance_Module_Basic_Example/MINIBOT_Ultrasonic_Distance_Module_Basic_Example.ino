/*
 * TR: ULTRASONİK MESAFE SENSÖRÜ MODÜLÜ (HC-SR04)
 *  - Sensörün önündeki cisme olan mesafeyi santimetre (cm) olarak ölçer ve her
 *    saniye seri porta yazar. 0 cm = yansıma yok (cisim çok uzakta, 4 m'den fazla,
 *    ya da sensör bağlı değil).
 *  - Cisim 20 cm'den yakınsa mavi LED yanar.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help      -> komut listesi
 *      oku    / read      -> hemen bir ölçüm yap
 *      hizli  / fast      -> hızlı ölçüm (200 ms) aç/kapat
 *      dil    / lang      -> dili değiştir (Türkçe <-> English)
 *
 * EN: ULTRASONIC DISTANCE SENSOR MODULE (HC-SR04)
 *  - Measures the distance to the object in front of the sensor in centimeters (cm)
 *    and prints it every second. 0 cm = no echo (object too far, more than 4 m, or
 *    the sensor is not connected).
 *  - The blue LED is on when the object is closer than 20 cm.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help / yardim      -> command list
 *      read / oku         -> take a reading now
 *      fast / hizli       -> fast readings (200 ms) on/off
 *      lang / dil         -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ultrasonik sensör SABİT pinler kullanır: TRIG = IO12,
 * ECHO = IO13. Modülü bu pinlerin olduğu sokete takın; pin seçmenize gerek yok.
 * The ultrasonic sensor uses FIXED pins: TRIG = IO12, ECHO = IO13. Plug the module
 * into the socket with these pins; no pin to choose.
 */

#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

#define NEAR_CM 20 // Bu mesafeden yakınsa LED yanar / LED on when closer than this

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

bool fastMode = false;      // true = 200 ms, false = 1 sn / 1 s
uint32_t lastReadMs = 0;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "HIZLI" -> "hizli"
// Lower-cases and simplifies Turkish letters: "HIZLI" -> "hizli"
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
// Ölçüm ve mesajlar / Reading and messages
// ---------------------------------------------------------------------------
void readAndPrint() {
  int distance = minibot.moduleUltrasonicDistanceRead(); // cm, 0 = yansıma yok / no echo
  minibot.ledWrite(distance > 0 && distance < NEAR_CM);
  if (distance == 0) {
    minibot.serialWrite(L("Mesafe: --- (yansıma yok / menzil dışı)", "Distance: --- (no echo / out of range)"));
  } else {
    minibot.serialWrite(String(L("Mesafe: ", "Distance: ")) + distance + " cm");
  }
}

void printHelp() {
  minibot.serialWrite(L("---- ULTRASONİK MESAFE - Komutlar ----", "---- ULTRASONIC DISTANCE - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  oku           : hemen ölç", "  read          : measure now"));
  minibot.serialWrite(L("  hizli         : hızlı ölçüm aç/kapat", "  fast          : fast readings on/off"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "oku" || cmd == "read") {
    readAndPrint();
    lastReadMs = millis();
  } else if (cmd == "hizli" || cmd == "fast") {
    fastMode = !fastMode;
    minibot.serialWrite(fastMode ? L("Hızlı ölçüm: 200 ms", "Fast readings: 200 ms") : L("Normal ölçüm: 1 sn", "Normal readings: 1 s"));
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
  minibot.serialWrite(L("Ultrasonik mesafe testi başladı.", "Ultrasonic distance test started."));
  printHelp();
}

void loop() {
  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) Aralık dolunca ölç (delay yok) / measure when the interval is up (no delay)
  if (millis() - lastReadMs >= (fastMode ? 200UL : 1000UL)) {
    lastReadMs = millis();
    readAndPrint();
  }
}
