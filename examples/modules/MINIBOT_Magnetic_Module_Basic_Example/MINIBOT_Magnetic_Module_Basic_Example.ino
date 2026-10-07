/*
 * TR: MANYETİK SENSÖR MODÜLÜ (reed / Hall)
 *  - Sensöre bir mıknatıs yaklaşınca "Mıknatıs VAR", uzaklaşınca "Mıknatıs YOK" yazar.
 *    Mesaj sadece durum DEĞİŞİNCE yazılır (seri port dolmaz). Mıknatıs varken mavi
 *    LED yanar; her değişimde kısa bir bip duyulur.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help     -> komut listesi
 *      oku    / read     -> şu anki durumu yaz
 *      sayac  / count    -> mıknatıs kaç kez algılandı
 *      ses    / sound    -> bip sesini aç/kapat
 *      dil    / lang     -> dili değiştir (Türkçe <-> English)
 *
 * EN: MAGNETIC SENSOR MODULE (reed / Hall)
 *  - When a magnet comes near the sensor it prints "Magnet PRESENT", when it moves
 *    away "Magnet ABSENT". Messages are printed only when the state CHANGES (no
 *    serial flood). The blue LED is on while the magnet is present; each change beeps.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help  / yardim    -> command list
 *      read  / oku       -> print the current state
 *      count / sayac     -> how many times the magnet was detected
 *      sound / ses       -> beep on/off
 *      lang  / dil       -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Manyetik sensör modülünü IO12'ye bağlı sokete takın.
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

bool magnetPresent = false;  // Son durum / last state
bool firstReading = true;
uint32_t detectCount = 0;    // Algılama sayısı / detection count
bool soundOn = true;         // Bip sesi açık mı / beep enabled?
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
  minibot.serialWrite(magnetPresent ? L("Manyetik sensör: Mıknatıs VAR", "Magnetic sensor: Magnet PRESENT")
                                    : L("Manyetik sensör: Mıknatıs YOK", "Magnetic sensor: Magnet ABSENT"));
}

void printHelp() {
  minibot.serialWrite(L("---- MANYETİK SENSÖR - Komutlar ----", "---- MAGNETIC SENSOR - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  oku           : şu anki durum", "  read          : current state"));
  minibot.serialWrite(L("  sayac         : algılama sayısı", "  count         : detection count"));
  minibot.serialWrite(L("  ses           : bip sesini aç/kapat", "  sound         : beep on/off"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "oku" || cmd == "read") {
    printState();
  } else if (cmd == "sayac" || cmd == "count") {
    minibot.serialWrite(String(L("Mıknatıs algılama sayısı: ", "Magnet detections: ")) + detectCount);
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
  minibot.serialWrite(L("Manyetik sensör testi başladı.", "Magnetic sensor test started."));
  printHelp();
}

void loop() {
  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) Sensörü 50 ms'de bir oku; sadece durum değişince yaz.
  // 2) Read the sensor every 50 ms; print only when the state changes.
  if (millis() - lastReadMs >= 50) {
    lastReadMs = millis();
    bool present = minibot.moduleMagneticRead(SENSOR_PIN); // true = mıknatıs var / magnet present
    if (present != magnetPresent || firstReading) {
      magnetPresent = present;
      if (present && !firstReading) detectCount++;
      firstReading = false;
      minibot.ledWrite(present);
      if (soundOn) minibot.buzzerPlay(present ? 1800 : 900, 50);
      printState();
    }
  }
}
