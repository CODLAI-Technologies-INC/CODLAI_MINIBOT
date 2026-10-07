/*
 * TR: TİTREŞİM SENSÖRÜ MODÜLÜ
 *  - Sensöre vurunca ya da onu sallayınca "Titreşim algılandı!" yazar, mavi LED
 *    yanar ve kısa bir bip duyulur. Bir darbe sensörü birkaç kez tetikleyebildiği
 *    için mesajlar en fazla 300 ms'de bir yazılır; toplam sayı da gösterilir.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim  / help     -> komut listesi
 *      sayac   / count    -> toplam titreşim sayısı
 *      sifirla / reset    -> sayacı sıfırla
 *      ses     / sound    -> bip sesini aç/kapat
 *      dil     / lang     -> dili değiştir (Türkçe <-> English)
 *
 * EN: VIBRATION SENSOR MODULE
 *  - Tap or shake the sensor: it prints "Vibration detected!", the blue LED lights
 *    up and a short beep sounds. One knock can trigger the sensor several times, so
 *    messages are printed at most every 300 ms; the total count is shown too.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help  / yardim     -> command list
 *      count / sayac      -> total vibration count
 *      reset / sifirla    -> reset the counter
 *      sound / ses        -> beep on/off
 *      lang  / dil        -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Titreşim sensörü modülünü IO12'ye bağlı sokete takın.
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

uint32_t vibrationCount = 0; // Toplam titreşim / total vibrations
uint32_t lastEventMs = 0;    // Son mesaj zamanı / time of the last message
bool soundOn = true;
uint32_t ledOffAtMs = 0;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "SIFIRLA" -> "sifirla"
// Lower-cases and simplifies Turkish letters: "SIFIRLA" -> "sifirla"
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
void printHelp() {
  minibot.serialWrite(L("---- TİTREŞİM SENSÖRÜ - Komutlar ----", "---- VIBRATION SENSOR - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  sayac         : toplam titreşim sayısı", "  count         : total vibration count"));
  minibot.serialWrite(L("  sifirla       : sayacı sıfırla", "  reset         : reset the counter"));
  minibot.serialWrite(L("  ses           : bip sesini aç/kapat", "  sound         : beep on/off"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "sayac" || cmd == "count") {
    minibot.serialWrite(String(L("Toplam titreşim: ", "Total vibrations: ")) + vibrationCount);
  } else if (cmd == "sifirla" || cmd == "reset") {
    vibrationCount = 0;
    minibot.serialWrite(L("Sayaç sıfırlandı.", "Counter reset."));
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
  minibot.serialWrite(L("Titreşim sensörü testi başladı. Sensöre hafifçe vurun.", "Vibration sensor test started. Tap the sensor gently."));
  printHelp();
}

void loop() {
  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) Sensörü sürekli oku (kısa darbeleri kaçırmamak için), en fazla 300 ms'de bir yaz.
  // 2) Read the sensor all the time (so short knocks are not missed), print at most every 300 ms.
  if (minibot.moduleVibrationDigitalRead(SENSOR_PIN) && millis() - lastEventMs >= 300) {
    lastEventMs = millis();
    vibrationCount++;
    minibot.serialWrite(String(L("Titreşim algılandı! (toplam: ", "Vibration detected! (total: ")) + vibrationCount + ")");
    minibot.ledWrite(true);
    ledOffAtMs = millis() + 200;
    if (soundOn) minibot.buzzerPlay(2200, 60);
  }

  // 3) Mavi LED'i söndür / turn the blue LED off
  if (ledOffAtMs != 0 && (int32_t)(millis() - ledOffAtMs) >= 0) {
    ledOffAtMs = 0;
    minibot.ledWrite(false);
  }
}
