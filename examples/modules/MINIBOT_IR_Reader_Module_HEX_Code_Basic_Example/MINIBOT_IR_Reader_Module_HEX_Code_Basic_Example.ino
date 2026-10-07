/*
 * TR: IR ALICI MODÜLÜ - Kumanda tuş kodlarını HEX (onaltılık) olarak okuma
 *  - Kızılötesi (IR) kumandayı modüle doğrultup bir tuşa basın: tuşun kodu seri
 *    porta "0xff30cf" gibi HEX olarak yazılır ve mavi LED kısa bir an yanar.
 *    Kumanda kodları genelde HEX yazılır; internetteki tablolarla karşılaştırabilirsiniz.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help     -> komut listesi
 *      son    / last     -> son okunan kodu tekrar yaz
 *      sayac  / count    -> kaç kod okunduğunu yaz
 *      dil    / lang     -> dili değiştir (Türkçe <-> English)
 *
 * EN: IR RECEIVER MODULE - Reading remote control key codes as HEX
 *  - Point an infrared (IR) remote at the module and press a key: the key's code is
 *    printed in HEX like "0xff30cf" and the blue LED flashes. Remote codes are
 *    usually written in HEX; you can compare them with tables on the internet.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help  / yardim    -> command list
 *      last  / son       -> print the last code again
 *      count / sayac     -> print how many codes were read
 *      lang  / dil       -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: IR alıcı modülünü IO12'ye bağlı sokete takın.
 * Desteklenen pinler / Supported pins: IO4 - IO5 - IO12 - IO13 - IO14
 *
 * NOT / NOTE: "#define USE_IR" satırı #include'dan ÖNCE yazılmalıdır.
 *             The "#define USE_IR" line must come BEFORE the #include.
 */

#define USE_IR
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

#define SENSOR_PIN IO12 // Sensörün bağlı olduğu pin / Pin the sensor is connected to

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

String lastCode = "";      // Son okunan kod / last code read
uint32_t codeCount = 0;    // Okunan kod sayısı / number of codes read
uint32_t ledOffAtMs = 0;

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
void printCode(const String &code) {
  String line = String(L("HEX kod: ", "HEX code: ")) + code;
  // NEC kumandalarda tuş basılı tutulunca 0xffffffff (tekrar kodu) gelir.
  // NEC remotes send 0xffffffff (repeat code) while a key is held down.
  if (code == "0xffffffff") line += L("  (tekrar: tuş basılı tutuluyor)", "  (repeat: key is held down)");
  minibot.serialWrite(line);
}

void printHelp() {
  minibot.serialWrite(L("---- IR ALICI (HEX) - Komutlar ----", "---- IR RECEIVER (HEX) - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  son           : son kodu tekrar yaz", "  last          : print the last code again"));
  minibot.serialWrite(L("  sayac         : okunan kod sayısı", "  count         : number of codes read"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "son" || cmd == "last") {
    if (codeCount == 0) minibot.serialWrite(L("Henüz kod okunmadı.", "No code read yet."));
    else printCode(lastCode);
  } else if (cmd == "sayac" || cmd == "count") {
    minibot.serialWrite(String(L("Okunan kod sayısı: ", "Codes read: ")) + codeCount);
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
  minibot.ledWrite(false);
  minibot.serialWrite(L("IR okuyucu testi başladı. Kumandadan bir tuşa basın.", "IR reader test started. Press a key on the remote."));
  printHelp();
}

void loop() {
  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) IR kodu oku: "0" = sinyal yok / read the IR code: "0" = no signal
  String code = minibot.moduleIRReadHex(SENSOR_PIN);
  if (code != "0") {
    lastCode = code;
    codeCount++;
    printCode(code);
    minibot.ledWrite(true);
    ledOffAtMs = millis() + 100;
  }

  // 3) Mavi LED'i söndür / turn the blue LED off
  if (ledOffAtMs != 0 && (int32_t)(millis() - ledOffAtMs) >= 0) {
    ledOffAtMs = 0;
    minibot.ledWrite(false);
  }
}
