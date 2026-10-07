/*
 * TR: IR ALICI MODÜLÜ - Kumanda tuş kodlarını ONDALIK (decimal) sayı olarak okuma
 *  - Kızılötesi (IR) kumandayı modüle doğrultup bir tuşa basın: tuşun kodu seri
 *    porta sayı olarak yazılır ve mavi LED kısa bir an yanar. Bu kodları kendi
 *    projelerinizde "if (kod == ...)" ile kullanabilirsiniz.
 *  - 32 bit mod tam kodu verir; 8 bit mod sadece son 8 biti (0-255, daha kısa ve
 *    çoğu kumandada tuşları ayırmaya yeter).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help        -> komut listesi
 *      bit 8  / bit 32      -> kod uzunluğu (8 bit ya da 32 bit)
 *      son    / last        -> son okunan kodu tekrar yaz
 *      dil    / lang        -> dili değiştir (Türkçe <-> English)
 *
 * EN: IR RECEIVER MODULE - Reading remote control key codes as DECIMAL numbers
 *  - Point an infrared (IR) remote at the module and press a key: the key's code is
 *    printed as a number and the blue LED flashes. You can use these codes in your
 *    own projects with "if (code == ...)".
 *  - 32-bit mode gives the full code; 8-bit mode only the last 8 bits (0-255,
 *    shorter and enough to tell keys apart on most remotes).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help / yardim        -> command list
 *      bit 8 / bit 32       -> code length (8-bit or 32-bit)
 *      last / son           -> print the last code again
 *      lang / dil           -> switch language (Turkish <-> English)
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

bool mode32 = true;        // true = 32 bit, false = 8 bit
uint32_t lastCode = 0;     // Son okunan kod / last code read
uint32_t codeCount = 0;    // Okunan kod sayısı / number of codes read
uint32_t ledOffAtMs = 0;

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
void printCode(uint32_t code) {
  String line = String(L("Ondalık kod: ", "Decimal code: ")) + code;
  // NEC kumandalarda tuş basılı tutulunca 0xFFFFFFFF (tekrar kodu) gelir.
  // NEC remotes send 0xFFFFFFFF (repeat code) while a key is held down.
  if (mode32 && code == 0xFFFFFFFFUL) line += L("  (tekrar: tuş basılı tutuluyor)", "  (repeat: key is held down)");
  minibot.serialWrite(line);
}

void printHelp() {
  minibot.serialWrite(L("---- IR ALICI (ondalık) - Komutlar ----", "---- IR RECEIVER (decimal) - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  bit 8 / bit 32: kod uzunluğu", "  bit 8 / bit 32: code length"));
  minibot.serialWrite(L("  son           : son kodu tekrar yaz", "  last          : print the last code again"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  int value = (space > 0) ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "bit" && (value == 8 || value == 32)) {
    mode32 = (value == 32);
    minibot.serialWrite(mode32 ? L("Mod: 32 bit (tam kod)", "Mode: 32-bit (full code)") : L("Mod: 8 bit (son 8 bit)", "Mode: 8-bit (last 8 bits)"));
  } else if (word == "son" || word == "last") {
    if (codeCount == 0) minibot.serialWrite(L("Henüz kod okunmadı.", "No code read yet."));
    else printCode(lastCode);
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
  minibot.ledWrite(false);
  minibot.serialWrite(L("IR okuyucu testi başladı. Kumandadan bir tuşa basın.", "IR reader test started. Press a key on the remote."));
  printHelp();
}

void loop() {
  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) IR kodu oku: 0 = sinyal yok / read the IR code: 0 = no signal
  uint32_t code = mode32 ? (uint32_t)minibot.moduleIRReadDecimalx32(SENSOR_PIN)
                         : (uint32_t)minibot.moduleIRReadDecimalx8(SENSOR_PIN);
  if (code != 0) {
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
