/*
 * TR: EEPROM (KALICI HAFIZA) - İleri seviye örnek
 *  - MINIBOT kütüphanesindeki EEPROM yardımcı fonksiyonlarını gösterir: 16 bit ve
 *    32 bit tam sayı, ondalık sayı (float), metin (String), bayt dizisi ve CRC korumalı
 *    "kayıt" (record) yazma/okuma.
 *  - EEPROM'daki veriler kart kapansa da SİLİNMEZ. Örnek bunu bir "açılış sayacı" ile
 *    gösterir: kart her açıldığında sayaç 1 artar (kartı resetleyip deneyin!).
 *  - ESP8266'da EEPROM aslında Flash bellektir; çok sık yazmak ömrünü kısaltır. Bu
 *    yüzden sadece açılışta ve sizin komutlarınızla yazıyoruz. Adres planını dikkatli
 *    yapın: farklı veriler aynı adres aralığını kullanmasın.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim     / help        -> komut listesi
 *      oku        / read        -> tüm örnek verileri oku ve yaz
 *      yaz 1234   / write 1234  -> 0. adrese 16 bit sayı yaz
 *      metin Ali  / text Ali    -> 30. adrese metin yaz (en fazla 30 harf)
 *      demo                     -> örnek verileri yeniden yaz
 *      sayac      / count       -> açılış sayacını yaz
 *      temizle    / clear       -> tüm EEPROM'u sil (0xFF) - dikkat!
 *      dil        / lang        -> dili değiştir (Türkçe <-> English)
 *
 * EN: EEPROM (PERSISTENT MEMORY) - Advanced example
 *  - Shows the EEPROM helper functions of the MINIBOT library: writing/reading 16-bit
 *    and 32-bit integers, floats, text (String), byte arrays and a CRC-protected
 *    "record".
 *  - Data in EEPROM is NOT lost when the board is turned off. The example shows this
 *    with a "boot counter": it goes up by 1 every time the board starts (reset the
 *    board and try!).
 *  - On the ESP8266 the EEPROM is really Flash memory; writing too often shortens its
 *    life. So we write only at startup and on your commands. Plan your addresses
 *    carefully: different data must not overlap.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help       / yardim      -> command list
 *      read       / oku         -> read and print all sample data
 *      write 1234 / yaz 1234    -> write a 16-bit number at address 0
 *      text Ali   / metin Ali   -> write text at address 30 (max 30 letters)
 *      demo                     -> write the sample data again
 *      count      / sayac       -> print the boot counter
 *      clear      / temizle     -> erase the whole EEPROM (0xFF) - careful!
 *      lang       / dil         -> switch language (Turkish <-> English)
 */

#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

// EEPROM adres planı (örnek) / EEPROM address layout (example)
//   0..1    : int16 (eski tip eepromWriteInt / legacy eepromWriteInt)
//   10..13  : int32
//   20..23  : float
//   30..95  : metin (2 bayt uzunluk + harfler) / text (2-byte length + letters)
//   120..123: bayt dizisi / byte array
//   200..   : CRC korumalı kayıt (açılış sayacı) / CRC-protected record (boot counter)
const int ADDR_INT16 = 0;
const int ADDR_INT32 = 10;
const int ADDR_FLOAT = 20;
const int ADDR_TEXT = 30;
const int ADDR_BYTES = 120;
const int CONFIG_ADDR = 200;

// CRC + sürümlü kayıtta saklanan yapı / structure stored in the CRC + versioned record
struct ExampleConfig {
  uint32_t bootCount;
  char name[16];
};

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

uint32_t bootCount = 0;
bool clearArmed = false; // "temizle" iki kez yazılınca siler / "clear" erases when typed twice

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// cmdRaw: komutun harfleri değiştirilmemiş hali (metin için).
// cmdRaw: the command with its letters untouched (for the text).
// ---------------------------------------------------------------------------
String cmdBuffer;
String cmdRaw;
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
      cmdRaw = cmdBuffer; cmdRaw.trim();
      cmd = normalizeCommand(cmdBuffer);
      cmdBuffer = "";
      return true;
    }
    if (cmdBuffer.length() < 40) cmdBuffer += c;
  }
  // "Satır sonu yok" seçiliyse: 150 ms sessizlikten sonra komutu kabul et.
  // "No line ending" selected: accept the command after 150 ms of silence.
  if (cmdBuffer.length() > 0 && millis() - lastCharMs > 150) {
    cmdRaw = cmdBuffer; cmdRaw.trim();
    cmd = normalizeCommand(cmdBuffer);
    cmdBuffer = "";
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// EEPROM işlemleri / EEPROM operations
// ---------------------------------------------------------------------------
void writeDemoData() {
  minibot.eepromWriteInt(ADDR_INT16, 1234);                 // 16 bit (2 bayt) / 16-bit (2 bytes)
  minibot.eepromWriteInt32(ADDR_INT32, 123456789);          // 32 bit (4 bayt) / 32-bit (4 bytes)
  minibot.eepromWriteFloat(ADDR_FLOAT, 36.5f);              // Ondalık (4 bayt) / float (4 bytes)
  minibot.eepromWriteString(ADDR_TEXT, String("Merhaba MINIBOT / Hello MINIBOT"), 64); // Uzunluk + metin / length + text
  uint8_t dataOut[4] = {1, 2, 3, 4};
  minibot.eepromWriteBytes(ADDR_BYTES, dataOut, sizeof(dataOut)); // Bayt dizisi / byte array
  minibot.serialWrite(L("Örnek veriler yazıldı.", "Sample data written."));
}

void readAll() {
  minibot.serialWrite(String(L("16 bit sayı  (adres 0)  : ", "16-bit number (addr 0)  : ")) + minibot.eepromReadInt(ADDR_INT16));
  minibot.serialWrite(String(L("32 bit sayı  (adres 10) : ", "32-bit number (addr 10) : ")) + (long)minibot.eepromReadInt32(ADDR_INT32, -1));
  minibot.serialWrite(String(L("Ondalık sayı (adres 20) : ", "Float        (addr 20)  : ")) + String(minibot.eepromReadFloat(ADDR_FLOAT, -1.0f), 2));
  minibot.serialWrite(String(L("Metin        (adres 30) : ", "Text         (addr 30)  : ")) + minibot.eepromReadString(ADDR_TEXT, 64));

  uint8_t dataIn[4] = {0};
  minibot.eepromReadBytes(ADDR_BYTES, dataIn, sizeof(dataIn));
  String bytes;
  for (size_t i = 0; i < sizeof(dataIn); i++) {
    bytes += String(dataIn[i]);
    if (i + 1 < sizeof(dataIn)) bytes += ",";
  }
  minibot.serialWrite(String(L("Bayt dizisi  (adres 120): ", "Byte array   (addr 120) : ")) + bytes);
  minibot.serialWrite(String(L("Açılış sayacı (kayıt)   : ", "Boot counter (record)   : ")) + bootCount);
}

// CRC korumalı kaydı oku, sayacı 1 artır, geri yaz.
// Read the CRC-protected record, add 1 to the counter, write it back.
void updateBootCounter() {
  ExampleConfig cfg;
  uint16_t outLen = 0, outVer = 0;
  bool ok = minibot.eepromReadRecord(CONFIG_ADDR, (uint8_t *)&cfg, (uint16_t)sizeof(cfg), &outLen, &outVer);
  if (ok && outLen == sizeof(cfg)) {
    cfg.bootCount++;
    minibot.serialWrite(String(L("Kayıt geçerli (sürüm ", "Record valid (version ")) + outVer + L(", uzunluk ", ", length ") + outLen + ").");
  } else {
    // İlk açılış ya da bozuk veri: sıfırdan başla / first boot or corrupted data: start over
    memset(&cfg, 0, sizeof(cfg));
    cfg.bootCount = 1;
    minibot.serialWrite(L("Kayıt bulunamadı (sihirli sayı/uzunluk/CRC uymadı) - yeni kayıt oluşturuluyor.",
                          "No record found (magic/length/CRC mismatch) - creating a new record."));
  }
  snprintf(cfg.name, sizeof(cfg.name), "%s", "MINIBOT");
  bool w = minibot.eepromWriteRecord(CONFIG_ADDR, (const uint8_t *)&cfg, (uint16_t)sizeof(cfg), 1);
  bootCount = cfg.bootCount;
  minibot.serialWrite(w ? String(L("Bu kart ", "This board has started ")) + bootCount + L(". kez açıldı.", " times.")
                        : String(L("Kayıt yazılamadı!", "Record write failed!")));
}

// ---------------------------------------------------------------------------
// Mesajlar ve komutlar / Messages and commands
// ---------------------------------------------------------------------------
void printHelp() {
  minibot.serialWrite(L("---- EEPROM - Komutlar ----", "---- EEPROM - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  oku           : tüm verileri oku", "  read          : read all data"));
  minibot.serialWrite(L("  yaz 1234      : 0. adrese sayı yaz", "  write 1234    : write a number at address 0"));
  minibot.serialWrite(L("  metin <yazı>  : 30. adrese metin yaz", "  text <words>  : write text at address 30"));
  minibot.serialWrite(L("  demo          : örnek verileri yeniden yaz", "  demo          : write the sample data again"));
  minibot.serialWrite(L("  sayac         : açılış sayacı", "  count         : boot counter"));
  minibot.serialWrite(L("  temizle       : EEPROM'u sil (iki kez yazın)", "  clear         : erase EEPROM (type twice)"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  bool isClear = (word == "temizle" || word == "clear");
  if (!isClear) clearArmed = false;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oku" || word == "read") {
    readAll();
  } else if ((word == "yaz" || word == "write") && hasValue) {
    int value = cmd.substring(space + 1).toInt();
    minibot.eepromWriteInt(ADDR_INT16, value);
    // 16 bit: -32768..32767 arası saklanır / 16-bit: stores -32768..32767
    minibot.serialWrite(String(L("Yazıldı. Geri okunan: ", "Written. Read back: ")) + minibot.eepromReadInt(ADDR_INT16));
  } else if ((word == "metin" || word == "text") && hasValue) {
    String text = cmdRaw.substring(cmdRaw.indexOf(' ') + 1);
    text.trim();
    minibot.eepromWriteString(ADDR_TEXT, text, 64);
    minibot.serialWrite(String(L("Yazıldı. Geri okunan: ", "Written. Read back: ")) + minibot.eepromReadString(ADDR_TEXT, 64));
  } else if (word == "demo") {
    writeDemoData();
  } else if (word == "sayac" || word == "count") {
    minibot.serialWrite(String(L("Açılış sayacı: ", "Boot counter: ")) + bootCount);
  } else if (isClear) {
    if (!clearArmed) {
      clearArmed = true;
      minibot.serialWrite(L("DİKKAT: tüm EEPROM silinecek! Onaylamak için tekrar \"temizle\" yazın.",
                            "WARNING: the whole EEPROM will be erased! Type \"clear\" again to confirm."));
    } else {
      clearArmed = false;
      minibot.eepromClear(0, 512, 0xFF);
      bootCount = 0;
      minibot.serialWrite(L("EEPROM silindi.", "EEPROM erased."));
    }
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
  delay(200);                  // Seri monitörün açılması için kısa bekleme / short wait for the serial monitor

  // EEPROM'u başlatın. ESP8266'da EEPROM.commit() için begin gereklidir.
  // Initialize EEPROM. On ESP8266, EEPROM.begin() is required for commit().
  bool ok = minibot.eepromBegin(512);
  minibot.serialWrite(ok ? L("[EEPROM] Hazır", "[EEPROM] Ready") : L("[EEPROM] Başlatılamadı", "[EEPROM] Begin failed"));

  updateBootCounter(); // Önerilen yöntem: CRC + sürümlü kayıt / recommended: CRC + versioned record
  writeDemoData();     // Diğer veri türleri / the other data types
  readAll();
  printHelp();
}

void loop() {
  // Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);
}
