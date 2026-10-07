/*
 * TR: DHT11 SICAKLIK VE NEM MODÜLÜ
 *  - Her 2 saniyede bir sıcaklığı (°C), nemi (%) ve hissedilen sıcaklığı ölçüp
 *    seri porta yazar. DHT11 yavaş bir sensördür; 2 saniyeden sık okunmaz.
 *  - Okuma yapılırken mavi LED kısa bir an yanar.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim   / help         -> komut listesi
 *      oku      / read         -> hemen bir ölçüm yap
 *      aralik 5 / interval 5   -> ölçüm aralığı (saniye, 2-60)
 *      dil      / lang         -> dili değiştir (Türkçe <-> English)
 *
 * EN: DHT11 TEMPERATURE AND HUMIDITY MODULE
 *  - Every 2 seconds it measures the temperature (°C), humidity (%) and the
 *    "feels like" temperature and prints them. The DHT11 is a slow sensor; it
 *    can't be read more often than every 2 seconds.
 *  - The blue LED flashes briefly on each reading.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help       / yardim     -> command list
 *      read       / oku        -> take a reading now
 *      interval 5 / aralik 5   -> reading interval (seconds, 2-60)
 *      lang       / dil        -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: DHT11 modülünü IO12'ye bağlı sokete takın.
 * Desteklenen pinler / Supported pins: IO4 - IO5 - IO12 - IO13 - IO14
 *
 * NOT / NOTE: "#define USE_DHT" satırı #include'dan ÖNCE yazılmalıdır.
 *             The "#define USE_DHT" line must come BEFORE the #include.
 */

#define USE_DHT
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

#define SENSOR_PIN IO12 // Sensörün bağlı olduğu pin / Pin the sensor is connected to

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

uint32_t intervalMs = 2000; // Ölçüm aralığı / reading interval
uint32_t lastReadMs = 0;
uint32_t ledOffAtMs = 0;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "ARALIK" -> "aralik"
// Lower-cases and simplifies Turkish letters: "ARALIK" -> "aralik"
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
  minibot.ledWrite(true);
  ledOffAtMs = millis() + 80;

  int temperature = minibot.moduleDhtTempReadC(SENSOR_PIN); // Sıcaklık (°C) / temperature (°C)
  int humidity = minibot.moduleDhtHumRead(SENSOR_PIN);      // Nem (%) / humidity (%)
  int feelsLike = minibot.moduleDthFeelingTempC(SENSOR_PIN); // Hissedilen (°C) / feels like (°C)

  // Kütüphane okuma hatasında -999 döndürür / the library returns -999 on a read error
  if (temperature == -999 || humidity == -999) {
    minibot.serialWrite(L("Sensör okunamadı - bağlantıyı kontrol edin (IO12).", "Sensor read failed - check the wiring (IO12)."));
    return;
  }
  String line = String(L("Sıcaklık: ", "Temperature: ")) + temperature + " °C  |  " +
                L("Nem: %", "Humidity: ") + humidity + L("", " %") + "  |  " +
                L("Hissedilen: ", "Feels like: ") + feelsLike + " °C";
  minibot.serialWrite(line);
}

void printHelp() {
  minibot.serialWrite(L("---- DHT11 - Komutlar ----", "---- DHT11 - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  oku           : hemen ölç", "  read          : measure now"));
  minibot.serialWrite(L("  aralik 2-60   : ölçüm aralığı (saniye)", "  interval 2-60 : reading interval (seconds)"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oku" || word == "read") {
    readAndPrint();
    lastReadMs = millis();
  } else if ((word == "aralik" || word == "interval") && hasValue) {
    intervalMs = (uint32_t)constrain(value, 2, 60) * 1000;
    minibot.serialWrite(String(L("Ölçüm aralığı: ", "Reading interval: ")) + (intervalMs / 1000) + L(" saniye", " seconds"));
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
  minibot.playIntro();         // Mavi LED 3 kez yanıp söner / the blue LED blinks 3 times
  minibot.serialWrite(L("DHT11 sıcaklık/nem testi başladı.", "DHT11 temperature/humidity test started."));
  printHelp();
}

void loop() {
  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) Aralık dolunca ölç (delay yok) / measure when the interval is up (no delay)
  if (millis() - lastReadMs >= intervalMs) {
    lastReadMs = millis();
    readAndPrint();
  }

  // 3) Mavi LED'i söndür / turn the blue LED off
  if (ledOffAtMs != 0 && (int32_t)(millis() - ledOffAtMs) >= 0) {
    ledOffAtMs = 0;
    minibot.ledWrite(false);
  }
}
