/*
 * TR: GERÇEK PROJE - Renkli Sıcaklık Göstergesi
 *  - OTOMATİK modda DHT sensörü odanın sıcaklığını ölçer, akıllı LED'ler de sıcaklığı
 *    RENK ile gösterir: soğuksa MAVİ, rahatsa YEŞİL, sıcaksa KIRMIZI - aradaki
 *    değerlerde renkler yavaşça birbirine karışır. Sıcaklık uyarı sınırını (32 °C)
 *    geçerse buzzer her saniye uyarı sesi verir. Her 2 saniyede bir seri porta
 *    sıcaklık, nem ve renk raporu yazılır. Sensörü elinizle ısıtıp renkleri izleyin!
 *  - B1 butonuna basınca MANUEL (deneme) moda geçer: sensör yerine sıcaklığı siz
 *    verirsiniz ("sicaklik 30" gibi) ve rengin nasıl değiştiğini görürsünüz.
 *    B1'e tekrar basınca gerçek sensöre döner.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim       / help           -> komut listesi
 *      oto          / auto           -> otomatik mod (gerçek sensör)
 *      manuel       / manual         -> manuel (deneme) mod
 *      sicaklik 30  / temp 30        -> 30 °C varmış gibi göster (manuele geçer)
 *      parlaklik 50 / brightness 50  -> LED parlaklığı (%0-100)
 *      sessiz       / mute           -> uyarı sesini kapat/aç
 *      oku          / read           -> sensörü hemen oku
 *      dil          / lang           -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Color Temperature Indicator
 *  - In AUTO mode the DHT sensor measures the room temperature and the smart LEDs show
 *    it as a COLOR: BLUE when cold, GREEN when comfortable, RED when hot - in between,
 *    the colors blend smoothly. If the temperature passes the warning limit (32 °C)
 *    the buzzer beeps every second. Every 2 seconds a temperature, humidity and color
 *    report is printed. Warm the sensor with your hand and watch the colors change!
 *  - Press B1 to switch to MANUAL (test) mode: instead of the sensor you give the
 *    temperature (like "temp 30") and see how the color changes. Press B1 again to
 *    go back to the real sensor.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help           / yardim       -> command list
 *      auto           / oto          -> auto mode (real sensor)
 *      manual         / manuel       -> manual (test) mode
 *      temp 30        / sicaklik 30  -> show as if it were 30 °C (switches to manual)
 *      brightness 50  / parlaklik 50 -> LED brightness (0-100 %)
 *      mute           / sessiz       -> warning beep off/on
 *      read           / oku          -> read the sensor now
 *      lang           / dil          -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: DHT sensörünü IO12'ye, akıllı LED modülünü Port A'ya (IO14)
 * takın. / Plug the DHT sensor into IO12 and the smart LED module into Port A (IO14).
 */

#define USE_DHT
#define USE_NEOPIXEL
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

#define DHT_PIN IO12       // DHT sensörü / DHT sensor
#define SMART_LED_PIN IO14 // Port A

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

const int kColdC = 18;              // Bu ve altı tam MAVİ / at or below: full BLUE
const int kComfortC = 24;           // Tam YEŞİL / full GREEN
const int kHotC = 30;               // Bu ve üstü tam KIRMIZI / at or above: full RED
const int kWarningC = 32;           // Bunun üstünde buzzer uyarısı / above this: buzzer warning
const uint32_t kReadEveryMs = 2000; // DHT11 yavaş bir sensördür / DHT11 is a slow sensor
const uint32_t kBeepEveryMs = 1000;

bool manualMode = false;            // false = gerçek sensör, true = deneme / false = real sensor, true = test
bool muted = false;
int brightnessPct = 25;             // Göz almasın / not too bright
int shownTempC = -999;              // Gösterilen sıcaklık (-999 = okunamadı) / shown temperature (-999 = no reading)
int humidity = -999;
uint32_t lastReadMs = 0, lastBeepMs = 0;

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

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "SICAKLIK" -> "sicaklik"
// Lower-cases and simplifies Turkish letters: "SICAKLIK" -> "sicaklik"
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
// Renk / Color
// ---------------------------------------------------------------------------
// Sıcaklığı renge çevir: mavi -> yeşil -> kırmızı. / Turn a temperature into a color: blue -> green -> red.
void tempToColor(int t, int &r, int &g, int &b) {
  t = constrain(t, kColdC, kHotC);
  if (t <= kComfortC) {                  // Mavi -> yeşil / blue -> green
    g = map(t, kColdC, kComfortC, 0, 255);
    b = 255 - g;
    r = 0;
  } else {                               // Yeşil -> kırmızı / green -> red
    r = map(t, kComfortC, kHotC, 0, 255);
    g = 255 - r;
    b = 0;
  }
}

const char *colorName(int t) {
  if (t <= kColdC + 2) return L("MAVİ (soğuk)", "BLUE (cold)");
  if (t < kComfortC - 1) return L("MAVİ-YEŞİL (serin)", "BLUE-GREEN (cool)");
  if (t <= kComfortC + 1) return L("YEŞİL (rahat)", "GREEN (comfortable)");
  if (t < kHotC - 1) return L("SARI-TURUNCU (ılık)", "YELLOW-ORANGE (warm)");
  return L("KIRMIZI (sıcak)", "RED (hot)");
}

// Rengi güncelle ve rapor yaz / update the color and print a report
void showTemperature() {
  if (shownTempC == -999) {
    minibot.moduleSmartLEDClear();
    minibot.serialWrite(L("Sensör okunamadı - bağlantıyı kontrol edin (IO12).", "Sensor read failed - check the wiring (IO12)."));
    return;
  }
  int r, g, b;
  tempToColor(shownTempC, r, g, b);
  minibot.moduleSmartLEDFill(r, g, b);
  String report = String(manualMode ? L("[DENEME] ", "[TEST] ") : "") +
                  L("Sıcaklık: ", "Temperature: ") + shownTempC + " °C";
  if (!manualMode && humidity != -999) report += String(L(" | Nem: %", " | Humidity: ")) + humidity + L("", " %");
  report += String(L(" | Renk: ", " | Color: ")) + colorName(shownTempC);
  if (shownTempC > kWarningC) report += L("  !!! ÇOK SICAK !!!", "  !!! TOO HOT !!!");
  minibot.serialWrite(report);
}

void readSensor() {
  shownTempC = minibot.moduleDhtTempReadC(DHT_PIN);
  humidity = minibot.moduleDhtHumRead(DHT_PIN);
  showTemperature();
}

// ---------------------------------------------------------------------------
// Mesajlar ve komutlar / Messages and commands
// ---------------------------------------------------------------------------
void printHelp() {
  minibot.serialWrite(L("---- RENKLİ SICAKLIK - Komutlar ----", "---- COLOR TEMPERATURE - Commands ----"));
  minibot.serialWrite(L("  yardim          : bu liste", "  help            : this list"));
  minibot.serialWrite(L("  oto             : gerçek sensör", "  auto            : real sensor"));
  minibot.serialWrite(L("  manuel          : deneme modu", "  manual          : test mode"));
  minibot.serialWrite(L("  sicaklik 10-40  : bu sıcaklığı göster", "  temp 10-40      : show this temperature"));
  minibot.serialWrite(L("  parlaklik 0-100 : LED parlaklığı (%)", "  brightness 0-100: LED brightness (%)"));
  minibot.serialWrite(L("  sessiz          : uyarı sesi kapat/aç", "  mute            : warning beep off/on"));
  minibot.serialWrite(L("  oku             : sensörü hemen oku", "  read            : read the sensor now"));
  minibot.serialWrite(L("  dil             : English'e geç", "  lang            : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu       : OTOMATİK <-> MANUEL", "  B1 button       : AUTO <-> MANUAL"));
}

void setMode(bool manual) {
  manualMode = manual;
  minibot.buzzerPlay(manual ? 1500 : 1000, 60); // Kısa bip / short beep
  minibot.ledWrite(manual);                     // Mavi LED yanıyorsa MANUEL / blue LED on = MANUAL
  if (manual) {
    minibot.serialWrite(L(">> MANUEL (deneme) mod: \"sicaklik 30\" gibi yazın.", ">> MANUAL (test) mode: type e.g. \"temp 30\"."));
  } else {
    minibot.serialWrite(L(">> OTOMATİK mod: gerçek sensör okunuyor.", ">> AUTO mode: reading the real sensor."));
    lastReadMs = millis();
    readSensor();
  }
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oto" || word == "otomatik" || word == "auto") {
    setMode(false);
  } else if (word == "manuel" || word == "manual") {
    setMode(true);
  } else if ((word == "sicaklik" || word == "temp" || word == "temperature") && hasValue) {
    if (!manualMode) setMode(true);
    shownTempC = constrain(value, 10, 40);
    showTemperature();
  } else if ((word == "parlaklik" || word == "brightness") && hasValue) {
    brightnessPct = constrain(value, 0, 100);
    minibot.moduleSmartLEDSetBrightness(brightnessPct * 255 / 100);
    showTemperature(); // Rengi yeni parlaklıkla tekrar yaz / rewrite the color at the new brightness
  } else if (word == "sessiz" || word == "mute") {
    muted = !muted;
    minibot.serialWrite(muted ? L("Uyarı sesi KAPALI.", "Warning beep OFF.") : L("Uyarı sesi AÇIK.", "Warning beep ON."));
  } else if (word == "oku" || word == "read") {
    if (manualMode) setMode(false); // Gerçek sensöre dön / back to the real sensor
    else { lastReadMs = millis(); readSensor(); }
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
  minibot.moduleSmartLEDPrepare(SMART_LED_PIN);
  minibot.moduleSmartLEDSetBrightness(brightnessPct * 255 / 100);
  minibot.serialWrite(L("Renkli sıcaklık göstergesi hazır.", "Color temperature indicator ready."));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) B1 -> mod değiştir / B1 -> toggle mode
  if (b1Pressed()) setMode(!manualMode);

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Otomatik modda her 2 saniyede bir oku, rengi güncelle, rapor yaz.
  // 3) In auto mode, every 2 seconds: read, update the color, print a report.
  if (!manualMode && now - lastReadMs >= kReadEveryMs) {
    lastReadMs = now;
    readSensor();
  }

  // 4) Sınır aşıldıysa her saniye uyarı sesi (arka planda çalar).
  // 4) If the limit is passed, a warning beep every second (plays in the background).
  if (!muted && shownTempC != -999 && shownTempC > kWarningC && now - lastBeepMs >= kBeepEveryMs) {
    lastBeepMs = now;
    minibot.buzzerPlay(2500, 150);
  }
}
