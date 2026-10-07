/*
 * TR: AKILLI LED MODÜLÜ 2 - LED'leri TEK TEK kontrol etmek
 *  - Modülde 3 LED vardır: 1, 2 ve 3 numara. moduleSmartLEDWrite(sıra, R, G, B) ile
 *    her birine ayrı renk verilebilir (sıra 0'dan başlar: 1. LED = 0).
 *  - Açılışta OTOMATİK mod çalışır: 1. LED kırmızı, 2. LED yeşil, 3. LED mavi
 *    yanar (saniyede bir LED), sonra hepsi söner ve baştan başlar.
 *  - B1 butonuna basınca MANUEL moda geçer: LED'ler olduğu gibi kalır, her LED'in
 *    rengini seri porttan siz verirsiniz. B1'e tekrar basınca otomatiğe döner.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help                    -> komut listesi
 *      oto    / auto                    -> otomatik mod
 *      manuel / manual                  -> manuel mod
 *      led 2 255 0 0                    -> 2. LED'i kırmızı yap (R G B, 0-255)
 *      led 2 mavi / led 2 blue          -> 2. LED'e renk adıyla renk ver
 *      renk 0 0 255 / color 0 0 255     -> üç LED'i birden aynı renge boya
 *      kirmizi, yesil, mavi, beyaz / red, green, blue, white -> üçünü o renge boya
 *      kapat / off                      -> hepsini söndür
 *      dil / lang                       -> dili değiştir (Türkçe <-> English)
 *
 * EN: SMART LED MODULE 2 - Controlling the LEDs ONE BY ONE
 *  - The module has 3 LEDs: number 1, 2 and 3. moduleSmartLEDWrite(index, R, G, B)
 *    gives each one its own color (index starts at 0: LED 1 = 0).
 *  - At startup AUTO mode runs: LED 1 red, LED 2 green, LED 3 blue light up
 *    (one LED per second), then all turn off and it starts again.
 *  - Press B1 to switch to MANUAL mode: the LEDs keep their colors and you set each
 *    LED's color from the serial port. Press B1 again to go back to auto.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help / yardim                    -> command list
 *      auto / oto                       -> auto mode
 *      manual / manuel                  -> manual mode
 *      led 2 255 0 0                    -> make LED 2 red (R G B, 0-255)
 *      led 2 blue / led 2 mavi          -> give LED 2 a color by name
 *      color 0 0 255 / renk 0 0 255     -> paint all three LEDs the same color
 *      red, green, blue, white          -> paint all three that color
 *      off / kapat                      -> all off
 *      lang / dil                       -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Akıllı LED modülünü IO12'ye bağlı sokete takın.
 * Desteklenen pinler / Supported pins: IO4 - IO5 - IO12 - IO13 - IO14
 * (IO5 kartın buzzer'ını da sürer / IO5 also drives the board's buzzer.)
 */

#define USE_NEOPIXEL
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

#define LED_PIN IO12 // Akıllı LED'in bağlı olduğu pin / Pin the smart LED is connected to
#define NUM_LEDS 3   // Modüldeki LED sayısı / number of LEDs on the module

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

bool manualMode = false;  // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
int autoStep = 0;         // 0,1,2 = o LED'i yak, 3 = hepsini söndür / 0,1,2 = light that LED, 3 = all off
uint32_t lastStepMs = 0;

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

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "YEŞİL" -> "yesil"
// Lower-cases and simplifies Turkish letters: "YEŞİL" -> "yesil"
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
// Renkler / Colors
// ---------------------------------------------------------------------------
// Renk adını R G B'ye çevirir; bilinmeyen ad ise false döner.
// Turns a color name into R G B; returns false for an unknown name.
bool colorByName(const String &name, int &r, int &g, int &b) {
  if (name == "kirmizi" || name == "red")         { r = 255; g = 0;   b = 0;   return true; }
  if (name == "yesil" || name == "green")         { r = 0;   g = 255; b = 0;   return true; }
  if (name == "mavi" || name == "blue")           { r = 0;   g = 0;   b = 255; return true; }
  if (name == "beyaz" || name == "white")         { r = 255; g = 255; b = 255; return true; }
  if (name == "sari" || name == "yellow")         { r = 255; g = 180; b = 0;   return true; }
  if (name == "mor" || name == "purple")          { r = 150; g = 0;   b = 255; return true; }
  if (name == "siyah" || name == "black" || name == "kapat" || name == "off") { r = 0; g = 0; b = 0; return true; }
  return false;
}

void writeLed(int index, int r, int g, int b) {
  minibot.moduleSmartLEDWrite(index, constrain(r, 0, 255), constrain(g, 0, 255), constrain(b, 0, 255));
}

void fillAll(int r, int g, int b) {
  for (int i = 0; i < NUM_LEDS; i++) writeLed(i, r, g, b);
}

// ---------------------------------------------------------------------------
// Mesajlar ve komutlar / Messages and commands
// ---------------------------------------------------------------------------
void printHelp() {
  minibot.serialWrite(L("---- AKILLI LED 2 - Komutlar ----", "---- SMART LED 2 - Commands ----"));
  minibot.serialWrite(L("  yardim          : bu liste", "  help            : this list"));
  minibot.serialWrite(L("  oto / manuel    : otomatik / manuel mod", "  auto / manual   : auto / manual mode"));
  minibot.serialWrite(L("  led 1-3 R G B   : ör. led 2 255 0 0", "  led 1-3 R G B   : e.g. led 2 255 0 0"));
  minibot.serialWrite(L("  led 1-3 renk    : ör. led 3 mavi", "  led 1-3 name    : e.g. led 3 blue"));
  minibot.serialWrite(L("  renk R G B      : üç LED aynı renk", "  color R G B     : all three the same color"));
  minibot.serialWrite(L("  kirmizi, yesil, mavi, beyaz : üçü birden", "  red, green, blue, white     : all three"));
  minibot.serialWrite(L("  kapat           : hepsini söndür", "  off             : all off"));
  minibot.serialWrite(L("  dil             : English'e geç", "  lang            : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu       : OTOMATİK <-> MANUEL", "  B1 button       : AUTO <-> MANUAL"));
}

void setMode(bool manual) {
  manualMode = manual;
  minibot.buzzerPlay(manual ? 1500 : 1000, 60); // Kısa bip / short beep
  minibot.ledWrite(manual);                     // Mavi LED yanıyorsa MANUEL / blue LED on = MANUAL
  if (manual) {
    minibot.serialWrite(L(">> MANUEL mod: ör. \"led 2 255 0 0\" yazın.", ">> MANUAL mode: type e.g. \"led 2 255 0 0\"."));
  } else {
    minibot.serialWrite(L(">> OTOMATİK mod: LED'ler sırayla yanıyor.", ">> AUTO mode: the LEDs light up in turn."));
    autoStep = 3; // Bir sonraki adımda hepsi söner ve baştan başlar / next step: all off, then restart
    lastStepMs = 0;
  }
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  String args = (space < 0) ? "" : cmd.substring(space + 1);
  int n, r, g, b;
  char name[16];

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oto" || word == "otomatik" || word == "auto") {
    setMode(false);
  } else if (word == "manuel" || word == "manual") {
    setMode(true);
  } else if (word == "led" && sscanf(args.c_str(), "%d %d %d %d", &n, &r, &g, &b) == 4) {
    // "led 2 255 0 0"
    if (n < 1 || n > NUM_LEDS) {
      minibot.serialWrite(L("LED numarası 1, 2 ya da 3 olmalı.", "LED number must be 1, 2 or 3."));
      return;
    }
    if (!manualMode) setMode(true);
    writeLed(n - 1, r, g, b); // Kullanıcı 1'den sayar, kütüphane 0'dan / users count from 1, the library from 0
    minibot.serialWrite(String(n) + L(". LED -> R G B: ", ". LED -> R G B: ") + r + " " + g + " " + b);
  } else if (word == "led" && sscanf(args.c_str(), "%d %15s", &n, name) == 2) {
    // "led 2 mavi"
    if (n < 1 || n > NUM_LEDS || !colorByName(String(name), r, g, b)) {
      minibot.serialWrite(L("Örnek: led 2 mavi  (LED 1-3)", "Example: led 2 blue  (LED 1-3)"));
      return;
    }
    if (!manualMode) setMode(true);
    writeLed(n - 1, r, g, b);
    minibot.serialWrite(String(n) + L(". LED -> ", ". LED -> ") + name);
  } else if ((word == "renk" || word == "color") && sscanf(args.c_str(), "%d %d %d", &r, &g, &b) == 3) {
    if (!manualMode) setMode(true);
    fillAll(r, g, b);
    minibot.serialWrite(String(L("Üç LED -> R G B: ", "All three -> R G B: ")) + r + " " + g + " " + b);
  } else if (colorByName(word, r, g, b)) {
    if (!manualMode) setMode(true);
    fillAll(r, g, b);
    minibot.serialWrite(String(L("Üç LED -> ", "All three -> ")) + word);
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
  minibot.moduleSmartLEDPrepare(LED_PIN); // Akıllı LED'leri hazırla / prepare the smart LEDs
  minibot.serialWrite(L("Akıllı LED testi başladı.", "Smart LED test started."));
  printHelp();
  autoStep = 3;
}

void loop() {
  // 1) B1 -> mod değiştir / B1 -> toggle mode
  if (b1Pressed()) setMode(!manualMode);

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Otomatik mod: her saniye bir adım (delay yok) / Auto mode: one step per second (no delay)
  if (!manualMode && millis() - lastStepMs >= 1000) {
    lastStepMs = millis();
    autoStep = (autoStep + 1) % 4;
    if (autoStep == 0) {
      writeLed(0, 255, 0, 0); // 1. LED: Kırmızı / Red
      minibot.serialWrite(L("1. LED: kırmızı", "LED 1: red"));
    } else if (autoStep == 1) {
      writeLed(1, 0, 255, 0); // 2. LED: Yeşil / Green
      minibot.serialWrite(L("2. LED: yeşil", "LED 2: green"));
    } else if (autoStep == 2) {
      writeLed(2, 0, 0, 255); // 3. LED: Mavi / Blue
      minibot.serialWrite(L("3. LED: mavi", "LED 3: blue"));
    } else {
      minibot.moduleSmartLEDClear(); // Hepsini söndür / all off
      minibot.serialWrite(L("Hepsi söndü", "All off"));
    }
  }
}
