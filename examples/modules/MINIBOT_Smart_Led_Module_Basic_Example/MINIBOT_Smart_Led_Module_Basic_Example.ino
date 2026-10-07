/*
 * TR: AKILLI LED (NeoPixel) MODÜLÜ - Otomatik efektler + Manuel renk kontrolü
 *  - Açılışta OTOMATİK mod çalışır: 3 LED'lik modülde sırayla 4 efekt oynar
 *    (her biri ~5 sn): gökkuşağı, gökkuşağı takip, kırmızı takip, mavi silme.
 *  - B1 butonuna basınca MANUEL moda geçer: efekt durur, rengi seri porttan siz
 *    seçersiniz. B1'e tekrar basınca otomatik moda döner.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help                 -> komut listesi
 *      oto    / auto                 -> otomatik mod
 *      manuel / manual               -> manuel mod
 *      kirmizi, yesil, mavi, sari, beyaz, mor, turuncu
 *      red, green, blue, yellow, white, purple, orange  -> o renge boya
 *      renk 255 0 0 / color 255 0 0  -> istediğiniz renk (R G B, 0-255)
 *      parlaklik 50 / brightness 50  -> parlaklık (%0-100)
 *      ac / on                       -> son rengi yak
 *      kapat / off                   -> LED'leri söndür
 *      dil / lang                    -> dili değiştir (Türkçe <-> English)
 *    (Renk komutları otomatik moddaysa manuel moda geçirir.)
 *
 * EN: SMART LED (NeoPixel) MODULE - Automatic effects + Manual color control
 *  - At startup AUTO mode runs: 4 effects play in turn on the 3-LED module
 *    (~5 s each): rainbow, rainbow chase, red chase, blue wipe.
 *  - Press B1 to switch to MANUAL mode: the effect stops and you choose the color
 *    from the serial port. Press B1 again to go back to auto mode.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help / yardim                 -> command list
 *      auto / oto                    -> auto mode
 *      manual / manuel               -> manual mode
 *      red, green, blue, yellow, white, purple, orange  -> paint that color
 *      color 255 0 0 / renk 255 0 0  -> any color (R G B, 0-255)
 *      brightness 50 / parlaklik 50  -> brightness (0-100 %)
 *      on / ac                       -> light the last color
 *      off / kapat                   -> turn the LEDs off
 *      lang / dil                    -> switch language (Turkish <-> English)
 *    (Color commands switch to manual mode if auto mode is running.)
 *
 * Bağlantı / Wiring: Akıllı LED modülünü IO12'ye bağlı sokete takın.
 * Desteklenen pinler / Supported pins: IO4 - IO5 - IO12 - IO13 - IO14
 * (IO5 kartın buzzer'ını da sürer / IO5 also drives the board's buzzer.)
 *
 * NOT / NOTE: Kütüphanenin hazır efekt fonksiyonları (ör. moduleSmartLEDRainbowEffect)
 * bitene kadar programı bekletir. Bu örnek efektleri küçük adımlarla (millis) kendisi
 * çizer; böylece B1 ve seri komutlar her an çalışır.
 * The library's ready-made effect functions (e.g. moduleSmartLEDRainbowEffect) block
 * until they finish. This example draws the effects itself in small steps (millis),
 * so B1 and serial commands always respond.
 */

#define USE_NEOPIXEL
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

#define LED_PIN IO12 // Akıllı LED'in bağlı olduğu pin / Pin the smart LED is connected to
#define NUM_LEDS 3   // moduleSmartLEDPrepare 3 LED'lik modülü hazırlar / prepares the 3-LED module

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

bool manualMode = false;            // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
int colorR = 255, colorG = 255, colorB = 255; // Manuel renk / manual color
int brightnessPct = 50;             // Parlaklık (%) / brightness (%)

// Otomatik efekt durumu / auto effect state
int effect = 0;                     // 0..3
uint32_t effectStartMs = 0;
uint32_t lastFrameMs = 0;
int frame = 0;
const uint32_t EFFECT_MS = 5000;    // Her efekt 5 sn / each effect 5 s

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

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "SARI" -> "sari"
// Lower-cases and simplifies Turkish letters: "SARI" -> "sari"
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
// Renk yardımcıları / Color helpers
// ---------------------------------------------------------------------------
// 0-255 arası bir sayıyı gökkuşağı rengine çevirir / turns 0-255 into a rainbow color
void wheel(uint8_t pos, int &r, int &g, int &b) {
  pos = 255 - pos;
  if (pos < 85) {
    r = 255 - pos * 3; g = 0; b = pos * 3;
  } else if (pos < 170) {
    pos -= 85;
    r = 0; g = pos * 3; b = 255 - pos * 3;
  } else {
    pos -= 170;
    r = pos * 3; g = 255 - pos * 3; b = 0;
  }
}

void applyBrightness() {
  minibot.moduleSmartLEDSetBrightness(brightnessPct * 255 / 100);
}

void showManualColor() {
  applyBrightness(); // Parlaklığı önce ayarla, sonra rengi yaz / set brightness first, then write the color
  minibot.moduleSmartLEDFill(colorR, colorG, colorB);
}

const char *effectName(int e) {
  switch (e) {
    case 0: return L("Gökkuşağı", "Rainbow");
    case 1: return L("Gökkuşağı takip", "Rainbow chase");
    case 2: return L("Kırmızı takip", "Red chase");
    default: return L("Mavi silme", "Blue wipe");
  }
}

void startEffect(int e) {
  effect = e;
  frame = 0;
  effectStartMs = millis();
  lastFrameMs = 0;
  minibot.moduleSmartLEDClear();
  minibot.serialWrite(String(L("Efekt: ", "Effect: ")) + effectName(effect));
}

// Her çağrıda efektin SADECE bir karesini çizer (beklemez).
// Draws ONLY one frame of the effect per call (never waits).
void runEffect() {
  uint32_t now = millis();
  if (now - effectStartMs >= EFFECT_MS) startEffect((effect + 1) % 4);

  uint32_t frameMs = (effect == 0) ? 20 : (effect == 3 ? 300 : 150);
  if (now - lastFrameMs < frameMs) return;
  lastFrameMs = now;
  frame++;

  int r, g, b;
  if (effect == 0) {                      // Gökkuşağı: renkler kayar / rainbow: colors slide
    for (int i = 0; i < NUM_LEDS; i++) {
      wheel((uint8_t)(frame * 3 + i * 85), r, g, b);
      minibot.moduleSmartLEDWrite(i, r, g, b);
    }
  } else if (effect == 1 || effect == 2) { // Takip: tek LED yanar ve döner / chase: one LED lit, moving
    int lit = frame % NUM_LEDS;
    if (effect == 1) wheel((uint8_t)(frame * 20), r, g, b);
    else { r = 255; g = 0; b = 0; }
    for (int i = 0; i < NUM_LEDS; i++) {
      if (i == lit) minibot.moduleSmartLEDWrite(i, r, g, b);
      else minibot.moduleSmartLEDWrite(i, 0, 0, 0);
    }
  } else {                                // Silme: LED'ler tek tek maviye boyanır / wipe: LEDs turn blue one by one
    int count = frame % (NUM_LEDS + 2);   // 0..4: 3 LED dolar, sonra kısa ara / 3 LEDs fill, then a short gap
    for (int i = 0; i < NUM_LEDS; i++) {
      if (i < count && count <= NUM_LEDS) minibot.moduleSmartLEDWrite(i, 0, 0, 255);
      else minibot.moduleSmartLEDWrite(i, 0, 0, 0);
    }
  }
}

// ---------------------------------------------------------------------------
// Mesajlar ve komutlar / Messages and commands
// ---------------------------------------------------------------------------
void printHelp() {
  minibot.serialWrite(L("---- AKILLI LED - Komutlar ----", "---- SMART LED - Commands ----"));
  minibot.serialWrite(L("  yardim          : bu liste", "  help            : this list"));
  minibot.serialWrite(L("  oto             : otomatik efektler", "  auto            : automatic effects"));
  minibot.serialWrite(L("  manuel          : manuel mod", "  manual          : manual mode"));
  minibot.serialWrite(L("  kirmizi, yesil, mavi, sari, beyaz, mor, turuncu", "  red, green, blue, yellow, white, purple, orange"));
  minibot.serialWrite(L("  renk R G B      : ör. renk 255 0 128", "  color R G B     : e.g. color 255 0 128"));
  minibot.serialWrite(L("  parlaklik 0-100 : parlaklık (%)", "  brightness 0-100: brightness (%)"));
  minibot.serialWrite(L("  ac / kapat      : yak / söndür", "  on / off        : light / turn off"));
  minibot.serialWrite(L("  dil             : English'e geç", "  lang            : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu       : OTOMATİK <-> MANUEL", "  B1 button       : AUTO <-> MANUAL"));
}

void setMode(bool manual) {
  manualMode = manual;
  minibot.buzzerPlay(manual ? 1500 : 1000, 60); // Kısa bip / short beep
  minibot.ledWrite(manual);                     // Mavi LED yanıyorsa MANUEL / blue LED on = MANUAL
  if (manual) {
    minibot.serialWrite(L(">> MANUEL mod: renk adı ya da \"renk R G B\" yazın.", ">> MANUAL mode: type a color name or \"color R G B\"."));
    showManualColor();
  } else {
    minibot.serialWrite(L(">> OTOMATİK mod: efektler sırayla oynuyor.", ">> AUTO mode: the effects play in turn."));
    applyBrightness();
    startEffect(0);
  }
}

void setColor(int r, int g, int b) {
  if (!manualMode) setMode(true);
  colorR = constrain(r, 0, 255);
  colorG = constrain(g, 0, 255);
  colorB = constrain(b, 0, 255);
  showManualColor();
  minibot.serialWrite(String(L("Renk (R G B): ", "Color (R G B): ")) + colorR + " " + colorG + " " + colorB);
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  String args = (space < 0) ? "" : cmd.substring(space + 1);
  int r, g, b;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oto" || word == "otomatik" || word == "auto") {
    setMode(false);
  } else if (word == "manuel" || word == "manual") {
    setMode(true);
  } else if (word == "kirmizi" || word == "red") {
    setColor(255, 0, 0);
  } else if (word == "yesil" || word == "green") {
    setColor(0, 255, 0);
  } else if (word == "mavi" || word == "blue") {
    setColor(0, 0, 255);
  } else if (word == "sari" || word == "yellow") {
    setColor(255, 180, 0);
  } else if (word == "beyaz" || word == "white") {
    setColor(255, 255, 255);
  } else if (word == "mor" || word == "purple") {
    setColor(150, 0, 255);
  } else if (word == "turuncu" || word == "orange") {
    setColor(255, 80, 0);
  } else if ((word == "renk" || word == "color") && sscanf(args.c_str(), "%d %d %d", &r, &g, &b) == 3) {
    setColor(r, g, b);
  } else if ((word == "parlaklik" || word == "brightness") && args.length() > 0) {
    brightnessPct = constrain(args.toInt(), 0, 100);
    if (manualMode) showManualColor();
    else applyBrightness();
    minibot.serialWrite(String(L("Parlaklık: %", "Brightness: ")) + brightnessPct + L("", " %"));
  } else if (word == "ac" || word == "on") {
    setColor(colorR, colorG, colorB);
  } else if (word == "kapat" || word == "off") {
    if (!manualMode) setMode(true);
    minibot.moduleSmartLEDClear();
    minibot.serialWrite(L("LED'ler söndü.", "LEDs off."));
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
  applyBrightness();
  minibot.serialWrite(L("Akıllı LED testi başladı.", "Smart LED test started."));
  printHelp();
  startEffect(0);
}

void loop() {
  // 1) B1 -> mod değiştir / B1 -> toggle mode
  if (b1Pressed()) setMode(!manualMode);

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Otomatik modda efektin bir sonraki karesi / next effect frame in auto mode
  if (!manualMode) runEffect();
}
