/*
 * TR: NEOPIXEL EFEKTLERİ - Otomatik demo + Manuel kontrol
 *  - Akıllı LED (NeoPixel) için kolaylık fonksiyonlarını gösterir: tek renge boyama
 *    (Fill), söndürme (Clear), parlaklık (SetBrightness), yanıp sönme (Blink) ve
 *    "nefes alma" (Breathe).
 *  - Açılışta OTOMATİK mod çalışır: kırmızı -> yeşil -> sönük -> loş mavi -> parlak
 *    mavi -> 3 kez sarı yanıp sönme -> mor nefes alma -> sönük ... ve baştan.
 *  - B1 butonuna basınca MANUEL moda geçer: demo durur, rengi ve efekti seri porttan
 *    siz seçersiniz. B1'e tekrar basınca otomatik moda döner.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help                    -> komut listesi
 *      oto    / auto                    -> otomatik demo
 *      manuel / manual                  -> manuel mod
 *      kirmizi, yesil, mavi, sari, beyaz, mor / red, green, blue, yellow, white, purple
 *      renk 255 0 0 / color 255 0 0     -> istediğiniz renk (R G B, 0-255)
 *      parlaklik 50 / brightness 50     -> parlaklık (%0-100)
 *      yanip  / blink                   -> son renkle 3 kez yanıp sön (moduleSmartLEDBlink)
 *      nefes  / breathe                 -> son renkle 2 sn nefes al (beklemeden)
 *      kapat  / off                     -> söndür (moduleSmartLEDClear)
 *      dil    / lang                    -> dili değiştir (Türkçe <-> English)
 *    (Renk/efekt komutları otomatik moddaysa manuel moda geçirir.)
 *
 * EN: NEOPIXEL EFFECTS - Automatic demo + Manual control
 *  - Shows the smart LED (NeoPixel) convenience functions: filling with one color
 *    (Fill), turning off (Clear), brightness (SetBrightness), blinking (Blink) and a
 *    "breathing" effect (Breathe).
 *  - At startup AUTO mode runs: red -> green -> off -> dim blue -> bright blue ->
 *    yellow blinks 3 times -> purple breathing -> off ... and again.
 *  - Press B1 to switch to MANUAL mode: the demo stops and you choose the color and
 *    effect from the serial port. Press B1 again to go back to auto mode.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help / yardim                    -> command list
 *      auto / oto                       -> auto demo
 *      manual / manuel                  -> manual mode
 *      red, green, blue, yellow, white, purple
 *      color 255 0 0 / renk 255 0 0     -> any color (R G B, 0-255)
 *      brightness 50 / parlaklik 50     -> brightness (0-100 %)
 *      blink / yanip                    -> blink 3 times in the last color (moduleSmartLEDBlink)
 *      breathe / nefes                  -> breathe for 2 s in the last color (non-blocking)
 *      off / kapat                      -> turn off (moduleSmartLEDClear)
 *      lang / dil                       -> switch language (Turkish <-> English)
 *    (Color/effect commands switch to manual mode if auto mode is running.)
 *
 * Bağlantı / Wiring: Akıllı LED modülünü IO12'ye bağlı sokete takın.
 * Desteklenen pinler / Supported pins: IO4 - IO5 - IO12 - IO13 - IO14
 * (IO5 kartın buzzer'ını da sürer / IO5 also drives the board's buzzer.)
 *
 * NOT / NOTE: moduleSmartLEDBlink efekt bitene kadar (1,2 sn) programı bekletir.
 * Otomatik demo yanıp sönmeyi ve nefes almayı millis() ile kendisi çizer, böylece B1
 * her an çalışır; "yanip" komutu ise kütüphane fonksiyonunu doğrudan çağırır.
 * Nefes alma: parlaklık her karede değişir ve renk YENİDEN yazılır (Fill); sadece
 * parlaklığı değiştirmek, kısılan rengi geri getiremez.
 * moduleSmartLEDBlink blocks the program until the effect ends (1.2 s). The auto demo
 * draws blinking and breathing itself with millis(), so B1 always responds; the
 * "blink" command calls the library function directly.
 * Breathing: the brightness changes every frame and the color is written AGAIN
 * (Fill); changing only the brightness cannot bring a dimmed color back.
 */

#define USE_NEOPIXEL
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

#define LED_PIN IO12 // Akıllı LED'in bağlı olduğu pin / Pin the smart LED is connected to

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

bool manualMode = false;                     // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
int colorR = 150, colorG = 0, colorB = 255;  // Manuel renk / manual color
int brightnessPct = 100;                     // Parlaklık (%) / brightness (%)

// Otomatik demo adımları / auto demo steps
const int STEP_COUNT = 8;
const uint32_t STEP_MS[STEP_COUNT] = {1000, 1000, 1000, 1000, 1000, 1200, 2000, 2000};
int step = 0;
uint32_t stepStartMs = 0;
uint32_t lastFrameMs = 0;
uint32_t breatheStartMs = 0;                 // Manuel nefes efekti (0 = yok) / manual breathe effect (0 = none)

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

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "YANIP" -> "yanip"
// Lower-cases and simplifies Turkish letters: "YANIP" -> "yanip"
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
// Otomatik demo / Auto demo
// ---------------------------------------------------------------------------
void setBrightnessPct(int pct) {
  minibot.moduleSmartLEDSetBrightness(pct * 255 / 100);
}

// Adım başlarken bir kez çağrılır / called once when a step starts
void beginStep() {
  stepStartMs = millis();
  lastFrameMs = 0;
  switch (step) {
    case 0:
      setBrightnessPct(100);
      minibot.moduleSmartLEDFill(255, 0, 0);
      minibot.serialWrite(L("Fill: kırmızı", "Fill: red"));
      break;
    case 1:
      minibot.moduleSmartLEDFill(0, 255, 0);
      minibot.serialWrite(L("Fill: yeşil", "Fill: green"));
      break;
    case 2:
      minibot.moduleSmartLEDClear();
      minibot.serialWrite(L("Clear: söndü", "Clear: off"));
      break;
    case 3:
      setBrightnessPct(15);
      minibot.moduleSmartLEDFill(0, 0, 255);
      minibot.serialWrite(L("Parlaklık: düşük (mavi)", "Brightness: low (blue)"));
      break;
    case 4:
      setBrightnessPct(100);
      minibot.moduleSmartLEDFill(0, 0, 255);
      minibot.serialWrite(L("Parlaklık: yüksek (mavi)", "Brightness: high (blue)"));
      break;
    case 5:
      minibot.serialWrite(L("Yanıp sönme: sarı, 3 kez", "Blink: yellow, 3 times"));
      break;
    case 6:
      minibot.serialWrite(L("Nefes alma: mor", "Breathe: purple"));
      break;
    default:
      setBrightnessPct(100);
      minibot.moduleSmartLEDClear();
      break;
  }
}

// Nefes alma karesi: 1 sn aydınlan, 1 sn karar (elapsed: 0-2000 ms).
// One breathing frame: 1 s brighten, 1 s dim (elapsed: 0-2000 ms).
void breatheFrame(uint32_t elapsed, int r, int g, int b) {
  int level = (elapsed < 1000) ? elapsed * 255 / 1000 : (2000 - elapsed) * 255 / 1000;
  minibot.moduleSmartLEDSetBrightness(constrain(level, 0, 255));
  minibot.moduleSmartLEDFill(r, g, b); // Rengi yeniden yaz / write the color again
}

// Yanıp sönme ve nefes alma adımlarını küçük karelerle çizer (beklemez).
// Draws the blink and breathe steps in small frames (never waits).
void runDemo() {
  uint32_t now = millis();
  uint32_t elapsed = now - stepStartMs;
  if (elapsed >= STEP_MS[step]) {
    step = (step + 1) % STEP_COUNT;
    beginStep();
    return;
  }
  if (now - lastFrameMs < 30) return;
  lastFrameMs = now;

  if (step == 5) {                       // 200 ms yanık, 200 ms sönük / 200 ms on, 200 ms off
    if ((elapsed / 200) % 2 == 0) minibot.moduleSmartLEDFill(255, 255, 0);
    else minibot.moduleSmartLEDClear();
  } else if (step == 6) {
    breatheFrame(elapsed, 150, 0, 255);
  }
}

// ---------------------------------------------------------------------------
// Manuel / Manual
// ---------------------------------------------------------------------------
void showManualColor() {
  breatheStartMs = 0; // Yeni renk/komut nefes efektini bitirir / a new color/command ends the breathe effect
  setBrightnessPct(brightnessPct);
  minibot.moduleSmartLEDFill(colorR, colorG, colorB);
}

void printHelp() {
  minibot.serialWrite(L("---- NEOPIXEL EFEKTLERİ - Komutlar ----", "---- NEOPIXEL EFFECTS - Commands ----"));
  minibot.serialWrite(L("  yardim          : bu liste", "  help            : this list"));
  minibot.serialWrite(L("  oto / manuel    : otomatik demo / manuel mod", "  auto / manual   : auto demo / manual mode"));
  minibot.serialWrite(L("  kirmizi, yesil, mavi, sari, beyaz, mor", "  red, green, blue, yellow, white, purple"));
  minibot.serialWrite(L("  renk R G B      : ör. renk 255 0 128", "  color R G B     : e.g. color 255 0 128"));
  minibot.serialWrite(L("  parlaklik 0-100 : parlaklık (%)", "  brightness 0-100: brightness (%)"));
  minibot.serialWrite(L("  yanip / nefes   : yanıp sön / nefes al", "  blink / breathe : blink / breathe"));
  minibot.serialWrite(L("  kapat           : söndür", "  off             : turn off"));
  minibot.serialWrite(L("  dil             : English'e geç", "  lang            : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu       : OTOMATİK <-> MANUEL", "  B1 button       : AUTO <-> MANUAL"));
}

void setMode(bool manual) {
  manualMode = manual;
  minibot.buzzerPlay(manual ? 1500 : 1000, 60); // Kısa bip / short beep
  minibot.ledWrite(manual);                     // Mavi LED yanıyorsa MANUEL / blue LED on = MANUAL
  if (manual) {
    minibot.serialWrite(L(">> MANUEL mod: renk adı, \"renk R G B\", \"yanip\", \"nefes\" yazın.",
                          ">> MANUAL mode: type a color name, \"color R G B\", \"blink\", \"breathe\"."));
    showManualColor();
  } else {
    minibot.serialWrite(L(">> OTOMATİK mod: demo başlıyor.", ">> AUTO mode: the demo starts."));
    step = 0;
    beginStep();
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
  } else if ((word == "renk" || word == "color") && sscanf(args.c_str(), "%d %d %d", &r, &g, &b) == 3) {
    setColor(r, g, b);
  } else if ((word == "parlaklik" || word == "brightness") && args.length() > 0) {
    brightnessPct = constrain(args.toInt(), 0, 100);
    if (!manualMode) setMode(true);
    showManualColor();
    minibot.serialWrite(String(L("Parlaklık: %", "Brightness: ")) + brightnessPct + L("", " %"));
  } else if (word == "yanip" || word == "blink") {
    if (!manualMode) setMode(true);
    minibot.serialWrite(L("Yanıp sönüyor (1,2 sn)...", "Blinking (1.2 s)..."));
    minibot.moduleSmartLEDBlink(colorR, colorG, colorB, 3, 200); // 3 kez, 200 ms / 3 times, 200 ms
    showManualColor();
  } else if (word == "nefes" || word == "breathe") {
    if (!manualMode) setMode(true);
    minibot.serialWrite(L("Nefes alıyor (2 sn)...", "Breathing (2 s)..."));
    breatheStartMs = millis(); // loop() 2 sn boyunca kareleri çizer / loop() draws the frames for 2 s
  } else if (word == "kapat" || word == "off") {
    if (!manualMode) setMode(true);
    breatheStartMs = 0;
    minibot.moduleSmartLEDClear();
    minibot.serialWrite(L("Söndü.", "Off."));
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
  minibot.moduleSmartLEDPrepare(LED_PIN);
  minibot.serialWrite(L("NeoPixel efektleri hazır.", "NeoPixel effects ready."));
  printHelp();
  beginStep();
}

void loop() {
  // 1) B1 -> mod değiştir / B1 -> toggle mode
  if (b1Pressed()) setMode(!manualMode);

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Otomatik demo (beklemeden) / auto demo (non-blocking)
  if (!manualMode) runDemo();

  // 4) Manuel nefes efekti: 2 sn boyunca 30 ms'de bir kare, sonra rengi geri koy.
  // 4) Manual breathe effect: a frame every 30 ms for 2 s, then restore the color.
  if (manualMode && breatheStartMs != 0 && millis() - lastFrameMs >= 30) {
    lastFrameMs = millis();
    uint32_t elapsed = millis() - breatheStartMs;
    if (elapsed >= 2000) {
      breatheStartMs = 0;
      showManualColor();
    } else {
      breatheFrame(elapsed, colorR, colorG, colorB);
    }
  }
}
