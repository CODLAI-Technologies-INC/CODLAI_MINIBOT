/*
 * TR: TRAFİK IŞIĞI MODÜLÜ - Otomatik demo + Manuel kontrol
 *  - Açılışta OTOMATİK mod çalışır: KIRMIZI (3 sn) -> SARI (1 sn) -> YEŞİL (3 sn)
 *    -> SARI (1 sn) -> tekrar KIRMIZI ... gerçek bir kavşak gibi.
 *  - B1 butonuna basınca MANUEL moda geçer: ışık olduğu gibi kalır, hangi ışığın
 *    yanacağını seri porttan siz seçersiniz. B1'e tekrar basınca otomatiğe döner.
 *  - Mod değişince mavi LED kısa bir an yanıp söner (bu modülde buzzer kullanılmaz,
 *    çünkü buzzer ile SARI ışık aynı pini (IO5) paylaşır).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim  / help      -> komut listesi
 *      oto     / auto      -> otomatik mod
 *      manuel  / manual    -> manuel mod
 *      kirmizi / red       -> sadece kırmızı (manuel moda geçer)
 *      sari    / yellow    -> sadece sarı
 *      yesil   / green     -> sadece yeşil
 *      hepsi   / all       -> üçü birden (lamba testi)
 *      kapat   / off       -> hepsini söndür
 *      dil     / lang      -> dili değiştir (Türkçe <-> English)
 *
 * EN: TRAFFIC LIGHT MODULE - Automatic demo + Manual control
 *  - At startup AUTO mode runs: RED (3 s) -> YELLOW (1 s) -> GREEN (3 s)
 *    -> YELLOW (1 s) -> RED again ... like a real junction.
 *  - Press B1 to switch to MANUAL mode: the lights keep their state and you choose
 *    which light is on from the serial port. Press B1 again to go back to auto.
 *  - On a mode change the blue LED flashes briefly (the buzzer is not used here,
 *    because the buzzer and the YELLOW light share the same pin (IO5)).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help    / yardim    -> command list
 *      auto    / oto       -> auto mode
 *      manual  / manuel    -> manual mode
 *      red     / kirmizi   -> red only (switches to manual)
 *      yellow  / sari      -> yellow only
 *      green   / yesil     -> green only
 *      all     / hepsi     -> all three (lamp test)
 *      off     / kapat     -> all off
 *      lang    / dil       -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Trafik ışığı modülü SABİT pinler kullanır: KIRMIZI = IO13,
 * SARI = IO5, YEŞİL = IO4. Soket seçmenize gerek yok.
 * The traffic light module uses FIXED pins: RED = IO13, YELLOW = IO5, GREEN = IO4.
 * No socket to choose.
 */

#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// Otomatik döngünün adımları / steps of the automatic cycle
const int STEP_COUNT = 4;
const bool STEP_RED[STEP_COUNT]    = {true,  false, false, false};
const bool STEP_YELLOW[STEP_COUNT] = {false, true,  false, true};
const bool STEP_GREEN[STEP_COUNT]  = {false, false, true,  false};
const uint32_t STEP_MS[STEP_COUNT] = {3000,  1000,  3000,  1000};

bool manualMode = false;    // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
int step = 0;               // Otomatik döngüdeki adım / step in the auto cycle
uint32_t stepStartMs = 0;   // Adımın başladığı an / when the step started
uint32_t ledOffAtMs = 0;    // Mavi LED'in sönme zamanı / when to turn the blue LED off

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

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "KIRMIZI" -> "kirmizi"
// Lower-cases and simplifies Turkish letters: "KIRMIZI" -> "kirmizi"
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
// Işıklar ve mesajlar / Lights and messages
// ---------------------------------------------------------------------------
void setLights(bool red, bool yellow, bool green) {
  minibot.moduleTraficLightWrite(red, yellow, green);
  String text = L("Işık: ", "Light: ");
  if (!red && !yellow && !green) text += L("hepsi sönük", "all off");
  if (red) text += L("KIRMIZI ", "RED ");
  if (yellow) text += L("SARI ", "YELLOW ");
  if (green) text += L("YEŞİL", "GREEN");
  minibot.serialWrite(text);
}

void showStep() {
  setLights(STEP_RED[step], STEP_YELLOW[step], STEP_GREEN[step]);
  stepStartMs = millis();
}

void printHelp() {
  minibot.serialWrite(L("---- TRAFİK IŞIĞI - Komutlar ----", "---- TRAFFIC LIGHT - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  oto           : otomatik mod", "  auto          : auto mode"));
  minibot.serialWrite(L("  manuel        : manuel mod", "  manual        : manual mode"));
  minibot.serialWrite(L("  kirmizi / sari / yesil : o ışığı yak", "  red / yellow / green  : light that one"));
  minibot.serialWrite(L("  hepsi         : üçü birden", "  all           : all three"));
  minibot.serialWrite(L("  kapat         : hepsini söndür", "  off           : all off"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu     : OTOMATİK <-> MANUEL", "  B1 button     : AUTO <-> MANUAL"));
}

void setMode(bool manual) {
  manualMode = manual;
  // Buzzer yerine mavi LED ile geri bildirim (buzzer IO5 = SARI ışık).
  // Feedback with the blue LED instead of the buzzer (buzzer IO5 = YELLOW light).
  minibot.ledWrite(true);
  ledOffAtMs = millis() + 150;
  if (manual) {
    minibot.serialWrite(L(">> MANUEL mod: \"kirmizi\", \"sari\", \"yesil\" yazın.", ">> MANUAL mode: type \"red\", \"yellow\", \"green\"."));
  } else {
    minibot.serialWrite(L(">> OTOMATİK mod: ışıklar sırayla değişiyor.", ">> AUTO mode: the lights change in turn."));
    step = 0;
    showStep();
  }
}

void manualLights(bool red, bool yellow, bool green) {
  if (!manualMode) setMode(true);
  setLights(red, yellow, green);
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "oto" || cmd == "otomatik" || cmd == "auto") {
    setMode(false);
  } else if (cmd == "manuel" || cmd == "manual") {
    setMode(true);
  } else if (cmd == "kirmizi" || cmd == "red") {
    manualLights(true, false, false);
  } else if (cmd == "sari" || cmd == "yellow") {
    manualLights(false, true, false);
  } else if (cmd == "yesil" || cmd == "green") {
    manualLights(false, false, true);
  } else if (cmd == "hepsi" || cmd == "all") {
    manualLights(true, true, true);
  } else if (cmd == "kapat" || cmd == "off") {
    manualLights(false, false, false);
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
  minibot.serialWrite(L("Trafik ışığı testi başladı.", "Traffic light test started."));
  printHelp();
  showStep();
}

void loop() {
  // 1) B1 -> mod değiştir / B1 -> toggle mode
  if (b1Pressed()) setMode(!manualMode);

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Otomatik mod: adımın süresi dolunca sonrakine geç (delay yok).
  // 3) Auto mode: go to the next step when its time is up (no delay).
  if (!manualMode && millis() - stepStartMs >= STEP_MS[step]) {
    step = (step + 1) % STEP_COUNT;
    showStep();
  }

  // 4) Mavi LED geri bildirimini söndür / turn the blue LED feedback off
  if (ledOffAtMs != 0 && (int32_t)(millis() - ledOffAtMs) >= 0) {
    ledOffAtMs = 0;
    minibot.ledWrite(false);
  }
}
