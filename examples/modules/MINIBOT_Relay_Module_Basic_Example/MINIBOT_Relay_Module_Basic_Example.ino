/*
 * TR: RÖLE MODÜLÜ - Otomatik demo + Manuel kontrol
 *  - Açılışta OTOMATİK mod çalışır: röle 3 saniyede bir açılıp kapanır (tık sesi duyarsınız).
 *  - B1 butonuna basınca MANUEL moda geçer: röle olduğu gibi kalır, seri porttan
 *    "ac" / "kapat" yazarak siz kontrol edersiniz. B1'e tekrar basınca otomatiğe döner.
 *  - Mavi LED rölenin durumunu gösterir (yanıyor = röle açık).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help        -> komut listesi
 *      oto    / auto        -> otomatik mod
 *      manuel / manual      -> manuel mod
 *      ac     / on          -> röleyi aç (manuel moda geçer)
 *      kapat  / off         -> röleyi kapat (manuel moda geçer)
 *      degistir / toggle    -> röleyi tersine çevir (manuel moda geçer)
 *      sure 5 / period 5    -> otomatik moddaki açma/kapama süresi (saniye, 1-60)
 *      dil    / lang        -> dili değiştir (Türkçe <-> English)
 *
 * EN: RELAY MODULE - Automatic demo + Manual control
 *  - At startup AUTO mode runs: the relay switches on/off every 3 seconds (you hear it click).
 *  - Press B1 to switch to MANUAL mode: the relay keeps its state and you control it
 *    from the serial port with "on" / "off". Press B1 again to go back to auto.
 *  - The blue LED shows the relay state (on = relay on).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim      -> command list
 *      auto   / oto         -> auto mode
 *      manual / manuel      -> manual mode
 *      on     / ac          -> relay on (switches to manual)
 *      off    / kapat       -> relay off (switches to manual)
 *      toggle / degistir    -> invert the relay (switches to manual)
 *      period 5 / sure 5    -> on/off time in auto mode (seconds, 1-60)
 *      lang   / dil         -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Röle modülünü IO12'ye bağlı sokete takın.
 * Desteklenen pinler / Supported pins: IO4 - IO5 - IO12 - IO13 - IO14
 * (IO5 kartın buzzer'ını da sürer / IO5 also drives the board's buzzer.)
 */

#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

#define RELAY_PIN IO12 // Rölenin bağlı olduğu pin / Pin the relay is connected to

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

bool manualMode = false;      // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
bool relayOn = false;         // Rölenin durumu / relay state
uint32_t periodMs = 3000;     // Otomatik moddaki süre / time per state in auto mode
uint32_t lastSwitchMs = 0;    // Son değişim zamanı / time of the last switch

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

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "AÇ" -> "ac"
// Lower-cases and simplifies Turkish letters: "AÇ" -> "ac"
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
// Röle ve mesajlar / Relay and messages
// ---------------------------------------------------------------------------
void setRelay(bool on) {
  relayOn = on;
  minibot.moduleRelayWrite(RELAY_PIN, on);
  minibot.ledWrite(on); // Mavi LED röleyi gösterir / blue LED mirrors the relay
  minibot.serialWrite(on ? L("Röle AÇIK", "Relay ON") : L("Röle KAPALI", "Relay OFF"));
}

void printHelp() {
  minibot.serialWrite(L("---- RÖLE - Komutlar ----", "---- RELAY - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  oto           : otomatik mod", "  auto          : auto mode"));
  minibot.serialWrite(L("  manuel        : manuel mod", "  manual        : manual mode"));
  minibot.serialWrite(L("  ac / kapat    : röleyi aç / kapat", "  on / off      : relay on / off"));
  minibot.serialWrite(L("  degistir      : röleyi tersine çevir", "  toggle        : invert the relay"));
  minibot.serialWrite(L("  sure 1-60     : otomatik süre (saniye)", "  period 1-60   : auto time (seconds)"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu     : OTOMATİK <-> MANUEL", "  B1 button     : AUTO <-> MANUAL"));
}

void setMode(bool manual) {
  manualMode = manual;
  lastSwitchMs = millis();
  minibot.buzzerPlay(manual ? 1500 : 1000, 60); // Kısa bip / short beep
  minibot.serialWrite(manual ? L(">> MANUEL mod: \"ac\" / \"kapat\" yazın.", ">> MANUAL mode: type \"on\" / \"off\".")
                             : L(">> OTOMATİK mod: röle kendi kendine açılıp kapanıyor.", ">> AUTO mode: the relay switches by itself."));
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
  } else if (word == "ac" || word == "on") {
    if (!manualMode) setMode(true);
    setRelay(true);
  } else if (word == "kapat" || word == "off") {
    if (!manualMode) setMode(true);
    setRelay(false);
  } else if (word == "degistir" || word == "toggle") {
    if (!manualMode) setMode(true);
    setRelay(!relayOn);
  } else if ((word == "sure" || word == "period") && hasValue) {
    periodMs = (uint32_t)constrain(value, 1, 60) * 1000;
    minibot.serialWrite(String(L("Otomatik süre: ", "Auto period: ")) + (periodMs / 1000) + L(" saniye", " seconds"));
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
  minibot.serialWrite(L("Röle testi başladı.", "Relay test started."));
  printHelp();
  setRelay(false);             // Röle kapalı başlar / the relay starts off
  lastSwitchMs = millis();
}

void loop() {
  // 1) B1 -> mod değiştir / B1 -> toggle mode
  if (b1Pressed()) setMode(!manualMode);

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Otomatik mod: süre dolunca röleyi tersine çevir (delay yok, B1 hemen çalışır).
  // 3) Auto mode: invert the relay when the time is up (no delay, B1 reacts instantly).
  if (!manualMode && millis() - lastSwitchMs >= periodMs) {
    lastSwitchMs = millis();
    setRelay(!relayOn);
  }
}
