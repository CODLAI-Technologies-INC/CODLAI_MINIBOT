/*
 * TR: GERÇEK PROJE - Butonlu Yaya Geçidi
 *  - OTOMATİK modda arabalar için ışık normalde YEŞİLDİR. Yaya B1 butonuna basınca
 *    istek kaydedilir (mavi LED yanar = "BEKLEYİNİZ"). Arabalar en az 8 saniye yeşil
 *    görmeden ışık değişmez; sonra SARI (2 sn) -> KIRMIZI (6 sn, yayalar geçer) ->
 *    tekrar YEŞİL. Kırmızı sırasında buzzer "tık tık" yapar, son 2 saniyede hızlanır
 *    - tıpkı görme engelliler için sesli yaya geçitleri gibi.
 *  - Kod hiç beklemeden (millis ile) çalışır, buton her an cevap verir.
 *  - B1 bu projede YAYA butonudur. MANUEL mod (ışıkları elle seçmek) seri porttan
 *    "manuel" yazarak açılır, "oto" ile otomatiğe dönülür.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim  / help     -> komut listesi
 *      bas     / press    -> yaya butonuna basmak gibi (geçiş isteği)
 *      oto     / auto     -> otomatik yaya geçidi
 *      manuel  / manual   -> manuel mod (ışıklar seri porttan)
 *      kirmizi / red, sari / yellow, yesil / green, kapat / off -> ışık seç (manuele geçer)
 *      durum   / status   -> şu anki evre
 *      dil     / lang     -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Push-Button Pedestrian Crossing
 *  - In AUTO mode the car light is normally GREEN. When a pedestrian presses B1 the
 *    request is stored (blue LED on = "PLEASE WAIT"). The light never changes before
 *    cars have had at least 8 seconds of green; then YELLOW (2 s) -> RED (6 s,
 *    pedestrians cross) -> GREEN again. During red the buzzer ticks, faster in the
 *    last 2 seconds - just like talking crossings for visually impaired people.
 *  - The code never blocks (uses millis), so the button always responds.
 *  - In this project B1 is the PEDESTRIAN button. MANUAL mode (choosing the lights by
 *    hand) is turned on by typing "manual", and "auto" goes back to automatic.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help    / yardim   -> command list
 *      press   / bas      -> same as pressing the pedestrian button
 *      auto    / oto      -> automatic pedestrian crossing
 *      manual  / manuel   -> manual mode (lights from serial)
 *      red / kirmizi, yellow / sari, green / yesil, off / kapat -> choose a light (switches to manual)
 *      status  / durum    -> current phase
 *      lang    / dil      -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Trafik ışığı modülü SABİT pinler kullanır (KIRMIZI = IO13,
 * SARI = IO5, YEŞİL = IO4) - soket seçmenize gerek yok. / The traffic light module
 * uses FIXED pins (RED = IO13, YELLOW = IO5, GREEN = IO4) - no socket to choose.
 * NOT: Kartın buzzer'ı da IO5'i (SARI ışık) kullanır. Bu yüzden tıklar sadece
 * KIRMIZI evrede (sarı sönükken) çalınır; her tıkta sarı ışık hafifçe parlarsa bu
 * normaldir. / NOTE: The board's buzzer also uses IO5 (YELLOW light). So ticks are
 * only played in the RED phase (while yellow is off); if the yellow light glows
 * faintly on each tick, that is normal.
 */

#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

const uint32_t kMinGreenMs = 8000;      // Arabalar için en kısa yeşil / shortest green for cars
const uint32_t kYellowMs = 2000;        // Sarı süresi / yellow time
const uint32_t kRedMs = 6000;           // Yaya geçiş süresi / pedestrian crossing time
const uint32_t kFastTickLastMs = 2000;  // Son 2 sn hızlı tık / fast ticks in the last 2 s

enum Phase { CAR_GREEN, CAR_YELLOW, CAR_RED };
Phase phase = CAR_GREEN;
uint32_t phaseStartMs = 0;
bool requested = false;                 // Yaya isteği var mı / pedestrian request waiting?
bool manualMode = false;                // true = ışıklar seri porttan / lights from serial
uint32_t lastTickMs = 0;

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

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "BAS" -> "bas", "SARI" -> "sari"
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
// Evreler ve mesajlar / Phases and messages
// ---------------------------------------------------------------------------
void printPhase() {
  if (phase == CAR_GREEN) minibot.serialWrite(L("Arabalar: YEŞİL (yaya bekler)", "Cars: GREEN (pedestrians wait)"));
  else if (phase == CAR_YELLOW) minibot.serialWrite(L("Arabalar: SARI (yavaşla!)", "Cars: YELLOW (slow down!)"));
  else minibot.serialWrite(L("Arabalar: KIRMIZI -> Yayalar GEÇEBİLİR", "Cars: RED -> Pedestrians may CROSS"));
}

void enterPhase(Phase p) {
  phase = p;
  phaseStartMs = millis();
  // Işığı SADECE evre değişince yaz: sürekli yazmak IO5'teki buzzer sesini keserdi.
  // Write the light ONLY on a phase change: writing it constantly would cut the buzzer tone on IO5.
  if (p == CAR_GREEN) {
    minibot.moduleTraficLightWrite(false, false, true);
  } else if (p == CAR_YELLOW) {
    minibot.moduleTraficLightWrite(false, true, false);
  } else {
    minibot.moduleTraficLightWrite(true, false, false);
    requested = false;
    minibot.ledWrite(false);
    lastTickMs = 0;
  }
  printPhase();
}

void pedestrianRequest() {
  if (manualMode) {
    minibot.serialWrite(L("Manuel moddayız - yaya isteği için önce \"oto\" yazın.", "Manual mode - type \"auto\" first for pedestrian requests."));
  } else if (phase != CAR_RED && !requested) {
    requested = true;
    minibot.ledWrite(true); // "BEKLEYİNİZ" ışığı / "PLEASE WAIT" light
    minibot.serialWrite(L("İstek alındı - BEKLEYİNİZ...", "Request received - PLEASE WAIT..."));
  }
}

void setManual(bool manual) {
  manualMode = manual;
  requested = false;
  minibot.ledWrite(false);
  if (manual) {
    minibot.serialWrite(L(">> MANUEL mod: \"kirmizi\", \"sari\", \"yesil\", \"kapat\" yazın.", ">> MANUAL mode: type \"red\", \"yellow\", \"green\", \"off\"."));
  } else {
    minibot.serialWrite(L(">> OTOMATİK mod: yaya geçidi çalışıyor.", ">> AUTO mode: the pedestrian crossing is running."));
    enterPhase(CAR_GREEN);
  }
}

void manualLights(bool red, bool yellow, bool green, const char *nameTr, const char *nameEn) {
  if (!manualMode) setManual(true);
  minibot.moduleTraficLightWrite(red, yellow, green);
  minibot.serialWrite(String(L("Işık: ", "Light: ")) + L(nameTr, nameEn));
}

void printHelp() {
  minibot.serialWrite(L("---- YAYA GEÇİDİ - Komutlar ----", "---- PEDESTRIAN CROSSING - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  bas           : yaya isteği (B1 gibi)", "  press         : pedestrian request (like B1)"));
  minibot.serialWrite(L("  oto / manuel  : otomatik / manuel mod", "  auto / manual : auto / manual mode"));
  minibot.serialWrite(L("  kirmizi / sari / yesil / kapat : ışık seç", "  red / yellow / green / off     : choose a light"));
  minibot.serialWrite(L("  durum         : şu anki evre", "  status        : current phase"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu     : yaya butonu", "  B1 button     : pedestrian button"));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "bas" || cmd == "press" || cmd == "istek" || cmd == "request") {
    pedestrianRequest();
  } else if (cmd == "oto" || cmd == "otomatik" || cmd == "auto") {
    setManual(false);
  } else if (cmd == "manuel" || cmd == "manual") {
    setManual(true);
  } else if (cmd == "kirmizi" || cmd == "red") {
    manualLights(true, false, false, "KIRMIZI", "RED");
  } else if (cmd == "sari" || cmd == "yellow") {
    manualLights(false, true, false, "SARI", "YELLOW");
  } else if (cmd == "yesil" || cmd == "green") {
    manualLights(false, false, true, "YEŞİL", "GREEN");
  } else if (cmd == "kapat" || cmd == "off") {
    manualLights(false, false, false, "hepsi sönük", "all off");
  } else if (cmd == "durum" || cmd == "status") {
    if (manualMode) minibot.serialWrite(L("Mod: MANUEL", "Mode: MANUAL"));
    else {
      printPhase();
      if (requested) minibot.serialWrite(L("Bekleyen yaya isteği var.", "A pedestrian request is waiting."));
    }
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
  minibot.serialWrite(L("Yaya geçidi hazır. Karşıya geçmek için B1'e basın.", "Pedestrian crossing ready. Press B1 to cross."));
  printHelp();
  enterPhase(CAR_GREEN);
}

void loop() {
  uint32_t now = millis();

  // 1) Yaya butonu (sadece yeşil/sarı evrede istek kaydedilir).
  // 1) Pedestrian button (a request is stored only in the green/yellow phase).
  if (b1Pressed()) pedestrianRequest();

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  if (manualMode) return; // Manuelde ışıklar komutla değişir / in manual the lights change by command

  // 3) Evre makinesi / phase machine
  uint32_t inPhase = now - phaseStartMs;
  switch (phase) {
    case CAR_GREEN:
      // İstek var VE arabalar en az 8 sn yeşil gördüyse sarıya geç.
      // Request waiting AND cars have had at least 8 s of green -> go yellow.
      if (requested && inPhase >= kMinGreenMs) enterPhase(CAR_YELLOW);
      break;

    case CAR_YELLOW:
      if (inPhase >= kYellowMs) enterPhase(CAR_RED);
      break;

    case CAR_RED: {
      // Normalde saniyede 1 tık, son 2 saniyede saniyede 4 tık.
      // Normally 1 tick per second, 4 ticks per second in the last 2 seconds.
      uint32_t tickGap = (inPhase >= kRedMs - kFastTickLastMs) ? 250 : 1000;
      if (lastTickMs == 0 || now - lastTickMs >= tickGap) {
        lastTickMs = now;
        minibot.buzzerPlay(2000, 40); // Kısa "tık" (arka planda) / short "tick" (in the background)
      }
      if (inPhase >= kRedMs) enterPhase(CAR_GREEN);
      break;
    }
  }
}
