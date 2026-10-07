/*
 * TR: SERVO MOTOR MODÜLÜ - Otomatik demo + Manuel kontrol
 *  - Açılışta OTOMATİK mod çalışır: servo 0° ile 180° arasında yavaşça gidip gelir.
 *  - B1 butonuna basınca MANUEL moda geçer: servo durur, açıyı seri porttan
 *    "aci 90" gibi komutlarla siz verirsiniz. B1'e tekrar basınca otomatik moda döner.
 *    (MINIBOT'ta potansiyometre yoktur; manuel kontrol seri port üzerindendir.)
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim  / help       -> komut listesi
 *      oto     / auto       -> otomatik mod
 *      manuel  / manual     -> manuel mod
 *      aci 90  / angle 90   -> servoyu 90°'ye götür (manuel moda geçer)
 *      dil     / lang       -> dili değiştir (Türkçe <-> English)
 *
 * EN: SERVO MOTOR MODULE - Automatic demo + Manual control
 *  - At startup AUTO mode runs: the servo sweeps slowly between 0° and 180°.
 *  - Press B1 to switch to MANUAL mode: the servo stops and you give the angle
 *    from the serial port with commands like "angle 90". Press B1 again to go
 *    back to auto mode. (MINIBOT has no potentiometer; manual control is via serial.)
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help    / yardim     -> command list
 *      auto    / oto        -> auto mode
 *      manual  / manuel     -> manual mode
 *      angle 90 / aci 90    -> move the servo to 90° (switches to manual)
 *      lang    / dil        -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Servoyu IO12'ye bağlı sokete takın.
 * Desteklenen pinler / Supported pins: IO4 - IO5 - IO12 - IO13 - IO14
 * (IO5 kartın buzzer'ını da sürer / IO5 also drives the board's buzzer.)
 *
 * NOT / NOTE: "#define USE_SERVO" satırı #include'dan ÖNCE yazılmalıdır.
 *             The "#define USE_SERVO" line must come BEFORE the #include.
 */

#define USE_SERVO
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

#define SERVO_PIN IO12 // Servonun bağlı olduğu pin / Pin the servo is connected to

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

bool manualMode = false;   // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
int angle = 90;            // Servonun şu anki açısı / current servo angle
int targetAngle = 90;      // Gitmek istediği açı / angle it is heading to
int sweepDir = 1;          // Otomatik moddaki yön (+1 / -1) / sweep direction in auto mode
uint32_t lastStepMs = 0;   // Son adım zamanı / time of the last step
uint32_t pauseUntilMs = 0; // Uçlarda kısa mola / short rest at the ends

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

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "AÇI" -> "aci"
// Lower-cases and simplifies Turkish letters: "AÇI" -> "aci"
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
void printHelp() {
  minibot.serialWrite(L("---- SERVO MOTOR - Komutlar ----", "---- SERVO MOTOR - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  oto           : otomatik mod", "  auto          : auto mode"));
  minibot.serialWrite(L("  manuel        : manuel mod", "  manual        : manual mode"));
  minibot.serialWrite(L("  aci 0-180     : servoyu o açıya götür", "  angle 0-180   : move the servo to that angle"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu     : OTOMATİK <-> MANUEL", "  B1 button     : AUTO <-> MANUAL"));
}

void setMode(bool manual) {
  manualMode = manual;
  minibot.buzzerPlay(manual ? 1500 : 1000, 60); // Kısa bip (arka planda çalar) / short beep (plays in the background)
  if (manual) {
    targetAngle = angle; // Manuelde olduğu yerde dur / stop where it is in manual
    minibot.serialWrite(L(">> MANUEL mod: açıyı \"aci 90\" gibi yazın.", ">> MANUAL mode: type the angle like \"angle 90\"."));
  } else {
    minibot.serialWrite(L(">> OTOMATİK mod: servo kendi kendine gidip geliyor.", ">> AUTO mode: the servo sweeps by itself."));
  }
  minibot.ledWrite(manual); // Mavi LED yanıyorsa MANUEL / blue LED on = MANUAL
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
  } else if ((word == "aci" || word == "angle") && hasValue) {
    if (!manualMode) setMode(true);
    targetAngle = constrain(value, 0, 180);
    minibot.serialWrite(String(L("Hedef açı: ", "Target angle: ")) + targetAngle + "°");
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
  minibot.moduleServoGoAngle(SERVO_PIN, angle, 0); // Ortadan başla (0 = hemen git) / start at the middle (0 = go instantly)
  minibot.ledWrite(false);
  minibot.serialWrite(L("Servo motor testi başladı.", "Servo motor test started."));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) B1 -> mod değiştir (sadece basıldığı an) / B1 -> toggle mode (on press only)
  if (b1Pressed()) setMode(!manualMode);

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Otomatik modda uca varınca yön değiştir ve yarım saniye bekle.
  // 3) In auto mode: at an end, reverse and rest for half a second.
  if (!manualMode && now >= pauseUntilMs && angle == targetAngle) {
    if (angle >= 180) sweepDir = -1;
    if (angle <= 0) sweepDir = 1;
    targetAngle = (sweepDir > 0) ? 180 : 0;
    minibot.serialWrite(String(L("Otomatik: hedef ", "Auto: heading to ")) + targetAngle + "°");
  }

  // 4) Servoyu her 15 ms'de 1° hedefe yaklaştır (loop hiç bloklanmaz, B1 anında çalışır).
  //    moduleServoGoAngle(pin, açı, 0) beklemeden gider; yavaş modu (3. değer > 0) loop'u bekletirdi.
  // 4) Move the servo 1° toward the target every 15 ms (loop never blocks, B1 reacts instantly).
  //    moduleServoGoAngle(pin, angle, 0) moves without waiting; its slow mode (3rd value > 0) would block.
  if (angle != targetAngle && now - lastStepMs >= 15) {
    lastStepMs = now;
    angle += (targetAngle > angle) ? 1 : -1;
    minibot.moduleServoGoAngle(SERVO_PIN, angle, 0);
    if (!manualMode && angle == targetAngle) pauseUntilMs = now + 500;
    if (manualMode && angle == targetAngle) {
      minibot.serialWrite(String(L("Servo ", "Servo at ")) + angle + L("° konumunda.", "°."));
    }
  }
}
