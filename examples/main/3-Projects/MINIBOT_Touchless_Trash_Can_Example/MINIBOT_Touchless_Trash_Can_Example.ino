/*
 * TR: GERÇEK PROJE - Temassız Çöp Kutusu
 *  - OTOMATİK modda elinizi (ya da çöpü) kapağın üzerine 15 cm'den yakına getirince
 *    ultrasonik sensör bunu görür ve servo motor kapağı KENDİLİĞİNDEN açar. Eliniz
 *    gittikten 3 saniye sonra kapak yavaşça kapanır. Kapak açıkken mavi LED yanar.
 *    Hiç dokunmadan - mikrop taşımadan - çöp atmanın yolu!
 *  - Servo yumuşak hareket eder ama kod hiç beklemez (millis); sensör her an okunur.
 *  - B1 butonuna basınca MANUEL moda geçer: sensör kapağı açmaz, kapağı seri porttan
 *    siz açıp kapatırsınız (kapak açısını ayarlamak için de kullanışlı).
 *    B1'e tekrar basınca otomatik moda döner.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help        -> komut listesi
 *      oto    / auto        -> otomatik mod (sensörle)
 *      manuel / manual      -> manuel mod
 *      ac     / open        -> kapağı aç (manuel moda geçer)
 *      kapat  / close       -> kapağı kapat (manuel moda geçer)
 *      aci 60 / angle 60    -> servoyu 60°'ye götür (manuel moda geçer)
 *      oku    / read        -> mesafeyi yaz
 *      dil    / lang        -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Touchless Trash Can
 *  - In AUTO mode, bring your hand (or the trash) closer than 15 cm above the lid:
 *    the ultrasonic sensor sees it and the servo motor opens the lid BY ITSELF. 3
 *    seconds after your hand is gone the lid closes slowly. The blue LED is on while
 *    the lid is open. A way to throw trash away without touching anything - no germs!
 *  - The servo moves smoothly but the code never blocks (millis); the sensor keeps reading.
 *  - Press B1 to switch to MANUAL mode: the sensor does not open the lid, you open and
 *    close it from the serial port (handy for adjusting the lid angle too). Press B1
 *    again to go back to auto mode.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help     / yardim    -> command list
 *      auto     / oto       -> auto mode (with the sensor)
 *      manual   / manuel    -> manual mode
 *      open     / ac        -> open the lid (switches to manual)
 *      close    / kapat     -> close the lid (switches to manual)
 *      angle 60 / aci 60    -> move the servo to 60° (switches to manual)
 *      read     / oku       -> print the distance
 *      lang     / dil       -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ultrasonik sensör sabit pinler kullanır (TRIG = IO12, ECHO = IO13)
 * - soket seçmenize gerek yok. Servo motoru Port B'ye (IO4) takın (ultrasonik
 * pinleriyle çakışmaz). Servo kolunu kapağa bağlayın. / The ultrasonic sensor uses
 * fixed pins (TRIG = IO12, ECHO = IO13) - no socket to choose. Plug the servo motor
 * into Port B (IO4) (it does not clash with the ultrasonic pins). Attach the servo
 * arm to the lid.
 */

#define USE_SERVO
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

#define SERVO_PIN IO4 // Port B

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

const int kOpenDistanceCm = 15;        // Bu mesafeden yakın = el var / closer than this = hand present
const int kClosedAngle = 0;            // Kapak kapalı açısı / lid closed angle
const int kOpenAngle = 100;            // Kapak açık açısı (kutunuza göre ayarlayın) / lid open angle (adjust to your bin)
const uint32_t kStayOpenMs = 3000;     // El gittikten sonra açık kalma süresi / stay open after the hand leaves
const uint32_t kMeasureEveryMs = 100;  // Ölçüm aralığı / measurement interval
const uint32_t kOpenStepMs = 5;        // Açılış hızı (derece başına ms) / opening speed (ms per degree)
const uint32_t kCloseStepMs = 15;      // Kapanış daha yavaş (parmak sıkışmasın) / closing is slower (no pinched fingers)

bool manualMode = false;               // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
bool lidOpen = false;
int nearCount = 0;                     // Arka arkaya "yakın" ölçüm sayısı / consecutive "near" readings
int lastDistance = 0;
uint32_t lastSeenMs = 0, lastMeasureMs = 0, lastStepMs = 0;
int currentAngle = kClosedAngle;
int targetAngle = kClosedAngle;

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
// Kapak ve mesajlar / Lid and messages
// ---------------------------------------------------------------------------
void moveLid(int angle) {
  targetAngle = constrain(angle, 0, 180);
  lidOpen = targetAngle != kClosedAngle;
  minibot.ledWrite(lidOpen);
}

void printHelp() {
  minibot.serialWrite(L("---- TEMASSIZ ÇÖP KUTUSU - Komutlar ----", "---- TOUCHLESS TRASH CAN - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  oto           : otomatik mod (sensörle)", "  auto          : auto mode (with the sensor)"));
  minibot.serialWrite(L("  manuel        : manuel mod", "  manual        : manual mode"));
  minibot.serialWrite(L("  ac / kapat    : kapağı aç / kapat", "  open / close  : open / close the lid"));
  minibot.serialWrite(L("  aci 0-180     : servo açısı", "  angle 0-180   : servo angle"));
  minibot.serialWrite(L("  oku           : mesafeyi yaz", "  read          : print the distance"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu     : OTOMATİK <-> MANUEL", "  B1 button     : AUTO <-> MANUAL"));
}

void setMode(bool manual) {
  manualMode = manual;
  nearCount = 0;
  minibot.buzzerPlay(manual ? 1500 : 1000, 60); // Kısa bip / short beep
  if (manual) {
    minibot.serialWrite(L(">> MANUEL mod: \"ac\" / \"kapat\" / \"aci 60\" yazın.", ">> MANUAL mode: type \"open\" / \"close\" / \"angle 60\"."));
  } else {
    minibot.serialWrite(L(">> OTOMATİK mod: elinizi kapağa yaklaştırın.", ">> AUTO mode: bring your hand near the lid."));
    moveLid(kClosedAngle); // Otomatik mod kapalı kapakla başlar / auto mode starts with the lid closed
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
  } else if (word == "ac" || word == "open") {
    if (!manualMode) setMode(true);
    moveLid(kOpenAngle);
    minibot.serialWrite(L("Kapak AÇILIYOR.", "Lid OPENING."));
  } else if (word == "kapat" || word == "close") {
    if (!manualMode) setMode(true);
    moveLid(kClosedAngle);
    minibot.serialWrite(L("Kapak kapanıyor.", "Lid closing."));
  } else if ((word == "aci" || word == "angle") && hasValue) {
    if (!manualMode) setMode(true);
    moveLid(value);
    minibot.serialWrite(String(L("Hedef açı: ", "Target angle: ")) + targetAngle + "°");
  } else if (word == "oku" || word == "read") {
    int d = minibot.moduleUltrasonicDistanceRead();
    if (d == 0) minibot.serialWrite(L("Mesafe: --- (yansıma yok)", "Distance: --- (no echo)"));
    else minibot.serialWrite(String(L("Mesafe: ", "Distance: ")) + d + " cm");
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
  minibot.moduleServoGoAngle(SERVO_PIN, kClosedAngle, 0); // Kapalı başlat / start closed
  minibot.ledWrite(false);
  minibot.serialWrite(L("Temassız çöp kutusu hazır. Elinizi kapağa yaklaştırın.", "Touchless trash can ready. Bring your hand near the lid."));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) B1 -> mod değiştir / B1 -> toggle mode
  if (b1Pressed()) setMode(!manualMode);

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  if (!manualMode) {
    // 3) Mesafeyi ölç. 0 = yansıma yok (uzak ya da geçersiz).
    // 3) Measure the distance. 0 = no echo (far away or invalid).
    if (now - lastMeasureMs >= kMeasureEveryMs) {
      lastMeasureMs = now;
      lastDistance = minibot.moduleUltrasonicDistanceRead();
      bool near = lastDistance > 0 && lastDistance < kOpenDistanceCm;
      // Tek bir hatalı ölçüm kapağı açmasın: arka arkaya 2 "yakın" ölçüm iste.
      // One bad reading must not open the lid: require 2 "near" readings in a row.
      nearCount = near ? nearCount + 1 : 0;

      if (nearCount >= 2) {
        lastSeenMs = now;
        if (!lidOpen) {
          moveLid(kOpenAngle);
          minibot.buzzerPlay(1800, 60);
          minibot.serialWrite(String(L("Kapak AÇILIYOR (", "Lid OPENING (")) + lastDistance + " cm)");
        }
      }
    }

    // 4) El 3 saniyedir yoksa kapat. / Close if there has been no hand for 3 seconds.
    if (lidOpen && now - lastSeenMs >= kStayOpenMs) {
      moveLid(kClosedAngle);
      minibot.serialWrite(L("Kapak kapanıyor.", "Lid closing."));
    }
  }

  // 5) Servoyu 1'er derece hedefe taşı (beklemeden). / Step the servo 1 degree toward the target (non-blocking).
  uint32_t stepMs = (targetAngle > currentAngle) ? kOpenStepMs : kCloseStepMs;
  if (currentAngle != targetAngle && now - lastStepMs >= stepMs) {
    lastStepMs = now;
    currentAngle += (targetAngle > currentAngle) ? 1 : -1;
    minibot.moduleServoGoAngle(SERVO_PIN, currentAngle, 0);
  }
}
