/*
 * TR: GERÇEK PROJE - Kablosuz Servo Alıcısı
 *  - UZAKTAN (otomatik) modda bu MINIBOT, IOTBOT kumanda panelinin ESP-NOW ile
 *    yayınladığı komutları dinler: "servo" gelince Port B'deki servo motoru o açıya
 *    YUMUŞAKÇA döndürür, "led" gelince mavi LED'i açar/kapatır. Aldığı her yeni
 *    komutu seri porta yazar. IOTBOT'un potansiyometresini çevirin!
 *  - B1 butonuna basınca MANUEL moda geçer: panelden gelen komutlar yok sayılır,
 *    servoyu ve LED'i seri porttan siz yönetirsiniz. B1'e tekrar basınca UZAKTAN
 *    moda döner.
 *  - Önce IOTBOT'a IOTBOT_ESPNOW_Remote_Control_Panel_Example.ino dosyasını
 *    yükleyin; isterseniz bir ROLEBOT'a da ROLEBOT_ESPNOW_Remote_Relay_Receiver_
 *    Example.ino dosyasını yükleyip rölelerini aynı panelden yönetin.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim    / help          -> komut listesi
 *      oto       / auto          -> UZAKTAN mod (panel komutları) ("uzaktan" / "remote" da olur)
 *      manuel    / manual        -> manuel mod
 *      aci 90    / angle 90      -> servoyu 90°'ye götür (manuel moda geçer)
 *      led ac    / led on        -> mavi LED'i yak (manuel moda geçer)
 *      led kapat / led off       -> mavi LED'i söndür
 *      durum     / status        -> mod, açı ve LED durumu
 *      dil       / lang          -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Wireless Servo Receiver
 *  - In REMOTE (auto) mode this MINIBOT listens to the commands the IOTBOT control
 *    panel broadcasts over ESP-NOW: on "servo" it turns the servo on Port B SMOOTHLY
 *    to that angle, on "led" it switches the blue LED on/off. It prints every new
 *    command. Turn the IOTBOT's potentiometer!
 *  - Press B1 to switch to MANUAL mode: commands from the panel are ignored and you
 *    drive the servo and LED from the serial port. Press B1 again to go back to
 *    REMOTE mode.
 *  - First upload IOTBOT_ESPNOW_Remote_Control_Panel_Example.ino to an IOTBOT;
 *    optionally upload ROLEBOT_ESPNOW_Remote_Relay_Receiver_Example.ino to a ROLEBOT
 *    and control its relays from the same panel.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help      / yardim        -> command list
 *      auto      / oto           -> REMOTE mode (panel commands) ("remote" / "uzaktan" works too)
 *      manual    / manuel        -> manual mode
 *      angle 90  / aci 90        -> move the servo to 90° (switches to manual)
 *      led on    / led ac        -> blue LED on (switches to manual)
 *      led off   / led kapat     -> blue LED off
 *      status    / durum         -> mode, angle and LED state
 *      lang      / dil           -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Servo motoru Port B'ye (IO4) takın. / Plug the servo motor into
 * Port B (IO4).
 */

#define USE_ESPNOW
#define USE_SERVO
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

#define SERVO_PIN IO4 // Port B

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

const int kEspNowChannel = 1;        // IOTBOT ile AYNI kanal / SAME channel as the IOTBOT
const uint32_t kStepEveryMs = 8;     // Her 8 ms'de 1 derece (yumuşak hareket) / 1 degree every 8 ms (smooth motion)

bool manualMode = false;             // false = UZAKTAN, true = MANUEL / false = REMOTE, true = MANUAL
int targetAngle = 90;
int currentAngle = 90;
uint32_t lastStepMs = 0;
int ledState = -1;                   // -1 = henüz komut yok / no command yet

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
// Servo, LED ve mesajlar / Servo, LED and messages
// ---------------------------------------------------------------------------
void setLed(int on) {
  ledState = on;
  minibot.ledWrite(on);
  minibot.serialWrite(on ? L("LED AÇIK", "LED ON") : L("LED KAPALI", "LED OFF"));
}

void printHelp() {
  minibot.serialWrite(L("---- SERVO ALICISI - Komutlar ----", "---- SERVO RECEIVER - Commands ----"));
  minibot.serialWrite(L("  yardim          : bu liste", "  help            : this list"));
  minibot.serialWrite(L("  oto / uzaktan   : panel komutlarını izle", "  auto / remote   : follow the panel commands"));
  minibot.serialWrite(L("  manuel          : manuel mod", "  manual          : manual mode"));
  minibot.serialWrite(L("  aci 0-180       : servo açısı", "  angle 0-180     : servo angle"));
  minibot.serialWrite(L("  led ac / kapat  : mavi LED", "  led on / off    : blue LED"));
  minibot.serialWrite(L("  durum           : mod, açı, LED", "  status          : mode, angle, LED"));
  minibot.serialWrite(L("  dil             : English'e geç", "  lang            : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu       : UZAKTAN <-> MANUEL", "  B1 button       : REMOTE <-> MANUAL"));
}

void setMode(bool manual) {
  manualMode = manual;
  minibot.buzzerPlay(manual ? 1500 : 1000, 60); // Kısa bip / short beep
  minibot.serialWrite(manual ? L(">> MANUEL mod: panel yok sayılıyor, \"aci 90\" yazın.", ">> MANUAL mode: panel ignored, type \"angle 90\".")
                             : L(">> UZAKTAN mod: IOTBOT panelinin komutları izleniyor.", ">> REMOTE mode: following the IOTBOT panel commands."));
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  String arg = (space < 0) ? "" : cmd.substring(space + 1);
  bool hasValue = arg.length() > 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oto" || word == "otomatik" || word == "auto" || word == "uzaktan" || word == "remote") {
    setMode(false);
  } else if (word == "manuel" || word == "manual") {
    setMode(true);
  } else if ((word == "aci" || word == "angle") && hasValue) {
    if (!manualMode) setMode(true);
    targetAngle = constrain(arg.toInt(), 0, 180);
    minibot.serialWrite(String(L("Servo hedefi: ", "Servo target: ")) + targetAngle + "°");
  } else if (word == "led" && (arg == "ac" || arg == "on")) {
    if (!manualMode) setMode(true);
    setLed(1);
  } else if (word == "led" && (arg == "kapat" || arg == "off")) {
    if (!manualMode) setMode(true);
    setLed(0);
  } else if (word == "durum" || word == "status") {
    minibot.serialWrite(String(manualMode ? L("Mod: MANUEL", "Mode: MANUAL") : L("Mod: UZAKTAN", "Mode: REMOTE")) +
                        L("  |  açı: ", "  |  angle: ") + currentAngle + "°  |  LED: " +
                        (ledState == 1 ? L("açık", "on") : L("kapalı", "off")));
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
  minibot.espNowBegin(kEspNowChannel);
  minibot.moduleServoGoAngle(SERVO_PIN, currentAngle, 0); // 0 = hemen git (bekleme yok) / 0 = go instantly (no waiting)
  minibot.ledWrite(false);
  minibot.serialWrite(L("Servo alıcısı hazır - IOTBOT panelinden komut bekleniyor...",
                        "Servo receiver ready - waiting for commands from the IOTBOT panel..."));
  printHelp();
}

void loop() {
  // 1) B1 -> mod değiştir / B1 -> toggle mode
  if (b1Pressed()) setMode(!manualMode);

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Gelen mesaj. NOT: espNowReadName() mesajı "okundu" işaretler; bu yüzden
  //    sayıyı ÖNCE receivedData.value'dan alıyoruz, adı SONRA okuyoruz.
  //    Manuel modda mesaj okunur ama uygulanmaz.
  // 3) Incoming message. NOTE: espNowReadName() marks the message as read, so
  //    we take the number FIRST from receivedData.value and read the name AFTER.
  //    In manual mode the message is read but not applied.
  if (minibot.espNowAvailable()) {
    minibot.espNowReadText();              // Metin mesajıysa at / drop it if it is a text message
    float value = minibot.receivedData.value;
    String name = minibot.espNowReadName();

    if (!manualMode && name == "servo") {
      int angle = constrain((int)value, 0, 180);
      if (angle != targetAngle) {          // Panel aynı değeri tekrarlar - sadece değişince yaz / the panel repeats values - print only on change
        targetAngle = angle;
        minibot.serialWrite(String(L("Servo hedefi: ", "Servo target: ")) + angle + L(" derece", " degrees"));
      }
    } else if (!manualMode && name == "led") {
      int on = value > 0.5f ? 1 : 0;
      if (on != ledState) setLed(on);
    }
    // "role1"/"role2" ROLEBOT içindir, burada yok sayılır. / "role1"/"role2" are for the ROLEBOT and are ignored here.
  }

  // 4) Servoyu hedefe doğru her seferinde 1 derece yaklaştır - loop() hiç durmadan
  //    (moduleServoGoAngle'ın yavaş modu loop'u bekletirdi).
  // 4) Move the servo 1 degree toward the target each step - loop() never stops
  //    (moduleServoGoAngle's slow mode would block the loop).
  if (currentAngle != targetAngle && millis() - lastStepMs >= kStepEveryMs) {
    lastStepMs = millis();
    currentAngle += (targetAngle > currentAngle) ? 1 : -1;
    minibot.moduleServoGoAngle(SERVO_PIN, currentAngle, 0);
  }
}
