/*
 * TR: GERÇEK PROJE - Kapı/Pencere Alarmı
 *  - Manyetik sensörün iki parçasını (mıknatıs + sensör) bir kapının/pencerenin
 *    açılan ve sabit tarafına yerleştirin. Sistem "kurulu" haldeyken kapı açılırsa
 *    (manyetik bağlantı koparsa) buzzer alarm çalar ve mavi LED yanıp söner.
 *  - B1 butonu sistemi KURAR / ÇÖZER (etkisizleştirir) - tıpkı gerçek bir ev alarmı
 *    gibi. Çözmek alarmı da susturur.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help      -> komut listesi
 *      kur    / arm       -> sistemi kur
 *      coz    / disarm    -> sistemi çöz (alarmı da susturur)
 *      durum  / status    -> sistem ve kapı durumu
 *      dil    / lang      -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Door/Window Alarm
 *  - Place the magnetic sensor's two parts (magnet + sensor) on a door/window's
 *    moving and fixed sides. While the system is "armed", opening the door (breaking
 *    the magnetic contact) makes the buzzer sound an alarm and the blue LED flash.
 *  - The B1 button ARMS / DISARMS the system - just like a real home alarm.
 *    Disarming also silences the alarm.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim    -> command list
 *      arm    / kur       -> arm the system
 *      disarm / coz       -> disarm the system (also silences the alarm)
 *      status / durum     -> system and door state
 *      lang   / dil       -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Manyetik sensörü IO12'ye bağlı sokete takın (başka bir soket
 * kullanırsanız MAGNETIC_PIN değerini değiştirin). / Plug the magnetic sensor into
 * the IO12 socket (change MAGNETIC_PIN if you use another socket).
 * Desteklenen pinler / Supported pins: IO4 - IO12 - IO13 - IO14 (IO5 = buzzer)
 */

#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

#define MAGNETIC_PIN IO12 // Manyetik sensörün bağlı olduğu pin / Pin the magnetic sensor is connected to

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

bool armed = false;        // Sistem kurulu mu / system armed?
bool alarmActive = false;  // Alarm çalıyor mu / alarm sounding?
bool ledState = false;
uint32_t lastBeepMs = 0;

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

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "ÇÖZ" -> "coz"
// Lower-cases and simplifies Turkish letters: "ÇÖZ" -> "coz"
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
// Alarm ve mesajlar / Alarm and messages
// ---------------------------------------------------------------------------
void setArmed(bool on) {
  armed = on;
  alarmActive = false;
  minibot.ledWrite(false);
  if (armed) {
    minibot.serialWrite(L("Sistem KURULDU.", "System ARMED."));
    if (!minibot.moduleMagneticRead(MAGNETIC_PIN)) {
      minibot.serialWrite(L("Dikkat: kapı şu an AÇIK - kapanmazsa alarm çalacak!", "Warning: the door is OPEN now - the alarm will sound!"));
    }
    minibot.buzzerPlay(1500, 100);
  } else {
    minibot.serialWrite(L("Sistem ÇÖZÜLDÜ (alarm kapalı).", "System DISARMED (alarm off)."));
    minibot.buzzerPlay(800, 200);
  }
}

void printStatus() {
  bool doorClosed = minibot.moduleMagneticRead(MAGNETIC_PIN); // true = mıknatıs var = kapı kapalı / magnet present = door closed
  String line = String(armed ? L("Sistem: KURULU", "System: ARMED") : L("Sistem: çözük", "System: disarmed")) +
                "  |  " + (doorClosed ? L("Kapı: kapalı", "Door: closed") : L("Kapı: AÇIK", "Door: OPEN"));
  if (alarmActive) line += L("  |  ALARM ÇALIYOR", "  |  ALARM SOUNDING");
  minibot.serialWrite(line);
}

void printHelp() {
  minibot.serialWrite(L("---- KAPI ALARMI - Komutlar ----", "---- DOOR ALARM - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  kur           : sistemi kur", "  arm           : arm the system"));
  minibot.serialWrite(L("  coz           : sistemi çöz / alarmı sustur", "  disarm        : disarm / silence the alarm"));
  minibot.serialWrite(L("  durum         : sistem ve kapı durumu", "  status        : system and door state"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu     : KUR <-> ÇÖZ", "  B1 button     : ARM <-> DISARM"));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "kur" || cmd == "arm") {
    setArmed(true);
  } else if (cmd == "coz" || cmd == "disarm" || cmd == "sustur" || cmd == "silence") {
    setArmed(false);
  } else if (cmd == "durum" || cmd == "status") {
    printStatus();
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
  minibot.serialWrite(L("Kapı alarmı hazır. Kurmak için B1'e basın ya da \"kur\" yazın.", "Door alarm ready. Press B1 or type \"arm\" to arm."));
  printHelp();
}

void loop() {
  // 1) B1 -> kur / çöz / B1 -> arm / disarm
  if (b1Pressed()) setArmed(!armed);

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Kuruluyken kapı açılırsa alarmı başlat / when armed and the door opens, start the alarm
  if (armed && !alarmActive) {
    bool doorClosed = minibot.moduleMagneticRead(MAGNETIC_PIN);
    if (!doorClosed) {
      alarmActive = true;
      minibot.serialWrite(L("ALARM: kapı/pencere açıldı! (B1 ya da \"coz\" = sustur)", "ALARM: door/window opened! (B1 or \"disarm\" = silence)"));
    }
  }

  // 4) Alarm: 250 ms'de bir bip + LED yanıp söner (delay yok)
  // 4) Alarm: beep + LED blink every 250 ms (no delay)
  if (alarmActive && millis() - lastBeepMs >= 250) {
    lastBeepMs = millis();
    ledState = !ledState;
    minibot.ledWrite(ledState);
    minibot.buzzerPlay(2200, 120);
  }
}
