/*
 * TR: GERÇEK PROJE - Titreşim/Darbe Alarmı ("Kutuma Dokunma!")
 *  - Titreşim sensörü üzerine konduğu cisimde sarsıntı/darbe hissedince buzzer kısa
 *    bir alarm patlaması (4 sn) çalar ve mavi LED yanıp söner - bisiklet kilidi ya da
 *    kutu hırsızlık alarmı gibi düşünün.
 *  - B1 butonu sistemi KURAR / ÇÖZER. Çözmek çalan alarmı da susturur.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help        -> komut listesi
 *      kur    / arm         -> sistemi kur
 *      coz    / disarm      -> sistemi çöz (alarmı da susturur)
 *      sure 4 / time 4      -> her darbede alarm süresi (saniye, 1-30)
 *      durum  / status      -> sistem durumu ve darbe sayısı
 *      dil    / lang        -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Vibration/Shock Alarm ("Don't touch my box!")
 *  - When the vibration sensor feels a shake/impact on the object it sits on, the
 *    buzzer sounds a short alarm burst (4 s) and the blue LED flashes - think of a
 *    bike lock or an anti-theft box alarm.
 *  - The B1 button ARMS / DISARMS the system. Disarming also silences the alarm.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim      -> command list
 *      arm    / kur         -> arm the system
 *      disarm / coz         -> disarm the system (also silences the alarm)
 *      time 4 / sure 4      -> alarm length per impact (seconds, 1-30)
 *      status / durum       -> system state and impact count
 *      lang   / dil         -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Titreşim sensörünü IO12'ye bağlı sokete takın (başka bir soket
 * kullanırsanız VIBRATION_PIN değerini değiştirin). / Plug the vibration sensor into
 * the IO12 socket (change VIBRATION_PIN if you use another socket).
 * Desteklenen pinler / Supported pins: IO4 - IO12 - IO13 - IO14 (IO5 = buzzer)
 */

#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

#define VIBRATION_PIN IO12 // Titreşim sensörünün bağlı olduğu pin / Pin the vibration sensor is connected to

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

bool armed = false;
bool alarmActive = false;
uint32_t alarmStartMs = 0;        // Alarmın başladığı an / when the alarm started
uint32_t alarmDurationMs = 4000;  // Her darbede alarm kaç ms sürsün / how long each alarm burst lasts
uint32_t impactCount = 0;
uint32_t lastBeepMs = 0;
bool ledState = false;

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
    minibot.serialWrite(L("Sistem KURULDU. Kutuya dokunmayın!", "System ARMED. Do not touch!"));
    minibot.buzzerPlay(1500, 100);
  } else {
    minibot.serialWrite(L("Sistem ÇÖZÜLDÜ.", "System DISARMED."));
    minibot.buzzerPlay(800, 200);
  }
}

void printHelp() {
  minibot.serialWrite(L("---- TİTREŞİM ALARMI - Komutlar ----", "---- VIBRATION ALARM - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  kur / coz     : sistemi kur / çöz", "  arm / disarm  : arm / disarm the system"));
  minibot.serialWrite(L("  sure 1-30     : alarm süresi (saniye)", "  time 1-30     : alarm length (seconds)"));
  minibot.serialWrite(L("  durum         : sistem durumu", "  status        : system state"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu     : KUR <-> ÇÖZ", "  B1 button     : ARM <-> DISARM"));
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "kur" || word == "arm") {
    setArmed(true);
  } else if (word == "coz" || word == "disarm" || word == "sustur" || word == "silence") {
    setArmed(false);
  } else if ((word == "sure" || word == "time") && hasValue) {
    alarmDurationMs = (uint32_t)constrain(value, 1, 30) * 1000;
    minibot.serialWrite(String(L("Alarm süresi: ", "Alarm length: ")) + (alarmDurationMs / 1000) + L(" saniye", " seconds"));
  } else if (word == "durum" || word == "status") {
    String line = String(armed ? L("Sistem: KURULU", "System: ARMED") : L("Sistem: çözük", "System: disarmed")) +
                  L("  |  darbe sayısı: ", "  |  impacts: ") + impactCount;
    if (alarmActive) line += L("  |  ALARM ÇALIYOR", "  |  ALARM SOUNDING");
    minibot.serialWrite(line);
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
  minibot.serialWrite(L("Titreşim alarmı hazır. Kurmak için B1'e basın ya da \"kur\" yazın.", "Vibration alarm ready. Press B1 or type \"arm\" to arm."));
  printHelp();
}

void loop() {
  // 1) B1 -> kur / çöz / B1 -> arm / disarm
  if (b1Pressed()) setArmed(!armed);

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Kuruluyken (ve alarm çalmıyorken) darbe gelirse alarmı başlat.
  // 3) When armed (and not already sounding), an impact starts the alarm.
  if (armed && !alarmActive && minibot.moduleVibrationDigitalRead(VIBRATION_PIN)) {
    alarmActive = true;
    alarmStartMs = millis();
    impactCount++;
    minibot.serialWrite(String(L("ALARM: darbe/titreşim algılandı! (", "ALARM: impact/vibration detected! (")) + impactCount + ")");
  }

  // 4) Alarm: 150 ms'de bir bip + LED; süre dolunca biter (delay yok).
  // 4) Alarm: beep + LED every 150 ms; stops when the time is up (no delay).
  if (alarmActive) {
    if (millis() - alarmStartMs >= alarmDurationMs) {
      alarmActive = false;
      minibot.ledWrite(false);
      minibot.serialWrite(L("Alarm bitti, sistem hâlâ kurulu.", "Alarm over, system still armed."));
    } else if (millis() - lastBeepMs >= 150) {
      lastBeepMs = millis();
      ledState = !ledState;
      minibot.ledWrite(ledState);
      minibot.buzzerPlay(2400, 100);
    }
  }
}
