/*
 * TR: GERÇEK PROJE - Güvenlik Alarmı
 *  - Sistem açılışta KURULU başlar. PIR hareket sensörü YENİ bir hareket algılayınca
 *    buzzer sürekli alarm çalar ve mavi LED yanıp söner.
 *  - B1 butonu: alarm çalarken alarmı SUSTURUR (gerçek alarmlardaki "iptal" tuşu
 *    gibi). Alarm yokken sistemi KURAR / ÇÖZER.
 *  - Susturulan alarm aynı hareketle hemen tekrar çalmaz: sensör "hareket yok"a
 *    dönüp YENİ bir hareket görmesi gerekir.
 *  - İpucu: PIR açıldıktan sonra ~30-60 sn ortama alışır; o sırada yanlış alarm olabilir.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim  / help      -> komut listesi
 *      sustur  / silence   -> alarmı sustur
 *      kur     / arm       -> sistemi kur
 *      coz     / disarm    -> sistemi çöz
 *      durum   / status    -> sistem ve sensör durumu
 *      dil     / lang      -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Security Alarm
 *  - The system starts ARMED. When the PIR motion sensor detects a NEW movement the
 *    buzzer sounds a continuous alarm and the blue LED flashes.
 *  - The B1 button: while the alarm sounds it SILENCES it (like the "cancel" key on
 *    real alarms). With no alarm it ARMS / DISARMS the system.
 *  - A silenced alarm does not restart from the same movement: the sensor must go
 *    back to "no motion" and then see a NEW movement.
 *  - Tip: after power-up the PIR needs ~30-60 s to settle; false alarms may happen then.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help    / yardim    -> command list
 *      silence / sustur    -> silence the alarm
 *      arm     / kur       -> arm the system
 *      disarm  / coz       -> disarm the system
 *      status  / durum     -> system and sensor state
 *      lang    / dil       -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: PIR sensörünü IO12'ye bağlı sokete takın (başka bir soket
 * kullanırsanız PIR_PIN değerini değiştirin). / Plug the PIR sensor into the IO12
 * socket (change PIR_PIN if you use another socket).
 * Desteklenen pinler / Supported pins: IO4 - IO12 - IO13 - IO14 (IO5 = buzzer)
 */

#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

#define PIR_PIN IO12 // PIR sensörünün bağlı olduğu pin / Pin the PIR sensor is connected to

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

bool armed = true;          // Sistem kurulu mu / system armed?
bool alarmActive = false;   // Alarm çalıyor mu / alarm sounding?
bool lastMotion = true;     // Son sensör durumu (true: açılışta hareket varsa hemen alarm olmasın)
                            // last sensor state (true: no instant alarm if there is motion at power-up)
bool ledState = false;
uint32_t lastBeepMs = 0;
uint32_t lastReadMs = 0;

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
void silenceAlarm() {
  if (!alarmActive) {
    minibot.serialWrite(L("Çalan bir alarm yok.", "No alarm is sounding."));
    return;
  }
  alarmActive = false;
  minibot.ledWrite(false);
  minibot.serialWrite(L("Alarm susturuldu. Sistem hâlâ kurulu (yeni harekette tekrar çalar).",
                        "Alarm silenced. System still armed (sounds again on new motion)."));
}

void setArmed(bool on) {
  armed = on;
  alarmActive = false;
  minibot.ledWrite(false);
  lastMotion = true; // Kurulunca o anki hareket alarm sayılmasın / motion present at arming is not an alarm
  minibot.buzzerPlay(on ? 1500 : 800, on ? 100 : 200);
  minibot.serialWrite(on ? L("Sistem KURULDU.", "System ARMED.") : L("Sistem ÇÖZÜLDÜ (alarm kapalı).", "System DISARMED (alarm off)."));
}

void printStatus() {
  String line = String(armed ? L("Sistem: KURULU", "System: ARMED") : L("Sistem: çözük", "System: disarmed")) + "  |  " +
                (minibot.moduleMotionRead(PIR_PIN) ? L("Sensör: hareket VAR", "Sensor: MOTION") : L("Sensör: hareket yok", "Sensor: no motion"));
  if (alarmActive) line += L("  |  ALARM ÇALIYOR", "  |  ALARM SOUNDING");
  minibot.serialWrite(line);
}

void printHelp() {
  minibot.serialWrite(L("---- GÜVENLİK ALARMI - Komutlar ----", "---- SECURITY ALARM - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  sustur        : alarmı sustur", "  silence       : silence the alarm"));
  minibot.serialWrite(L("  kur / coz     : sistemi kur / çöz", "  arm / disarm  : arm / disarm the system"));
  minibot.serialWrite(L("  durum         : sistem ve sensör durumu", "  status        : system and sensor state"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu     : alarmı sustur / kur <-> çöz", "  B1 button     : silence / arm <-> disarm"));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "sustur" || cmd == "silence") {
    silenceAlarm();
  } else if (cmd == "kur" || cmd == "arm") {
    setArmed(true);
  } else if (cmd == "coz" || cmd == "disarm") {
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
  minibot.serialWrite(L("Güvenlik sistemi KURULU.", "Security system ARMED."));
  printHelp();
}

void loop() {
  // 1) B1: alarm çalıyorsa sustur, yoksa kur/çöz / B1: silence if sounding, else arm/disarm
  if (b1Pressed()) {
    if (alarmActive) silenceAlarm();
    else setArmed(!armed);
  }

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Sensörü 100 ms'de bir oku. Alarm sadece YENİ harekette (yok -> var) başlar.
  //    Böylece susturulan alarm, sensör hâlâ "var" derken hemen tekrar çalmaz.
  // 3) Read the sensor every 100 ms. The alarm starts only on NEW motion (none -> motion),
  //    so a silenced alarm does not restart while the sensor still says "motion".
  if (millis() - lastReadMs >= 100) {
    lastReadMs = millis();
    bool motion = minibot.moduleMotionRead(PIR_PIN);
    if (armed && !alarmActive && motion && !lastMotion) {
      alarmActive = true;
      minibot.serialWrite(L("ALARM: hareket algılandı! (B1 ya da \"sustur\")", "ALARM: motion detected! (B1 or \"silence\")"));
    }
    lastMotion = motion;
  }

  // 4) Alarm: 300 ms'de bir bip + LED yanıp söner (delay yok)
  // 4) Alarm: beep + LED blink every 300 ms (no delay)
  if (alarmActive && millis() - lastBeepMs >= 300) {
    lastBeepMs = millis();
    ledState = !ledState;
    minibot.ledWrite(ledState);
    minibot.buzzerPlay(2000, 150);
  }
}
