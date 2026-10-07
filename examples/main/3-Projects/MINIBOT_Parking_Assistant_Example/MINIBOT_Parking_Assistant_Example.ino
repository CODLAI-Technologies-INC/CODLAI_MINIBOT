/*
 * TR: GERÇEK PROJE - Park Sensörü
 *  - Ultrasonik mesafe sensörü bir cisme (örneğin bir duvara) yaklaştıkça buzzer
 *    GİDEREK HIZLANAN bir "bip" sesi çıkarır - tıpkı arabalardaki park sensörü gibi.
 *    Çok yaklaşınca (10 cm altı) ses sürekli hale gelir ve mavi LED yanar.
 *    100 cm'den uzaktaki cisimler için ses yoktur.
 *  - Kod hiç beklemeden (millis ile) çalışır: ölçüm, bip ve komutlar aynı anda yürür.
 *  - B1 butonu sesi KAPATIR / AÇAR (LED ve seri port çalışmaya devam eder).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim   / help      -> komut listesi
 *      sessiz   / mute      -> sesi kapat/aç
 *      oku      / read      -> şu anki mesafeyi yaz
 *      yakin 10 / near 10   -> sürekli ses mesafesi (cm, 3-50)
 *      uzak 100 / far 100   -> bip başlama mesafesi (cm, 20-300)
 *      dil      / lang      -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Parking Sensor
 *  - As the ultrasonic distance sensor gets closer to an object (e.g. a wall), the
 *    buzzer beeps FASTER AND FASTER - just like a real car's parking sensor. When very
 *    close (under 10 cm) the sound becomes constant and the blue LED turns on.
 *    Objects farther than 100 cm make no sound.
 *  - The code never waits (uses millis): measuring, beeping and commands run together.
 *  - The B1 button MUTES / UNMUTES the sound (the LED and serial keep working).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help     / yardim    -> command list
 *      mute     / sessiz    -> sound off/on
 *      read     / oku       -> print the current distance
 *      near 10  / yakin 10  -> constant-tone distance (cm, 3-50)
 *      far 100  / uzak 100  -> distance where beeping starts (cm, 20-300)
 *      lang     / dil       -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ultrasonik sensör SABİT pinler kullanır: TRIG = IO12,
 * ECHO = IO13 - pin seçmenize gerek yok. / The ultrasonic sensor uses FIXED pins:
 * TRIG = IO12, ECHO = IO13 - no pin to choose.
 */

#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

int stopDistanceCm = 10;      // Bu mesafe ve altında sürekli ses / at or below this: constant tone
int maxUsefulDistanceCm = 100; // Bu mesafenin üstünde sessiz / above this: silent
bool muted = false;

int distance = 0;             // Son ölçüm (cm, 0 = yansıma yok) / last reading (cm, 0 = no echo)
int lastZone = -1;            // 0 = uzak, 1 = yaklaşıyor, 2 = DUR / 0 = far, 1 = approaching, 2 = STOP
uint32_t lastMeasureMs = 0, lastBeepMs = 0, lastPrintMs = 0;

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

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "YAKIN" -> "yakin"
// Lower-cases and simplifies Turkish letters: "YAKIN" -> "yakin"
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
int zoneOf(int d) {
  if (d <= 0 || d > maxUsefulDistanceCm) return 0; // uzak ya da yansıma yok / far or no echo
  if (d <= stopDistanceCm) return 2;               // DUR / STOP
  return 1;                                        // yaklaşıyor / approaching
}

void printDistance() {
  int zone = zoneOf(distance);
  if (distance <= 0) {
    minibot.serialWrite(L("Mesafe: --- (yansıma yok, uzak)", "Distance: --- (no echo, far)"));
  } else if (zone == 2) {
    minibot.serialWrite(String(L("DUR! ", "STOP! ")) + distance + " cm");
  } else {
    minibot.serialWrite(String(L("Mesafe: ", "Distance: ")) + distance + " cm" + (zone == 0 ? L("  (uzak, sessiz)", "  (far, silent)") : ""));
  }
}

void printHelp() {
  minibot.serialWrite(L("---- PARK SENSÖRÜ - Komutlar ----", "---- PARKING SENSOR - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  sessiz        : sesi kapat/aç", "  mute          : sound off/on"));
  minibot.serialWrite(L("  oku           : şu anki mesafe", "  read          : current distance"));
  minibot.serialWrite(L("  yakin 3-50    : sürekli ses mesafesi (cm)", "  near 3-50     : constant-tone distance (cm)"));
  minibot.serialWrite(L("  uzak 20-300   : bip başlama mesafesi (cm)", "  far 20-300    : beeping starts at (cm)"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu     : ses kapat/aç", "  B1 button     : sound off/on"));
}

void toggleMute() {
  muted = !muted;
  minibot.serialWrite(muted ? L("Ses KAPALI.", "Sound OFF.") : L("Ses AÇIK.", "Sound ON."));
  if (!muted) minibot.buzzerPlay(1000, 60);
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "sessiz" || word == "mute" || word == "ses" || word == "sound") {
    toggleMute();
  } else if (word == "oku" || word == "read") {
    printDistance();
  } else if ((word == "yakin" || word == "near") && hasValue) {
    stopDistanceCm = constrain(value, 3, 50);
    if (maxUsefulDistanceCm <= stopDistanceCm + 10) maxUsefulDistanceCm = stopDistanceCm + 10;
    minibot.serialWrite(String(L("Sürekli ses mesafesi: ", "Constant-tone distance: ")) + stopDistanceCm + " cm");
  } else if ((word == "uzak" || word == "far") && hasValue) {
    maxUsefulDistanceCm = constrain(value, max(20, stopDistanceCm + 10), 300);
    minibot.serialWrite(String(L("Bip başlama mesafesi: ", "Beeping starts at: ")) + maxUsefulDistanceCm + " cm");
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
  minibot.serialWrite(L("Park sensörü hazır.", "Parking sensor ready."));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) B1 -> ses kapat/aç / B1 -> sound off/on
  if (b1Pressed()) toggleMute();

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) 60 ms'de bir ölç / measure every 60 ms
  if (now - lastMeasureMs >= 60) {
    lastMeasureMs = now;
    distance = minibot.moduleUltrasonicDistanceRead();
    int zone = zoneOf(distance);
    minibot.ledWrite(zone == 2); // Çok yakın: LED yanar / very close: LED on
    // Bölge değişince ya da yakındayken yarım saniyede bir yaz.
    // Print when the zone changes, or every half second while close.
    if (zone != lastZone || (zone != 0 && now - lastPrintMs >= 500)) {
      lastZone = zone;
      lastPrintMs = now;
      printDistance();
    }
  }

  // 4) Bip: yaklaştıkça bipler arası süre kısalır (delay yok).
  // 4) Beep: the closer, the shorter the gap between beeps (no delay).
  int zone = zoneOf(distance);
  if (!muted && zone == 2 && now - lastBeepMs >= 140) {
    lastBeepMs = now;
    minibot.buzzerPlay(1800, 160);               // Üst üste binen bipler = sürekli ses / overlapping beeps = constant tone
  } else if (!muted && zone == 1) {
    uint32_t gap = map(distance, stopDistanceCm, maxUsefulDistanceCm, 60, 600);
    if (now - lastBeepMs >= gap + 60) {
      lastBeepMs = now;
      minibot.buzzerPlay(1800, 60);
    }
  }
}
