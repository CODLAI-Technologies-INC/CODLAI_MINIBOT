/*
 * TR: KABLOSUZ AKILLI EV FİKRİ - OTOMATİK VANTİLATÖR
 *  - Bu MINIBOT'un kendi sıcaklık sensörü YOK - bunun yerine, aynı odadaki bir
 *    IOTBOT'un yayınladığı (broadcast) DHT sıcaklık verisini ESP-NOW ile dinler ve
 *    OTOMATİK modda sıcaklık eşiği (28 °C) geçince röle modülüne bağlı GERÇEK bir
 *    vantilatörü/fanı açar, serinleyince kapatır. Mavi LED fanın durumunu gösterir.
 *  - Önce IOTBOT_ESPNOW_Temperature_Broadcast_Example.ino dosyasını bir IOTBOT'a,
 *    sonra bu kodu bir MINIBOT'a yükleyin. IOTBOT'un DHT sensörünü elinizle ısıtınca
 *    MINIBOT'un rölesi (ve bağlıysa vantilatörünüz) açılacak!
 *  - B1 butonuna basınca MANUEL moda geçer: sıcaklık fanı yönetmez, fanı seri porttan
 *    siz açıp kapatırsınız. B1'e tekrar basınca otomatiğe döner.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim   / help           -> komut listesi
 *      oto      / auto           -> otomatik mod (sıcaklığa göre)
 *      manuel   / manual         -> manuel mod
 *      ac       / on             -> fanı aç (manuel moda geçer)
 *      kapat    / off            -> fanı kapat (manuel moda geçer)
 *      esik 30  / threshold 30   -> sıcaklık eşiği (°C)
 *      durum    / status         -> son sıcaklık ve fan durumu
 *      dil      / lang           -> dili değiştir (Türkçe <-> English)
 *
 * EN: A WIRELESS SMART HOME IDEA - AUTOMATIC FAN
 *  - This MINIBOT has NO temperature sensor of its own - instead, it listens over
 *    ESP-NOW to the DHT temperature data broadcast by an IOTBOT in the same room and,
 *    in AUTO mode, turns on a REAL fan (wired to its relay module) when the threshold
 *    (28 °C) is passed and off when it cools down. The blue LED shows the fan state.
 *  - First upload IOTBOT_ESPNOW_Temperature_Broadcast_Example.ino to an IOTBOT, then
 *    this code to a MINIBOT. Warm the IOTBOT's DHT sensor with your hand and watch the
 *    MINIBOT's relay (and your fan, if wired) turn on!
 *  - Press B1 to switch to MANUAL mode: the temperature does not control the fan, you
 *    switch it on/off from the serial port. Press B1 again to go back to auto.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help         / yardim     -> command list
 *      auto         / oto        -> auto mode (by temperature)
 *      manual       / manuel     -> manual mode
 *      on           / ac         -> fan on (switches to manual)
 *      off          / kapat      -> fan off (switches to manual)
 *      threshold 30 / esik 30    -> temperature threshold (°C)
 *      status       / durum      -> last temperature and fan state
 *      lang         / dil        -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Röle modülünü IO12'ye bağlı sokete takın (başka bir soket
 * kullanırsanız RELAY_PIN değerini değiştirin). / Plug the relay module into the IO12
 * socket (change RELAY_PIN if you use another socket).
 * Desteklenen pinler / Supported pins: IO4 - IO12 - IO13 - IO14 (IO5 = buzzer)
 */

#define USE_ESPNOW
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

#define RELAY_PIN IO12 // Röle modülünün bağlı olduğu pin / Pin the relay module is connected to

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// Bu eşiğin ÜSTÜNDEKİ değerler "sıcak" sayılır - ortamınıza göre ayarlayın.
// Values ABOVE this threshold count as "hot" - adjust to your environment.
int hotThresholdC = 28;

bool manualMode = false;     // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
bool fanOn = false;
int lastTempC = -999;        // Son gelen sıcaklık (-999 = henüz yok) / last temperature (-999 = none yet)
uint32_t lastTempMs = 0;     // Ne zaman geldi / when it arrived

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

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "EŞİK" -> "esik"
// Lower-cases and simplifies Turkish letters: "EŞİK" -> "esik"
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
// Fan ve mesajlar / Fan and messages
// ---------------------------------------------------------------------------
void setFan(bool on) {
  fanOn = on;
  minibot.moduleRelayWrite(RELAY_PIN, on);
  minibot.ledWrite(on);
  minibot.serialWrite(on ? L("VANTİLATÖR AÇIK", "FAN ON") : L("VANTİLATÖR KAPALI", "FAN OFF"));
}

// Otomatik modda sıcaklığa göre fanı ayarla / in auto mode set the fan by temperature
void applyAuto() {
  if (manualMode || lastTempC == -999) return;
  bool shouldBeOn = lastTempC > hotThresholdC;
  if (shouldBeOn != fanOn) {
    minibot.serialWrite(String(L("Sıcaklık ", "Temperature ")) + lastTempC + " °C " +
                        (shouldBeOn ? L("> eşik (sıcak)", "> threshold (hot)") : L("<= eşik (serin)", "<= threshold (cool)")));
    setFan(shouldBeOn);
  }
}

void printStatus() {
  String line = String(manualMode ? L("Mod: MANUEL", "Mode: MANUAL") : L("Mod: OTOMATİK", "Mode: AUTO")) + "  |  " +
                (fanOn ? L("Fan: AÇIK", "Fan: ON") : L("Fan: kapalı", "Fan: off")) + "  |  " +
                L("Eşik: ", "Threshold: ") + hotThresholdC + " °C  |  ";
  if (lastTempC == -999) line += L("Henüz sıcaklık gelmedi", "No temperature yet");
  else line += String(L("Son sıcaklık: ", "Last temperature: ")) + lastTempC + " °C (" + ((millis() - lastTempMs) / 1000) + L(" sn önce)", " s ago)");
  minibot.serialWrite(line);
}

void printHelp() {
  minibot.serialWrite(L("---- OTOMATİK VANTİLATÖR - Komutlar ----", "---- AUTOMATIC FAN - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  oto / manuel  : otomatik / manuel mod", "  auto / manual : auto / manual mode"));
  minibot.serialWrite(L("  ac / kapat    : fanı aç / kapat", "  on / off      : fan on / off"));
  minibot.serialWrite(L("  esik 15-45    : sıcaklık eşiği (°C)", "  threshold 15-45: temperature threshold (°C)"));
  minibot.serialWrite(L("  durum         : son sıcaklık ve fan", "  status        : last temperature and fan"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu     : OTOMATİK <-> MANUEL", "  B1 button     : AUTO <-> MANUAL"));
}

void setMode(bool manual) {
  manualMode = manual;
  minibot.buzzerPlay(manual ? 1500 : 1000, 60); // Kısa bip / short beep
  if (manual) {
    minibot.serialWrite(L(">> MANUEL mod: \"ac\" / \"kapat\" yazın.", ">> MANUAL mode: type \"on\" / \"off\"."));
  } else {
    minibot.serialWrite(L(">> OTOMATİK mod: fan sıcaklığa göre çalışır.", ">> AUTO mode: the fan follows the temperature."));
    applyAuto();
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
  } else if (word == "ac" || word == "on") {
    if (!manualMode) setMode(true);
    setFan(true);
  } else if (word == "kapat" || word == "off") {
    if (!manualMode) setMode(true);
    setFan(false);
  } else if ((word == "esik" || word == "threshold") && hasValue) {
    hotThresholdC = constrain(value, 15, 45);
    minibot.serialWrite(String(L("Eşik: ", "Threshold: ")) + hotThresholdC + " °C");
    applyAuto();
  } else if (word == "durum" || word == "status") {
    printStatus();
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
  minibot.initESPNow();
  minibot.startListening();    // Gelen IOTBOT yayınını minibot.receivedData'ya yazar / fills minibot.receivedData
  minibot.moduleRelayWrite(RELAY_PIN, false);
  minibot.ledWrite(false);
  minibot.serialWrite(L("Otomatik vantilatör hazır - IOTBOT'tan sıcaklık verisi bekleniyor...",
                        "Automatic fan ready - waiting for temperature data from IOTBOT..."));
  printHelp();
}

void loop() {
  // 1) B1 -> mod değiştir / B1 -> toggle mode
  if (b1Pressed()) setMode(!manualMode);

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Gelen veri: sadece IOTBOT sıcaklık yayını (deviceType 11) kabul edilir.
  // 3) Incoming data: only an IOTBOT temperature broadcast (deviceType 11) is accepted.
  if (minibot.newData) {
    minibot.newData = false;
    if (minibot.receivedData.deviceType == 11) {
      int tempC = minibot.receivedData.axis1;
      lastTempMs = millis();
      if (tempC != lastTempC) { // Sadece değişince yaz / print only on change
        lastTempC = tempC;
        minibot.serialWrite(String(L("IOTBOT sıcaklığı: ", "IOTBOT temperature: ")) + tempC + " °C");
      }
      applyAuto();
    }
  }
}
