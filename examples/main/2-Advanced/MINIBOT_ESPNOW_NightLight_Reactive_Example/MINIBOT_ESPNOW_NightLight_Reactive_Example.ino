/*
 * TR: KABLOSUZ AKILLI EV FİKRİ - OTOMATİK GECE LAMBASI
 *  - Bu MINIBOT'un kendi ışık sensörü YOK - bunun yerine, aynı odadaki bir IOTBOT'un
 *    yayınladığı (broadcast) ışık sensörü verisini ESP-NOW ile dinler ve OTOMATİK
 *    modda hava kararınca (ışık değeri eşiğin altına düşünce) kendi mavi LED'ini yakar.
 *    İki kartı kablo OLMADAN birlikte çalışır hale getiriyoruz.
 *  - Önce IOTBOT_ESPNOW_LightSensor_Broadcast_Example.ino dosyasını bir IOTBOT'a,
 *    sonra bu kodu bir MINIBOT'a yükleyin. IOTBOT'un ışık sensörünü elinizle
 *    kapatınca MINIBOT'un LED'i yanacak!
 *  - B1 butonuna basınca MANUEL moda geçer: lambayı seri porttan siz açıp
 *    kapatırsınız. B1'e tekrar basınca otomatiğe döner.
 *  - MINIBOT'ta LCD ekran YOK; tüm bilgiler seri port (USB) üzerinden verilir.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim    / help            -> komut listesi
 *      oto       / auto            -> otomatik mod (ışığa göre)
 *      manuel    / manual          -> manuel mod
 *      ac        / on              -> lambayı yak (manuel moda geçer)
 *      kapat     / off             -> lambayı söndür (manuel moda geçer)
 *      esik 1200 / threshold 1200  -> karanlık eşiği (0-4095)
 *      durum     / status          -> son ışık değeri ve lamba durumu
 *      dil       / lang            -> dili değiştir (Türkçe <-> English)
 *
 * EN: A WIRELESS SMART HOME IDEA - AUTOMATIC NIGHT LIGHT
 *  - This MINIBOT has NO light sensor of its own - instead, it listens over ESP-NOW to
 *    the light sensor data broadcast by an IOTBOT in the same room and, in AUTO mode,
 *    turns its own blue LED on when it gets dark (the light value drops below the
 *    threshold). Two boards work together WITHOUT any wire.
 *  - First upload IOTBOT_ESPNOW_LightSensor_Broadcast_Example.ino to an IOTBOT, then
 *    this code to a MINIBOT. Cover the IOTBOT's light sensor with your hand and watch
 *    the MINIBOT's LED turn on!
 *  - Press B1 to switch to MANUAL mode: you switch the lamp on/off from the serial
 *    port. Press B1 again to go back to auto.
 *  - MINIBOT has NO LCD screen; all feedback is given through Serial (USB).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help           / yardim     -> command list
 *      auto           / oto        -> auto mode (by light)
 *      manual         / manuel     -> manual mode
 *      on             / ac         -> lamp on (switches to manual)
 *      off            / kapat      -> lamp off (switches to manual)
 *      threshold 1200 / esik 1200  -> darkness threshold (0-4095)
 *      status         / durum      -> last light value and lamp state
 *      lang           / dil        -> switch language (Turkish <-> English)
 */

#define USE_ESPNOW
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// Bu eşiğin ALTINDAKİ değerler "karanlık" sayılır - IOTBOT'unuzun ortam ışığına göre
// ayarlayın. / Values BELOW this threshold count as "dark" - adjust to your IOTBOT's
// ambient light level.
int darkThreshold = 1500;

bool manualMode = false;     // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
bool lampOn = false;
int lastLight = -1;          // Son ışık değeri (-1 = henüz yok) / last light value (-1 = none yet)
int lastPrintedLight = -1000;

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
// Lamba ve mesajlar / Lamp and messages
// ---------------------------------------------------------------------------
void setLamp(bool on) {
  lampOn = on;
  minibot.ledWrite(on);
  minibot.serialWrite(on ? L("LAMBA AÇIK", "LAMP ON") : L("LAMBA KAPALI", "LAMP OFF"));
}

// Otomatik modda ışığa göre lambayı ayarla / in auto mode set the lamp by the light value
void applyAuto() {
  if (manualMode || lastLight < 0) return;
  bool shouldBeOn = lastLight < darkThreshold;
  if (shouldBeOn != lampOn) {
    minibot.serialWrite(String(L("Işık değeri ", "Light value ")) + lastLight +
                        (shouldBeOn ? L(" -> karanlık", " -> dark") : L(" -> aydınlık", " -> bright")));
    setLamp(shouldBeOn);
  }
}

void printHelp() {
  minibot.serialWrite(L("---- GECE LAMBASI - Komutlar ----", "---- NIGHT LIGHT - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  oto / manuel  : otomatik / manuel mod", "  auto / manual : auto / manual mode"));
  minibot.serialWrite(L("  ac / kapat    : lambayı yak / söndür", "  on / off      : lamp on / off"));
  minibot.serialWrite(L("  esik 0-4095   : karanlık eşiği", "  threshold 0-4095: darkness threshold"));
  minibot.serialWrite(L("  durum         : son ışık değeri", "  status        : last light value"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu     : OTOMATİK <-> MANUEL", "  B1 button     : AUTO <-> MANUAL"));
}

void setMode(bool manual) {
  manualMode = manual;
  minibot.buzzerPlay(manual ? 1500 : 1000, 60); // Kısa bip / short beep
  if (manual) {
    minibot.serialWrite(L(">> MANUEL mod: \"ac\" / \"kapat\" yazın.", ">> MANUAL mode: type \"on\" / \"off\"."));
  } else {
    minibot.serialWrite(L(">> OTOMATİK mod: lamba ışığa göre yanar.", ">> AUTO mode: the lamp follows the light."));
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
    setLamp(true);
  } else if (word == "kapat" || word == "off") {
    if (!manualMode) setMode(true);
    setLamp(false);
  } else if ((word == "esik" || word == "threshold") && hasValue) {
    darkThreshold = constrain(value, 0, 4095);
    minibot.serialWrite(String(L("Karanlık eşiği: ", "Darkness threshold: ")) + darkThreshold);
    applyAuto();
  } else if (word == "durum" || word == "status") {
    String line = String(manualMode ? L("Mod: MANUEL", "Mode: MANUAL") : L("Mod: OTOMATİK", "Mode: AUTO")) + "  |  " +
                  (lampOn ? L("Lamba: AÇIK", "Lamp: ON") : L("Lamba: kapalı", "Lamp: off")) + "  |  " +
                  L("Eşik: ", "Threshold: ") + darkThreshold + "  |  ";
    if (lastLight < 0) line += L("Henüz ışık verisi gelmedi", "No light data yet");
    else line += String(L("Son ışık: ", "Last light: ")) + lastLight;
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
  minibot.initESPNow();
  minibot.startListening();    // Gelen IOTBOT yayınını minibot.receivedData'ya yazar / fills minibot.receivedData
  minibot.ledWrite(false);
  minibot.serialWrite(L("Otomatik gece lambası hazır - IOTBOT'tan ışık verisi bekleniyor...",
                        "Automatic night light ready - waiting for light data from IOTBOT..."));
  printHelp();
}

void loop() {
  // 1) B1 -> mod değiştir / B1 -> toggle mode
  if (b1Pressed()) setMode(!manualMode);

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Gelen veri: sadece IOTBOT ışık yayını (deviceType 10) kabul edilir.
  // 3) Incoming data: only an IOTBOT light broadcast (deviceType 10) is accepted.
  if (minibot.newData) {
    minibot.newData = false;
    if (minibot.receivedData.deviceType == 10) {
      lastLight = minibot.receivedData.axis1;
      // Belirgin değişimde yaz (titreşen değerler seri portu doldurmasın).
      // Print on a clear change (so jittery values do not flood the serial port).
      if (abs(lastLight - lastPrintedLight) >= 100) {
        lastPrintedLight = lastLight;
        minibot.serialWrite(String(L("IOTBOT ışık değeri: ", "IOTBOT light value: ")) + lastLight);
      }
      applyAuto();
    }
  }
}
