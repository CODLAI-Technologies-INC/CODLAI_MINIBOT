/*
 * TR: EĞLENCELİ KABLOSUZ ÖRNEK - Uzaktan akıllı LED kumandası
 *  - Bu MINIBOT'un B1 butonuna her bastığınızda, uzaktaki bir IOTBOT'un akıllı LED
 *    serisine (NeoPixel) "bir sonraki efekte geç" komutu gönderilir. Hiçbir kablo
 *    yok - sadece ESP-NOW. Efektler: 0 = gökkuşağı, 1 = gökkuşağı takip,
 *    2 = tiyatro takip, 3 = renk silme.
 *  - Önce IOTBOT_MiniBot_SmartLED_Remote_Example.ino dosyasını bir IOTBOT'a, sonra
 *    bu kodu bir MINIBOT'a yükleyin.
 *  - ÖNEMLİ: kPeerMac dizisini kendi IOTBOT'unuzun MAC adresiyle değiştirin.
 *  - B1 bu örnekte kumanda butonudur.
 *  - MINIBOT paketleri 41 kimliğiyle (deviceType, MINIBOT örnek kartı) gönderir.
 *  - MINIBOT'ta LCD ekran YOK; tüm bilgiler seri port (USB) üzerinden verilir.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim   / help       -> komut listesi
 *      sonraki  / next       -> sonraki efekt (B1 gibi)
 *      efekt 2  / effect 2   -> doğrudan o efekti gönder (0-3)
 *      mac                   -> bu kartın ve eşin MAC adresleri
 *      dil      / lang       -> dili değiştir (Türkçe <-> English)
 *
 * EN: A FUN WIRELESS EXAMPLE - Remote smart LED control
 *  - Every time you press this MINIBOT's B1 button, a "switch to the next effect"
 *    command is sent to a remote IOTBOT's smart LED strip (NeoPixel). No wire - just
 *    ESP-NOW. Effects: 0 = rainbow, 1 = rainbow chase, 2 = theater chase,
 *    3 = color wipe.
 *  - Upload IOTBOT_MiniBot_SmartLED_Remote_Example.ino to an IOTBOT first, then this
 *    code to a MINIBOT.
 *  - IMPORTANT: Replace kPeerMac with your own IOTBOT's MAC address.
 *  - In this example B1 is the remote-control button.
 *  - MINIBOT sends its packets with ID 41 (deviceType, the MINIBOT example board).
 *  - MINIBOT has NO LCD screen; all feedback is given through Serial (USB).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help     / yardim     -> command list
 *      next     / sonraki    -> next effect (like B1)
 *      effect 2 / efekt 2    -> send that effect directly (0-3)
 *      mac                   -> this board's and the peer's MAC addresses
 *      lang     / dil        -> switch language (Turkish <-> English)
 */

#define USE_ESPNOW
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// ÖNEMLİ: Kendi IOTBOT'unuzun MAC adresiyle değiştirin (IOTBOT_MiniBot_SmartLED_
// Remote_Example.ino seri porta kendi MAC'ini yazdırır).
// IMPORTANT: Replace with your own IOTBOT's MAC address (the IOTBOT-side sketch
// prints its own MAC to Serial).
uint8_t kPeerMac[] = {0x30, 0x83, 0x98, 0x46, 0x3A, 0xB8};

uint8_t effectIndex = 0;
uint32_t ledOffAtMs = 0;

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

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir / lower-cases and simplifies Turkish letters
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
// Gönderim ve mesajlar / Sending and messages
// ---------------------------------------------------------------------------
const char *effectName(uint8_t e) {
  switch (e) {
    case 0: return L("gökkuşağı", "rainbow");
    case 1: return L("gökkuşağı takip", "rainbow chase");
    case 2: return L("tiyatro takip", "theater chase");
    default: return L("renk silme", "color wipe");
  }
}

void sendEffect() {
  CodlaiESPNowMessage outgoing = {}; // Tüm alanlar sıfır / all fields zero
  outgoing.deviceType = 41; // 41 = MINIBOT örnek kartı kimliği (40-49 örnekler için ayrıldı) / MINIBOT example board ID (40-49 reserved for examples)
  outgoing.axis1 = 0;
  outgoing.axis2 = 0;
  outgoing.axis3 = 0;
  outgoing.gripper = 0;
  outgoing.action = effectIndex; // IOTBOT efekti bu alandan okur / the IOTBOT reads the effect from this field
  minibot.sendESPNow(kPeerMac, (uint8_t *)&outgoing, sizeof(outgoing));

  minibot.serialWrite(String(L("Efekt komutu gönderildi: ", "Effect command sent: ")) + effectIndex + " (" + effectName(effectIndex) + ")");
  minibot.ledWrite(true);       // Kısa LED flaşı (beklemeden) / short LED flash (non-blocking)
  ledOffAtMs = millis() + 80;
}

String macToString(const uint8_t *mac) {
  char text[18];
  snprintf(text, sizeof(text), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(text);
}

void printHelp() {
  minibot.serialWrite(L("---- UZAKTAN LED KUMANDASI - Komutlar ----", "---- REMOTE LED CONTROL - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  sonraki       : sonraki efekt", "  next          : next effect"));
  minibot.serialWrite(L("  efekt 0-3     : o efekti gönder", "  effect 0-3    : send that effect"));
  minibot.serialWrite(L("  mac           : MAC adresleri", "  mac           : MAC addresses"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu     : sonraki efekt", "  B1 button     : next effect"));
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "sonraki" || word == "next") {
    effectIndex = (effectIndex + 1) % 4;
    sendEffect();
  } else if ((word == "efekt" || word == "effect") && hasValue) {
    if (value < 0 || value > 3) {
      minibot.serialWrite(L("Efekt 0-3 arası olmalı.", "Effect must be 0-3."));
      return;
    }
    effectIndex = value;
    sendEffect();
  } else if (word == "mac") {
    minibot.serialWrite(String(L("Benim MAC adresim: ", "My MAC address: ")) + WiFi.macAddress());
    minibot.serialWrite(String(L("Eş (IOTBOT) MAC  : ", "Peer (IOTBOT) MAC: ")) + macToString(kPeerMac));
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
  minibot.ledWrite(false);
  minibot.serialWrite(L("Uzaktan LED kumandası hazır - B1'e basarak efekt değiştirin!",
                        "Remote LED control ready - press B1 to change the effect!"));
  printHelp();
}

void loop() {
  // 1) B1 -> sonraki efekt / B1 -> next effect
  if (b1Pressed()) {
    effectIndex = (effectIndex + 1) % 4;
    sendEffect();
  }

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Mavi LED'i söndür / turn the blue LED off
  if (ledOffAtMs != 0 && (int32_t)(millis() - ledOffAtMs) >= 0) {
    ledOffAtMs = 0;
    minibot.ledWrite(false);
  }
}
