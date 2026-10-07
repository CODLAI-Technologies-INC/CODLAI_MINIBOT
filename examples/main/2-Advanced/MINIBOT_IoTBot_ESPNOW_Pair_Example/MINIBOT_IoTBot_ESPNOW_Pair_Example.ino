/*
 * TR: MINIBOT <-> IOTBOT ESP-NOW EŞLEŞMESİ
 *  - MINIBOT'u, aynı odadaki bir IOTBOT ile ROUTER/WIFI AĞI OLMADAN (ESP-NOW ile)
 *    doğrudan haberleştirir. MINIBOT yarım saniyede bir kendi B1 butonunun durumunu
 *    ve artan bir sayacı IOTBOT'a gönderir; IOTBOT'un potansiyometre değerini geri
 *    alıp kendi mavi LED'ini o değere göre yanıp söndürür (pot yükseldikçe daha hızlı).
 *    IOTBOT'un B3 butonu basılıysa LED sabit yanar.
 *  - B1 bu örnekte IOTBOT'a gönderilen butondur (basılıyken IOTBOT bunu görür).
 *  - MINIBOT paketleri 41 kimliğiyle (deviceType, MINIBOT örnek kartı) gönderir.
 *  - MINIBOT'ta LCD ekran YOK; tüm bilgiler seri port (USB) üzerinden verilir.
 *  - Eşlenecek IOTBOT'a CODLAI_IOTBOT kütüphanesindeki IOTBOT_MiniBot_ESPNOW_Pair_
 *    Example.ino dosyasını yükleyin.
 *  - ÖNEMLİ: Aşağıdaki kPeerMac dizisini GERÇEK IOTBOT'unuzun MAC adresiyle
 *    değiştirin (IOTBOT tarafındaki kod seri porta kendi MAC'ini yazar).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help     -> komut listesi
 *      mac               -> bu kartın ve eşin MAC adresleri
 *      durum  / status   -> son pot değeri, gönderilen paket sayısı
 *      dil    / lang     -> dili değiştir (Türkçe <-> English)
 *
 * EN: MINIBOT <-> IOTBOT ESP-NOW PAIRING
 *  - Talks directly (peer-to-peer, no router/WiFi network needed) with an IOTBOT in
 *    the same room over ESP-NOW. Every half second MINIBOT sends its own B1 button
 *    state and an increasing counter to the IOTBOT; it receives the IOTBOT's
 *    potentiometer value back and blinks its blue LED at a rate based on it (higher
 *    pot = faster). The LED stays solid on while the IOTBOT's B3 button is held.
 *  - In this example B1 is the button sent to the IOTBOT (the IOTBOT sees it while held).
 *  - MINIBOT sends its packets with ID 41 (deviceType, the MINIBOT example board).
 *  - MINIBOT has NO LCD screen; all feedback is given through Serial (USB).
 *  - Upload IOTBOT_MiniBot_ESPNOW_Pair_Example.ino (in the CODLAI_IOTBOT library's
 *    examples) to the IOTBOT you want to pair with.
 *  - IMPORTANT: Replace kPeerMac below with your actual IOTBOT's MAC address (the
 *    IOTBOT-side sketch prints its own MAC to Serial).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim   -> command list
 *      mac               -> this board's and the peer's MAC addresses
 *      status / durum    -> last pot value, packets sent
 *      lang   / dil      -> switch language (Turkish <-> English)
 */

#define USE_ESPNOW
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// Test için kullanılan gerçek bir IOTBOT'un MAC adresi - kendi kartınıza göre
// değiştirin. / A real IOTBOT's MAC address used for testing - change this to
// match your own board.
uint8_t kPeerMac[] = {0x30, 0x83, 0x98, 0x46, 0x3A, 0xB8};

uint32_t lastSendMs = 0;
const uint32_t kSendIntervalMs = 500;
uint32_t counter = 0;

uint32_t lastBlinkMs = 0;
bool ledState = false;
int blinkIntervalMs = 500;   // IOTBOT'un pot değerine göre güncellenir / updated from IOTBOT's pot value
bool remoteButtonHeld = false;
int iotbotPot = -1;          // Son pot değeri (-1 = henüz yok) / last pot value (-1 = none yet)
int lastPrintedPot = -1000;
uint32_t lastPotPrintMs = 0;

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
// Mesajlar / Messages
// ---------------------------------------------------------------------------
String macToString(const uint8_t *mac) {
  char text[18];
  snprintf(text, sizeof(text), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(text);
}

void printHelp() {
  minibot.serialWrite(L("---- IOTBOT EŞLEŞMESİ - Komutlar ----", "---- IOTBOT PAIRING - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  mac           : MAC adresleri", "  mac           : MAC addresses"));
  minibot.serialWrite(L("  durum         : pot değeri, paket sayısı", "  status        : pot value, packet count"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu     : IOTBOT'a gönderilir", "  B1 button     : sent to the IOTBOT"));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "mac") {
    minibot.serialWrite(String(L("Benim MAC adresim: ", "My MAC address: ")) + WiFi.macAddress());
    minibot.serialWrite(String(L("Eş (IOTBOT) MAC  : ", "Peer (IOTBOT) MAC: ")) + macToString(kPeerMac));
  } else if (cmd == "durum" || cmd == "status") {
    String line = String(L("Gönderilen paket: ", "Packets sent: ")) + counter + "  |  ";
    if (iotbotPot < 0) line += L("IOTBOT'tan henüz veri yok (MAC doğru mu?)", "No data from the IOTBOT yet (is the MAC right?)");
    else line += String(L("IOTBOT pot: ", "IOTBOT pot: ")) + iotbotPot + (remoteButtonHeld ? L("  |  IOTBOT B3 basılı", "  |  IOTBOT B3 held") : "");
    minibot.serialWrite(line);
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
  minibot.serialWrite(L("ESP-NOW başlatılıyor...", "Starting ESP-NOW..."));

  minibot.initESPNow();
  minibot.setWiFiChannel(1); // İki taraf da AYNI kanalda olmalı / Both sides must use the SAME channel.
  minibot.startListening();  // Gelen mesajları minibot.receivedData'ya yazar / Fills minibot.receivedData on arrival.

  minibot.serialWrite(String(L("Benim MAC adresim: ", "My MAC address: ")) + WiFi.macAddress());
  minibot.serialWrite(L("ESP-NOW hazır, IOTBOT ile eşleşme aktif.", "ESP-NOW ready, paired with IOTBOT."));
  printHelp();
}

void loop() {
  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) Gönderim: buton durumu + sayaç / Sending: button state + counter
  if (millis() - lastSendMs >= kSendIntervalMs) {
    lastSendMs = millis();
    counter++;
    CodlaiESPNowMessage outgoing = {}; // Tüm alanlar sıfır / all fields zero
    outgoing.deviceType = 41; // 41 = MINIBOT örnek kartı kimliği (40-49 örnekler için ayrıldı) / MINIBOT example board ID (40-49 reserved for examples)
    outgoing.axis1 = counter;
    outgoing.axis2 = 0;
    outgoing.axis3 = 0;
    outgoing.gripper = 0;
    outgoing.action = minibot.button1Read() ? 0 : 1; // button1Read() aktif-LOW / active-LOW
    minibot.sendESPNow(kPeerMac, (uint8_t *)&outgoing, sizeof(outgoing));
  }

  // 3) Alış: IOTBOT'tan gelen veri / Receiving: data from the IOTBOT
  if (minibot.newData) {
    minibot.newData = false;
    iotbotPot = minibot.receivedData.axis1;
    remoteButtonHeld = minibot.receivedData.action == 1;
    // Pot değeri (0-4095) yanıp sönme aralığına (900 ms..80 ms) eşleniyor -
    // pot ne kadar yüksekse LED o kadar hızlı yanıp söner.
    // Pot value (0-4095) maps to a blink interval (900 ms..80 ms) - the
    // higher the pot, the faster the LED blinks.
    blinkIntervalMs = map(constrain(iotbotPot, 0, 4095), 0, 4095, 900, 80);
    // Pot belirgin değişince yaz (en fazla 300 ms'de bir) / print on a clear change (at most every 300 ms)
    if (abs(iotbotPot - lastPrintedPot) >= 50 && millis() - lastPotPrintMs >= 300) {
      lastPrintedPot = iotbotPot;
      lastPotPrintMs = millis();
      minibot.serialWrite(String(L("IOTBOT pot değeri: ", "IOTBOT pot value: ")) + iotbotPot);
    }
  }

  // 4) LED geri bildirimi / LED feedback
  if (remoteButtonHeld) {
    minibot.ledWrite(true);
  } else if (millis() - lastBlinkMs >= (uint32_t)blinkIntervalMs) {
    lastBlinkMs = millis();
    ledState = !ledState;
    minibot.ledWrite(ledState);
  }
}
