/*
 * TR: ESP-NOW'A İLK ADIM - en basit kablosuz haberleşme örneği
 *  - Bir MAC adresi bilmenize GEREK YOK: bu kod her saniye bir sayacı "yayın"
 *    (broadcast) olarak havaya gönderir ve aynı odada ESP-NOW ile dinleyen HERHANGİ
 *    bir CODLAI kartı (başka bir MINIBOT, bir IOTBOT ya da bir ROLEBOT - hepsi aynı
 *    veri yapısını kullanır) bunu duyabilir. Aynı anda hem gönderiyor hem dinliyoruz.
 *    Bir yayın gelince mavi LED kısa bir an yanar.
 *  - Gönderen kart, paketteki kimlikten (deviceType) tanınır: 40 = IOTBOT,
 *    41 = MINIBOT, 42 = ROLEBOT (40-49 kütüphane örneklerine ayrılmıştır).
 *  - Daha sonra MINIBOT_IoTBot_ESPNOW_Pair_Example.ino ile İKİ KART ARASINDA ÖZEL
 *    (MAC adresine dayalı) bir eşleşme yapmayı öğrenebilirsiniz.
 *  - MINIBOT'ta LCD ekran YOK; tüm bilgiler seri port (USB) üzerinden verilir.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help      -> komut listesi
 *      dur    / pause     -> göndermeyi duraklat (dinleme sürer)
 *      devam  / resume    -> göndermeye devam et
 *      gonder / send      -> hemen bir yayın gönder
 *      mac                -> bu kartın MAC adresini yaz
 *      dil    / lang      -> dili değiştir (Türkçe <-> English)
 *
 * EN: FIRST STEP INTO ESP-NOW - the simplest wireless example
 *  - You do NOT need to know any MAC address: this code broadcasts a counter into the
 *    air every second, and ANY nearby CODLAI board listening over ESP-NOW (another
 *    MINIBOT, an IOTBOT, or a ROLEBOT - they all share the same data structure) can
 *    hear it. We both send AND listen at the same time. The blue LED flashes briefly
 *    when a broadcast arrives.
 *  - The sender is recognised by the ID in the packet (deviceType): 40 = IOTBOT,
 *    41 = MINIBOT, 42 = ROLEBOT (40-49 are reserved for the library examples).
 *  - Later, see MINIBOT_IoTBot_ESPNOW_Pair_Example.ino to learn how to pair TWO
 *    SPECIFIC boards together using their MAC addresses.
 *  - MINIBOT has NO LCD screen; all feedback is given through Serial (USB).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim    -> command list
 *      pause  / dur       -> pause sending (listening continues)
 *      resume / devam     -> resume sending
 *      send   / gonder    -> send a broadcast now
 *      mac                -> print this board's MAC address
 *      lang   / dil       -> switch language (Turkish <-> English)
 */

#define USE_ESPNOW
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint32_t counter = 0;
uint32_t lastSendMs = 0;
const uint32_t kSendIntervalMs = 1000;
bool sending = true;       // Gönderim açık mı / sending enabled?
uint32_t ledOffAtMs = 0;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "GÖNDER" -> "gonder"
// Lower-cases and simplifies Turkish letters: "GÖNDER" -> "gonder"
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
// ESP-NOW
// ---------------------------------------------------------------------------
void sendCounter() {
  counter++;
  CodlaiESPNowMessage outgoing = {}; // Tüm alanlar sıfır (metin alanı boş) / all fields zero (empty text field)
  outgoing.deviceType = 41; // 41 = MINIBOT örnek kartı kimliği (40-49 örnekler için ayrıldı) / MINIBOT example board ID (40-49 reserved for examples)
  outgoing.axis1 = counter;
  outgoing.axis2 = 0;
  outgoing.axis3 = 0;
  outgoing.gripper = 0;
  outgoing.action = 0;
  minibot.sendESPNow(broadcastAddress, (uint8_t *)&outgoing, sizeof(outgoing));
  minibot.serialWrite(String(L("Gönderilen: ", "Sent: ")) + counter);
}

void printHelp() {
  minibot.serialWrite(L("---- ESP-NOW YAYIN - Komutlar ----", "---- ESP-NOW BROADCAST - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  dur / devam   : göndermeyi duraklat / sürdür", "  pause / resume: pause / resume sending"));
  minibot.serialWrite(L("  gonder        : hemen gönder", "  send          : send now"));
  minibot.serialWrite(L("  mac           : bu kartın MAC adresi", "  mac           : this board's MAC address"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "dur" || cmd == "pause" || cmd == "stop") {
    sending = false;
    minibot.serialWrite(L("Gönderim duraklatıldı (dinleme sürüyor).", "Sending paused (still listening)."));
  } else if (cmd == "devam" || cmd == "resume") {
    sending = true;
    minibot.serialWrite(L("Gönderim sürüyor.", "Sending resumed."));
  } else if (cmd == "gonder" || cmd == "send") {
    sendCounter();
  } else if (cmd == "mac") {
    minibot.serialWrite(String(L("Benim MAC adresim: ", "My MAC address: ")) + WiFi.macAddress());
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
  minibot.serialWrite(L("ESP-NOW yayın modu başlatılıyor...", "Starting ESP-NOW broadcast mode..."));

  minibot.initESPNow();
  minibot.startListening(); // Gelen HERHANGİ bir yayını minibot.receivedData'ya yazar / fills minibot.receivedData

  minibot.serialWrite(L("Yayın modu hazır - herkese açığız!", "Broadcast mode ready - open to everyone!"));
  printHelp();
}

void loop() {
  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) Gönderim: her saniye sayacı yayınla / Sending: broadcast the counter every second
  if (sending && millis() - lastSendMs >= kSendIntervalMs) {
    lastSendMs = millis();
    sendCounter();
  }

  // 3) Alış: başka bir karttan gelen HERHANGİ bir yayın / Receiving: ANY broadcast from another board
  if (minibot.newData) {
    minibot.newData = false;
    const char *senderName = "?";
    switch (minibot.receivedData.deviceType) {
      // Örnek kartı kimlikleri 40-49 (bkz. CodlaiESPNowMessage) / example board IDs 40-49 (see CodlaiESPNowMessage)
      case 40: senderName = "IOTBOT"; break;
      case 41: senderName = "MINIBOT"; break;
      case 42: senderName = "ROLEBOT"; break;
      default: break;
    }
    minibot.serialWrite(String(L("Yayın alındı -> gönderen: ", "Broadcast received -> from: ")) + senderName +
                        " (" + minibot.receivedData.deviceType + ")" + L(", değer: ", ", value: ") + minibot.receivedData.axis1);
    minibot.ledWrite(true);       // Kısa LED flaşı (beklemeden) / short LED flash (non-blocking)
    ledOffAtMs = millis() + 50;
  }

  // 4) Mavi LED'i söndür / turn the blue LED off
  if (ledOffAtMs != 0 && (int32_t)(millis() - ledOffAtMs) >= 0) {
    ledOffAtMs = 0;
    minibot.ledWrite(false);
  }
}
