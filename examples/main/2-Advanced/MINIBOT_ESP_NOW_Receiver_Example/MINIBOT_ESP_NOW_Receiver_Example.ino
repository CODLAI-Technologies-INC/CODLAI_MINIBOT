/*
 * TR: ESP-NOW ALICI ÖRNEĞİ (kendi veri yapımızla)
 *  - MINIBOT_ESP_NOW_Sender_Example.ino'nun gönderdiği paketi dinler ve içindeki
 *    metin, tam sayı, ondalık sayı ve mantıksal (bool) değeri seri porta yazar.
 *    Paket gelince mavi LED kısa bir an yanar.
 *  - Gönderici ile alıcıdaki struct_message yapısı BİREBİR aynı olmalıdır.
 *  - Gelen veri ESP-NOW'un kendi "geri çağırma" (callback) fonksiyonunda sadece
 *    kopyalanır; seri porta yazmak loop() içinde yapılır (daha güvenli).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help     -> komut listesi
 *      son    / last     -> son paketi tekrar yaz
 *      sayac  / count    -> kaç paket geldi
 *      mac               -> bu kartın MAC adresi (göndericiye yazmak için)
 *      dil    / lang     -> dili değiştir (Türkçe <-> English)
 *
 * EN: ESP-NOW RECEIVER EXAMPLE (with our own data structure)
 *  - Listens for the packet sent by MINIBOT_ESP_NOW_Sender_Example.ino and prints the
 *    text, integer, float and boolean values inside it. The blue LED flashes briefly
 *    when a packet arrives.
 *  - The struct_message structure must be EXACTLY the same on sender and receiver.
 *  - The incoming data is only copied in ESP-NOW's "callback" function; printing is
 *    done in loop() (safer).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help  / yardim    -> command list
 *      last  / son       -> print the last packet again
 *      count / sayac     -> how many packets arrived
 *      mac               -> this board's MAC address (to put in the sender)
 *      lang  / dil       -> switch language (Turkish <-> English)
 */

#define USE_ESPNOW
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// Alınacak veri yapısı - göndericidekiyle AYNI olmalı.
// Structure to receive - must match the sender structure.
typedef struct struct_message {
  char a[32];
  int b;
  float c;
  bool d;
} struct_message;

struct_message myData;              // Son gelen paket / last packet received
volatile bool packetReady = false;  // Callback yeni paket koydu mu / did the callback store a new packet?
volatile uint8_t lastLen = 0;       // Gelen bayt sayısı / bytes received
uint32_t packetCount = 0;
uint32_t ledOffAtMs = 0;

// Paket gelince ESP-NOW bu fonksiyonu çağırır. Burada SADECE kopyalıyoruz.
// ESP-NOW calls this when a packet arrives. We ONLY copy it here.
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  // Gelen paket daha kısaysa yapının dışını okumayalım: sıfırla ve sadece gelen kadarını kopyala.
  // If the packet is shorter, don't read past it: clear, then copy only what arrived.
  memset(&myData, 0, sizeof(myData));
  memcpy(&myData, incomingData, len < sizeof(myData) ? len : sizeof(myData));
  myData.a[sizeof(myData.a) - 1] = '\0'; // Metin her zaman sonlansın / always terminate the text
  lastLen = len;
  packetReady = true;
}

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "SAYAÇ" -> "sayac"
// Lower-cases and simplifies Turkish letters: "SAYAÇ" -> "sayac"
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
void printPacket() {
  minibot.serialWrite(String(L("Alınan bayt: ", "Bytes received: ")) + lastLen);
  minibot.serialWrite(String(L("  Metin (char) : ", "  Text (char)  : ")) + myData.a);
  minibot.serialWrite(String(L("  Tam sayı (int): ", "  Integer (int): ")) + myData.b);
  minibot.serialWrite(String(L("  Ondalık (float): ", "  Float        : ")) + String(myData.c, 2));
  minibot.serialWrite(String(L("  Mantıksal (bool): ", "  Bool         : ")) + (myData.d ? "true" : "false"));
}

void printHelp() {
  minibot.serialWrite(L("---- ESP-NOW ALICI - Komutlar ----", "---- ESP-NOW RECEIVER - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  son           : son paketi tekrar yaz", "  last          : print the last packet again"));
  minibot.serialWrite(L("  sayac         : gelen paket sayısı", "  count         : packets received"));
  minibot.serialWrite(L("  mac           : bu kartın MAC adresi", "  mac           : this board's MAC address"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "son" || cmd == "last") {
    if (packetCount == 0) minibot.serialWrite(L("Henüz paket gelmedi.", "No packet yet."));
    else printPacket();
  } else if (cmd == "sayac" || cmd == "count") {
    minibot.serialWrite(String(L("Gelen paket sayısı: ", "Packets received: ")) + packetCount);
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
  minibot.ledWrite(false);
  minibot.serialWrite(L("ESP-NOW Alıcı Örneği", "ESP-NOW Receiver Example"));

  WiFi.mode(WIFI_STA);              // Kartı WiFi istasyonu yap / set the board as a Wi-Fi station
  minibot.initESPNow();             // ESP-NOW'u başlat / init ESP-NOW
  minibot.registerOnRecv(OnDataRecv); // Paket gelince OnDataRecv çağrılsın / call OnDataRecv on each packet

  minibot.serialWrite(String(L("Benim MAC adresim: ", "My MAC address: ")) + WiFi.macAddress());
  printHelp();
}

void loop() {
  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) Yeni paket geldiyse yaz / print a new packet
  if (packetReady) {
    packetReady = false;
    packetCount++;
    printPacket();
    minibot.ledWrite(true);
    ledOffAtMs = millis() + 50;
  }

  // 3) Mavi LED'i söndür / turn the blue LED off
  if (ledOffAtMs != 0 && (int32_t)(millis() - ledOffAtMs) >= 0) {
    ledOffAtMs = 0;
    minibot.ledWrite(false);
  }
}
