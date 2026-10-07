/*
 * TR: ESP-NOW GÖNDERİCİ ÖRNEĞİ (kendi veri yapımızla)
 *  - Her 2 saniyede bir; metin, tam sayı (1-19 arası rastgele), ondalık sayı ve
 *    mantıksal (bool) değer içeren bir paket gönderir. MINIBOT_ESP_NOW_Receiver_
 *    Example.ino yüklü bir kart bu paketi alıp ekrana yazar.
 *  - Gönderici ile alıcıdaki struct_message yapısı BİREBİR aynı olmalıdır.
 *  - Varsayılan adres FF:FF:FF:FF:FF:FF = YAYIN (herkese). Sadece tek bir karta
 *    göndermek için alıcının MAC adresini aşağıya yazın (alıcı kendi MAC'ini yazar).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim   / help         -> komut listesi
 *      gonder   / send         -> hemen bir paket gönder
 *      dur      / pause        -> otomatik gönderimi duraklat
 *      devam    / resume       -> otomatik gönderime devam et
 *      aralik 5 / interval 5   -> gönderim aralığı (saniye, 1-60)
 *      dil      / lang         -> dili değiştir (Türkçe <-> English)
 *
 * EN: ESP-NOW SENDER EXAMPLE (with our own data structure)
 *  - Every 2 seconds it sends a packet with a text, an integer (random 1-19), a float
 *    and a boolean. A board running MINIBOT_ESP_NOW_Receiver_Example.ino receives and
 *    prints it.
 *  - The struct_message structure must be EXACTLY the same on sender and receiver.
 *  - The default address FF:FF:FF:FF:FF:FF = BROADCAST (everyone). To send to one
 *    board only, put the receiver's MAC address below (the receiver prints its MAC).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help       / yardim     -> command list
 *      send       / gonder     -> send a packet now
 *      pause      / dur        -> pause automatic sending
 *      resume     / devam      -> resume automatic sending
 *      interval 5 / aralik 5   -> sending interval (seconds, 1-60)
 *      lang       / dil        -> switch language (Turkish <-> English)
 */

#define USE_ESPNOW
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// ALICININ MAC ADRESİNİ YAZIN (FF:FF... = herkese yayın)
// REPLACE WITH YOUR RECEIVER MAC Address (FF:FF... = broadcast to everyone)
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Gönderilecek veri yapısı - alıcıdakiyle AYNI olmalı.
// Structure to send - must match the receiver structure.
typedef struct struct_message {
  char a[32];
  int b;
  float c;
  bool d;
} struct_message;

struct_message myData;

bool autoSend = true;
uint32_t intervalMs = 2000;
uint32_t lastSendMs = 0;
uint32_t sentCount = 0;

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
// Gönderim ve mesajlar / Sending and messages
// ---------------------------------------------------------------------------
void sendPacket() {
  // Gönderilecek değerleri hazırla / set values to send
  memset(&myData, 0, sizeof(myData));
  strncpy(myData.a, L("MINIBOT'tan merhaba", "Hello from MINIBOT"), sizeof(myData.a) - 1);
  myData.b = random(1, 20);
  myData.c = 1.2;
  myData.d = false;

  // ESP-NOW ile gönder / send via ESP-NOW
  minibot.sendESPNow(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  sentCount++;
  minibot.serialWrite(String(L("Gönderildi #", "Sent #")) + sentCount + " -> a=\"" + myData.a + "\" b=" + myData.b +
                      " c=" + String(myData.c, 2) + " d=" + (myData.d ? "true" : "false"));
}

void printHelp() {
  minibot.serialWrite(L("---- ESP-NOW GÖNDERİCİ - Komutlar ----", "---- ESP-NOW SENDER - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  gonder        : hemen gönder", "  send          : send now"));
  minibot.serialWrite(L("  dur / devam   : otomatik gönderimi duraklat / sürdür", "  pause / resume: pause / resume automatic sending"));
  minibot.serialWrite(L("  aralik 1-60   : gönderim aralığı (saniye)", "  interval 1-60 : sending interval (seconds)"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "gonder" || word == "send") {
    sendPacket();
  } else if (word == "dur" || word == "pause" || word == "stop") {
    autoSend = false;
    minibot.serialWrite(L("Otomatik gönderim duraklatıldı.", "Automatic sending paused."));
  } else if (word == "devam" || word == "resume") {
    autoSend = true;
    minibot.serialWrite(L("Otomatik gönderim sürüyor.", "Automatic sending resumed."));
  } else if ((word == "aralik" || word == "interval") && hasValue) {
    intervalMs = (uint32_t)constrain(value, 1, 60) * 1000;
    minibot.serialWrite(String(L("Gönderim aralığı: ", "Sending interval: ")) + (intervalMs / 1000) + L(" saniye", " seconds"));
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
  minibot.serialWrite(L("ESP-NOW Gönderici Örneği", "ESP-NOW Sender Example"));

  WiFi.mode(WIFI_STA);  // Kartı WiFi istasyonu yap / set the board as a Wi-Fi station
  minibot.initESPNow(); // ESP-NOW'u başlat / init ESP-NOW
  printHelp();
}

void loop() {
  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) Aralık dolunca gönder (delay yok) / send when the interval is up (no delay)
  if (autoSend && millis() - lastSendMs >= intervalMs) {
    lastSendMs = millis();
    sendPacket();
  }
}
