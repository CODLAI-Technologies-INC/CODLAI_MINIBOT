/*
 * TR: KABLOSUZ İLETİŞİME İLK ADIM - En basit WiFi örneği
 *  - MINIBOT'u evinizin/okulunuzun WiFi ağına bağlar; bağlantı başarılı olursa aldığı
 *    IP adresini seri portta gösterir ve mavi LED yanık kalır. Sunucu YOK, web sayfası
 *    YOK - sadece "ağa katılmak" ne demek onu öğretir.
 *  - Bağlantı koparsa ya da geri gelirse seri porta yazılır (LED de söner/yanar).
 *  - MINIBOT'ta LCD ekran olmadığı için tüm bilgiler seri port (USB) üzerinden verilir.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help      -> komut listesi
 *      durum  / status    -> bağlantı, IP, MAC ve sinyal gücü
 *      baglan / connect   -> ağa yeniden bağlanmayı dene
 *      dil    / lang      -> dili değiştir (Türkçe <-> English)
 *
 * EN: FIRST STEP INTO WIRELESS COMMUNICATION - the simplest WiFi example
 *  - Connects MINIBOT to your home/school WiFi network; once connected it shows the
 *    IP address on Serial and the blue LED stays on. NO server, NO web page - just
 *    teaches what "joining a network" means.
 *  - If the connection drops or comes back it is printed (and the LED goes off/on).
 *  - MINIBOT has no LCD screen, so all feedback is given through Serial (USB).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help    / yardim   -> command list
 *      status  / durum    -> connection, IP, MAC and signal strength
 *      connect / baglan   -> try to join the network again
 *      lang    / dil      -> switch language (Turkish <-> English)
 */

#define USE_WIFI
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

// ÖNEMLİ: Kendi WiFi ağınızın adını ve şifresini yazın.
// IMPORTANT: Fill in your own WiFi network's name and password.
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

bool wasConnected = false;
uint32_t lastCheckMs = 0;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "BAĞLAN" -> "baglan"
// Lower-cases and simplifies Turkish letters: "BAĞLAN" -> "baglan"
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
void printStatus() {
  if (WiFi.status() == WL_CONNECTED) {
    minibot.serialWrite(String(L("Bağlı! Ağ: ", "Connected! Network: ")) + WIFI_SSID);
    minibot.serialWrite(String(L("  IP adresi : ", "  IP address: ")) + minibot.wifiGetIPAddress());
    minibot.serialWrite(String(L("  Sinyal    : ", "  Signal    : ")) + WiFi.RSSI() + L(" dBm (0'a yakın = daha güçlü)", " dBm (closer to 0 = stronger)"));
  } else {
    minibot.serialWrite(L("Bağlı DEĞİL. SSID/şifreyi kontrol edin ya da \"baglan\" yazın.", "NOT connected. Check SSID/password or type \"connect\"."));
  }
  minibot.serialWrite(String(L("  MAC adresi: ", "  MAC address: ")) + minibot.wifiGetMACAddress());
}

void printHelp() {
  minibot.serialWrite(L("---- WiFi DURUMU - Komutlar ----", "---- WiFi STATUS - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  durum         : bağlantı bilgileri", "  status        : connection info"));
  minibot.serialWrite(L("  baglan        : yeniden bağlan", "  connect       : connect again"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "durum" || cmd == "status") {
    printStatus();
  } else if (cmd == "baglan" || cmd == "connect") {
    minibot.serialWrite(L("Yeniden bağlanılıyor (arka planda)...", "Reconnecting (in the background)..."));
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS); // Beklemeden başlatır; sonucu loop() yazar / starts without waiting; loop() prints the result
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
  minibot.serialWrite(L("WiFi'ye bağlanılıyor (15 sn'ye kadar sürebilir)...", "Connecting to WiFi (may take up to 15 s)..."));

  minibot.wifiStartAndConnect(WIFI_SSID, WIFI_PASS); // En fazla ~15 sn dener / tries for up to ~15 s

  wasConnected = minibot.wifiConnectionControl();
  if (wasConnected) {
    minibot.serialWrite(String(L("Bağlandı! IP adresi: ", "Connected! IP address: ")) + minibot.wifiGetIPAddress());
  } else {
    minibot.serialWrite(L("Bağlantı başarısız! SSID/şifreyi kontrol edin.", "Connection failed! Check SSID/password."));
  }
  minibot.ledWrite(wasConnected); // Bağlıyken LED yanık kalır / LED stays on while connected
  printHelp();
}

void loop() {
  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) Saniyede bir bağlantıyı kontrol et; değişince yaz.
  // 2) Check the connection every second; print when it changes.
  if (millis() - lastCheckMs >= 1000) {
    lastCheckMs = millis();
    bool connected = (WiFi.status() == WL_CONNECTED);
    if (connected != wasConnected) {
      wasConnected = connected;
      minibot.ledWrite(connected);
      if (connected) minibot.serialWrite(String(L("Bağlandı! IP adresi: ", "Connected! IP address: ")) + minibot.wifiGetIPAddress());
      else minibot.serialWrite(L("WiFi bağlantısı KOPTU.", "WiFi connection LOST."));
    }
  }
}
