/*
 * TR: ERİŞİM NOKTASI (AP) MODUNDA YEREL WEB SUNUCU
 *  - MINIBOT kendi WiFi ağını kurar ("CODLAI Server", şifre "12345678"). Telefonunuzu
 *    ya da bilgisayarınızı bu ağa bağlayıp tarayıcıda http://192.168.4.1/demopage
 *    adresini açın: MINIBOT'un sunduğu web sayfasını görürsünüz. Router/İnternet GEREKMEZ.
 *  - Sayfa HTML (içerik), CSS (görünüm) ve JavaScript (davranış) parçalarından oluşur.
 *  - Bir cihaz ağa bağlanınca / ayrılınca seri porta yazılır.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help     -> komut listesi
 *      durum  / status   -> ağ adı, IP adresi ve bağlı cihaz sayısı
 *      dil    / lang     -> dili değiştir (Türkçe <-> English)
 *
 * EN: LOCAL WEB SERVER IN ACCESS POINT (AP) MODE
 *  - MINIBOT creates its own WiFi network ("CODLAI Server", password "12345678").
 *    Connect your phone or computer to it and open http://192.168.4.1/demopage in a
 *    browser: you see the web page served by MINIBOT. NO router/Internet needed.
 *  - The page is built from HTML (content), CSS (look) and JavaScript (behavior) parts.
 *  - When a device joins / leaves the network it is printed to Serial.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim   -> command list
 *      status / durum    -> network name, IP address and number of connected devices
 *      lang   / dil      -> switch language (Turkish <-> English)
 *
 * NOT / NOTE: "#define USE_SERVER" satırı #include'dan ÖNCE yazılmalıdır.
 *             The "#define USE_SERVER" line must come BEFORE the #include.
 */

#define USE_SERVER
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

// Erişim Noktası (AP) modu için WiFi bilgileri / WiFi details for Access Point (AP) mode
#define AP_SSID "CODLAI Server" // AP modu ağ adı / AP mode network name
#define AP_PASS "12345678"      // AP modu şifresi (en az 8 karakter) / AP mode password (at least 8 characters)

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// ---------------------------------------------------------------------------
// Web sayfası (HTML, CSS, JavaScript) / Web page (HTML, CSS, JavaScript)
// Sayfa bir kez kaydedildiği için metinleri iki dilde yazdık.
// The page is registered once, so its texts are written in both languages.
// ---------------------------------------------------------------------------
// JavaScript: butona tıklanınca bir mesaj gösterir / shows a message when the button is clicked
const char WEBPageScript[] PROGMEM = R"rawliteral(
<script>
  function sayHello() {
    alert("Merhaba MINIBOT! / Hello MINIBOT!");
  }
</script>
)rawliteral";

// CSS: sayfanın görünümü / the page's look
const char WEBPageCSS[] PROGMEM = R"rawliteral(
<style>
  body { text-align: center; font-family: Arial, sans-serif; }
  button { font-size: 20px; padding: 10px; margin: 20px; }
</style>
)rawliteral";

// HTML: sayfanın içeriği. İlk %s yerine JavaScript, ikinci %s yerine CSS konur.
// HTML: the page content. The first %s is replaced by the JavaScript, the second by the CSS.
const char WEBPageHTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="tr">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>MINIBOT Web Server</title>
  %s <!-- JavaScript -->
  %s <!-- CSS -->
</head>
<body>
  <h1>MINIBOT Web Sayfası / Web Page</h1>
  <button onclick="sayHello()">Tıklayın / Click</button>
</body>
</html>
)rawliteral";

int lastStations = -1; // Son bağlı cihaz sayısı / last number of connected devices

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
void printStatus() {
  minibot.serialWrite(String(L("Ağ adı (SSID): ", "Network (SSID): ")) + AP_SSID + L("   Şifre: ", "   Password: ") + AP_PASS);
  minibot.serialWrite(String(L("Sayfa adresi: http://", "Page address: http://")) + WiFi.softAPIP().toString() + "/demopage");
  minibot.serialWrite(String(L("Bağlı cihaz sayısı: ", "Connected devices: ")) + WiFi.softAPgetStationNum());
}

void printHelp() {
  minibot.serialWrite(L("---- AP WEB SUNUCU - Komutlar ----", "---- AP WEB SERVER - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  durum         : ağ, IP, bağlı cihazlar", "  status        : network, IP, connected devices"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
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

  // MINIBOT'u erişim noktası (AP) olarak başlat / start MINIBOT as an access point (AP)
  minibot.serverStart("AP", AP_SSID, AP_PASS);

  // Web sayfasını yayınla: http://192.168.4.1/demopage / publish the web page
  minibot.serverCreateLocalPage("demopage", WEBPageScript, WEBPageCSS, WEBPageHTML);

  minibot.serialWrite(L("Web sunucu hazır. Telefonunuzu bu ağa bağlayın:", "Web server ready. Connect your phone to this network:"));
  printStatus();
  printHelp();
}

void loop() {
  minibot.serverContinue(); // AP modunda DNS yönlendirmeyi sürdür / keep DNS redirection running in AP mode

  // Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // Bağlı cihaz sayısı değişince yaz / print when the number of connected devices changes
  int stations = WiFi.softAPgetStationNum();
  if (stations != lastStations) {
    if (lastStations >= 0) {
      minibot.serialWrite(String(stations > lastStations ? L("Bir cihaz bağlandı. ", "A device connected. ")
                                                         : L("Bir cihaz ayrıldı. ", "A device left. ")) +
                          L("Bağlı cihaz: ", "Connected: ") + stations);
    }
    lastStations = stations;
  }
}
