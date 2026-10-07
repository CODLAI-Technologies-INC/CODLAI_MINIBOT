/*
 * TR: İSTASYON (STA) MODUNDA YEREL WEB SUNUCU
 *  - MINIBOT evinizin/okulunuzun WiFi ağına bağlanır ve o ağdaki herkesin açabileceği
 *    bir web sayfası sunar. Seri portta yazan adresi (ör. http://192.168.1.45/demopage)
 *    aynı ağdaki bir telefonda/bilgisayarda açın.
 *  - Ağa bağlanamazsa kendi ağını (AP modu, "CODLAI Server" / "12345678") kurar;
 *    o zaman adres http://192.168.4.1/demopage olur.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help     -> komut listesi
 *      durum  / status   -> mod (STA/AP), IP adresi, sinyal gücü
 *      dil    / lang     -> dili değiştir (Türkçe <-> English)
 *
 * EN: LOCAL WEB SERVER IN STATION (STA) MODE
 *  - MINIBOT joins your home/school WiFi network and serves a web page anyone on that
 *    network can open. Open the address printed on Serial (e.g.
 *    http://192.168.1.45/demopage) on a phone/computer on the same network.
 *  - If it can't join the network it creates its own (AP mode, "CODLAI Server" /
 *    "12345678"); then the address is http://192.168.4.1/demopage.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim   -> command list
 *      status / durum    -> mode (STA/AP), IP address, signal strength
 *      lang   / dil      -> switch language (Turkish <-> English)
 *
 * NOT / NOTE: "#define USE_SERVER" satırı #include'dan ÖNCE yazılmalıdır. WiFi'ye
 * bağlanmak 30 saniyeye kadar sürebilir. / The "#define USE_SERVER" line must come
 * BEFORE the #include. Connecting to WiFi can take up to 30 seconds.
 */

#define USE_SERVER
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

// WiFi ayarları: bağlanmak istediğiniz ağın adını ve şifresini yazın.
// WiFi settings: enter the name and password of the network you want to join.
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

// Bağlanamazsa kurulacak erişim noktası (AP) / access point (AP) used if joining fails
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

bool apMode = false; // true = kendi ağını kurdu / created its own network

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
  if (apMode) {
    minibot.serialWrite(String(L("Mod: AP (kendi ağı) - ağ adı: ", "Mode: AP (own network) - network: ")) + AP_SSID);
    minibot.serialWrite(String(L("Sayfa adresi: http://", "Page address: http://")) + WiFi.softAPIP().toString() + "/demopage");
  } else {
    minibot.serialWrite(String(L("Mod: STA (ağa bağlı) - ağ: ", "Mode: STA (joined) - network: ")) + WIFI_SSID +
                        L("  sinyal: ", "  signal: ") + WiFi.RSSI() + " dBm");
    minibot.serialWrite(String(L("Sayfa adresi: http://", "Page address: http://")) + WiFi.localIP().toString() + "/demopage");
  }
}

void printHelp() {
  minibot.serialWrite(L("---- STA WEB SUNUCU - Komutlar ----", "---- STA WEB SERVER - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  durum         : mod, IP, sinyal", "  status        : mode, IP, signal"));
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
  minibot.serialWrite(L("WiFi'ye bağlanılıyor (30 sn'ye kadar sürebilir)...", "Connecting to WiFi (may take up to 30 s)..."));

  // STA modunda bağlan / connect in STA mode
  minibot.serverStart("STA", WIFI_SSID, WIFI_PASS);

  // STA bağlantısı başarısız olursa kendi AP ağımızı kur.
  // NOT: serverStart("STA") başarısız olursa kendisi de "CODLAI-MINIBOT" adlı bir AP
  // kurar; bu satır AP'yi bizim AP_SSID / AP_PASS bilgilerimizle yeniden kurar
  // (sunucu sayfaları iki kez eklenmez).
  // If the STA connection fails, start our own AP network.
  // NOTE: serverStart("STA") also falls back to an AP named "CODLAI-MINIBOT" on its
  // own; this line re-creates the AP with our AP_SSID / AP_PASS (the server pages are
  // not added twice).
  if (!minibot.wifiConnectionControl()) {
    apMode = true;
    minibot.serverStart("AP", AP_SSID, AP_PASS);
  }

  // Web sayfasını yayınla / publish the web page
  minibot.serverCreateLocalPage("demopage", WEBPageScript, WEBPageCSS, WEBPageHTML);

  printStatus();
  printHelp();
}

void loop() {
  minibot.serverContinue(); // AP modundaysa DNS yönlendirmeyi sürdür / keep DNS redirection running in AP mode

  // Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);
}
