/*
 * TR: HAVA DURUMU BİLGİSİ
 *  - MINIBOT WiFi'ye bağlanır ve seçtiğiniz şehrin hava durumunu (sıcaklık + durum)
 *    internetten alıp her dakika seri porta yazar. Gelen metin servisin dilindedir
 *    (genelde İngilizce, ör. "+18°C Partly cloudy").
 *  - API_KEY boş bırakılırsa ücretsiz wttr.in servisi kullanılır; OpenWeatherMap
 *    anahtarınız varsa API_KEY'e yazabilirsiniz.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim       / help         -> komut listesi
 *      guncelle     / update       -> hava durumunu şimdi al
 *      sehir Ankara / city Ankara  -> şehri değiştir
 *      aralik 5     / interval 5   -> güncelleme aralığı (dakika, 1-60)
 *      dil          / lang         -> dili değiştir (Türkçe <-> English)
 *
 * EN: WEATHER INFO
 *  - MINIBOT joins WiFi, gets the weather (temperature + condition) of the city you
 *    choose from the internet and prints it every minute. The text comes in the
 *    service's language (usually English, e.g. "+18°C Partly cloudy").
 *  - If API_KEY is left empty the free wttr.in service is used; if you have an
 *    OpenWeatherMap key you can put it in API_KEY.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help        / yardim        -> command list
 *      update      / guncelle      -> get the weather now
 *      city Ankara / sehir Ankara  -> change the city
 *      interval 5  / aralik 5      -> update interval (minutes, 1-60)
 *      lang        / dil           -> switch language (Turkish <-> English)
 */

#define USE_WEATHER
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// OpenWeatherMap API anahtarı (isteğe bağlı; boşsa wttr.in kullanılır)
// OpenWeatherMap API Key (optional; leave empty to use wttr.in)
#define API_KEY ""

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

String city = "Istanbul";          // Şehir / city
uint32_t intervalMs = 60000;       // Güncelleme aralığı / update interval
uint32_t lastUpdateMs = 0;
bool firstUpdateDone = false;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// cmdRaw: komutun harfleri değiştirilmemiş hali (şehir adı için).
// cmdRaw: the command with its letters untouched (for the city name).
// ---------------------------------------------------------------------------
String cmdBuffer;
String cmdRaw;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "ŞEHİR" -> "sehir"
// Lower-cases and simplifies Turkish letters: "ŞEHİR" -> "sehir"
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
      cmdRaw = cmdBuffer; cmdRaw.trim();
      cmd = normalizeCommand(cmdBuffer);
      cmdBuffer = "";
      return true;
    }
    if (cmdBuffer.length() < 40) cmdBuffer += c;
  }
  // "Satır sonu yok" seçiliyse: 150 ms sessizlikten sonra komutu kabul et.
  // "No line ending" selected: accept the command after 150 ms of silence.
  if (cmdBuffer.length() > 0 && millis() - lastCharMs > 150) {
    cmdRaw = cmdBuffer; cmdRaw.trim();
    cmd = normalizeCommand(cmdBuffer);
    cmdBuffer = "";
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Hava durumu ve mesajlar / Weather and messages
// ---------------------------------------------------------------------------
// Şehir adını olduğu gibi verin: getWeather() boşluk ve Türkçe harfleri adres (URL)
// için kendisi kodlar ("İzmir", "New York"). / Pass the city name as it is: getWeather()
// encodes spaces and Turkish letters for the address (URL) by itself.
void updateWeather() {
  if (WiFi.status() != WL_CONNECTED) {
    minibot.serialWrite(L("WiFi bağlı değil - hava durumu alınamıyor.", "WiFi not connected - can't get the weather."));
    return;
  }
  minibot.serialWrite(String(L("Hava durumu alınıyor: ", "Getting the weather: ")) + city + " ...");
  String weather = minibot.getWeather(city, API_KEY); // Birkaç saniye sürebilir / may take a few seconds
  minibot.serialWrite(String(L("Hava durumu (", "Weather in ")) + city + L("): ", ": ") + weather);
}

void printHelp() {
  minibot.serialWrite(L("---- HAVA DURUMU - Komutlar ----", "---- WEATHER - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  guncelle      : şimdi al", "  update        : get it now"));
  minibot.serialWrite(L("  sehir <ad>    : şehri değiştir", "  city <name>   : change the city"));
  minibot.serialWrite(L("  aralik 1-60   : aralık (dakika)", "  interval 1-60 : interval (minutes)"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "guncelle" || word == "update") {
    updateWeather();
    lastUpdateMs = millis();
  } else if ((word == "sehir" || word == "city") && hasValue) {
    city = cmdRaw.substring(cmdRaw.indexOf(' ') + 1);
    city.trim();
    minibot.serialWrite(String(L("Şehir: ", "City: ")) + city);
    updateWeather();
    lastUpdateMs = millis();
  } else if ((word == "aralik" || word == "interval") && hasValue) {
    intervalMs = (uint32_t)constrain(cmd.substring(space + 1).toInt(), 1, 60) * 60000UL;
    minibot.serialWrite(String(L("Güncelleme aralığı: ", "Update interval: ")) + (intervalMs / 60000) + L(" dakika", " minutes"));
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
  minibot.serialWrite(L("Hava Durumu Örneği", "Weather Info Example"));
  minibot.wifiStartAndConnect(WIFI_SSID, WIFI_PASSWORD);
  printHelp();
}

void loop() {
  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) İlk seferde hemen, sonra her aralıkta güncelle (delay yok).
  // 2) Update right away the first time, then every interval (no delay).
  if (!firstUpdateDone || millis() - lastUpdateMs >= intervalMs) {
    if (WiFi.status() == WL_CONNECTED) {
      firstUpdateDone = true;
      lastUpdateMs = millis();
      updateWeather();
    } else if (firstUpdateDone || millis() > 20000) {
      // WiFi yoksa bir dakika sonra tekrar dene / without WiFi, try again in a minute
      firstUpdateDone = true;
      lastUpdateMs = millis();
      minibot.serialWrite(L("WiFi bağlı değil - SSID/şifreyi kontrol edin.", "WiFi not connected - check SSID/password."));
    }
  }
}
