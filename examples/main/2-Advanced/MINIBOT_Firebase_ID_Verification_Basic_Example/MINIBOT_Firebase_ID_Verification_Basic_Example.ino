/*
 * TR: FIREBASE KULLANICI DOĞRULAMA VE VERİ OKU/YAZ
 *  - MINIBOT WiFi'ye bağlanır, Firebase'e e-posta/şifre ile giriş yapar (kimliği
 *    doğrulanır) ve gerçek zamanlı veritabanına (Realtime Database) örnek veriler
 *    yazar: /device/temperature, /device/status, /device/active.
 *  - Sonra bu verileri belirli aralıklarla (varsayılan 30 sn) geri okuyup seri porta
 *    yazar. Firebase konsolunda değerleri değiştirirseniz bir sonraki okumada görürsünüz.
 *  - WiFi açılışta bağlanamazsa kart DONMAZ: arka planda bağlanmayı bekler, bağlanınca
 *    Firebase'i başlatır.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim      / help          -> komut listesi
 *      oku         / read          -> verileri hemen oku
 *      sicaklik 25 / temp 25       -> /device/temperature değerini yaz
 *      aktif       / active        -> /device/active değerini tersine çevir
 *      aralik 30   / interval 30   -> okuma aralığı (saniye, 5-600)
 *      dil         / lang          -> dili değiştir (Türkçe <-> English)
 *
 * EN: FIREBASE USER VERIFICATION AND DATA READ/WRITE
 *  - MINIBOT joins WiFi, signs in to Firebase with email/password (its identity is
 *    verified) and writes sample data to the Realtime Database: /device/temperature,
 *    /device/status, /device/active.
 *  - Then it reads the data back at intervals (30 s by default) and prints it. If you
 *    change the values in the Firebase console you see them at the next read.
 *  - If WiFi fails at startup the board does NOT freeze: it waits for the connection in
 *    the background and starts Firebase once connected.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help        / yardim        -> command list
 *      read        / oku           -> read the data now
 *      temp 25     / sicaklik 25   -> write /device/temperature
 *      active      / aktif         -> invert /device/active
 *      interval 30 / aralik 30     -> read interval (seconds, 5-600)
 *      lang        / dil           -> switch language (Turkish <-> English)
 *
 * KURULUM / SETUP: Firebase konsolunda bir proje açın, Authentication'da
 * "E-posta/Şifre" ile bir kullanıcı oluşturun, Realtime Database'i açın ve aşağıdaki
 * bilgileri kendi projenize göre doldurun. / Create a project in the Firebase console,
 * add an "Email/Password" user under Authentication, enable the Realtime Database and
 * fill in the values below for your own project.
 *
 * NOT / NOTE: "#define USE_FIREBASE" satırı #include'dan ÖNCE yazılmalıdır.
 *             The "#define USE_FIREBASE" line must come BEFORE the #include.
 */

#define USE_FIREBASE
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

// Firebase yapılandırması / Firebase configuration
#define FIREBASE_PROJECT_URL "https://YOUR_PROJECT_ID-default-rtdb.firebaseio.com/" // Firebase veritabanı adresi / database URL
#define FIREBASE_API_KEY "YOUR_FIREBASE_API_KEY"                                    // Proje ayarlarındaki Web API anahtarı / Web API key from project settings

// Firebase kullanıcı doğrulama / Firebase user authentication
#define USER_EMAIL "YOUR_USER_EMAIL@example.com" // Firebase'de oluşturduğunuz kullanıcının e-postası / email of the Firebase user
#define USER_PASSWORD "YOUR_USER_PASSWORD"       // O kullanıcının şifresi / that user's password

// WiFi ayarları / WiFi settings
#define WIFI_SSID "YOUR_WIFI_SSID"     // Bağlanılacak WiFi ağının adı / WiFi network name
#define WIFI_PASS "YOUR_WIFI_PASSWORD" // WiFi şifresi / WiFi password

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

bool firebaseStarted = false;   // Firebase başlatıldı mı / Firebase started?
bool activeValue = true;        // /device/active için yazdığımız değer / value we write to /device/active
uint32_t readIntervalMs = 30000;
uint32_t lastReadMs = 0;
uint32_t lastWifiCheckMs = 0;
bool waitingMessageShown = false;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "SICAKLIK" -> "sicaklik"
// Lower-cases and simplifies Turkish letters: "SICAKLIK" -> "sicaklik"
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
// Firebase
// ---------------------------------------------------------------------------
void startFirebase() {
  // Giriş yap ve kimliği doğrula (25 sn'ye kadar sürebilir) / sign in and verify (may take up to 25 s)
  minibot.serialWrite(L("Firebase'e giriş yapılıyor...", "Signing in to Firebase..."));
  minibot.fbServerSetandStartWithUser(FIREBASE_PROJECT_URL, FIREBASE_API_KEY, USER_EMAIL, USER_PASSWORD);
  firebaseStarted = true;

  // Örnek verileri yaz / write sample data
  minibot.fbServerSetInt("/device/temperature", 25);
  minibot.fbServerSetString("/device/status", "Online");
  minibot.fbServerSetBool("/device/active", activeValue);
  minibot.serialWrite(L("Veriler Firebase'e gönderildi.", "Data sent to Firebase."));
}

void readFirebase() {
  int temp = minibot.fbServerGetInt("/device/temperature");
  String status = minibot.fbServerGetString("/device/status");
  bool active = minibot.fbServerGetBool("/device/active");

  minibot.serialWrite(String(L("Sıcaklık: ", "Temperature: ")) + temp);
  minibot.serialWrite(String(L("Durum   : ", "Status     : ")) + status);
  minibot.serialWrite(String(L("Aktif   : ", "Active     : ")) + (active ? L("Evet", "Yes") : L("Hayır", "No")));
}

// ---------------------------------------------------------------------------
// Mesajlar ve komutlar / Messages and commands
// ---------------------------------------------------------------------------
void printHelp() {
  minibot.serialWrite(L("---- FIREBASE - Komutlar ----", "---- FIREBASE - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  oku           : verileri hemen oku", "  read          : read the data now"));
  minibot.serialWrite(L("  sicaklik 25   : sıcaklık değerini yaz", "  temp 25       : write the temperature value"));
  minibot.serialWrite(L("  aktif         : active değerini çevir", "  active        : invert the active value"));
  minibot.serialWrite(L("  aralik 5-600  : okuma aralığı (saniye)", "  interval 5-600: read interval (seconds)"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    minibot.serialWrite(L("Dil: Türkçe", "Language: English"));
    printHelp();
  } else if ((word == "aralik" || word == "interval") && hasValue) {
    readIntervalMs = (uint32_t)constrain(value, 5, 600) * 1000;
    minibot.serialWrite(String(L("Okuma aralığı: ", "Read interval: ")) + (readIntervalMs / 1000) + L(" saniye", " seconds"));
  } else if (!firebaseStarted && (word == "oku" || word == "read" || word == "sicaklik" || word == "temp" || word == "aktif" || word == "active")) {
    minibot.serialWrite(L("Firebase henüz hazır değil (WiFi bekleniyor).", "Firebase not ready yet (waiting for WiFi)."));
  } else if (word == "oku" || word == "read") {
    readFirebase();
    lastReadMs = millis();
  } else if ((word == "sicaklik" || word == "temp") && hasValue) {
    minibot.fbServerSetInt("/device/temperature", value);
  } else if (word == "aktif" || word == "active") {
    activeValue = !activeValue;
    minibot.fbServerSetBool("/device/active", activeValue);
  } else {
    minibot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  minibot.begin();             // MINIBOT başlatılıyor / Initialize MINIBOT
  minibot.serialStart(115200); // Seri haberleşme / Serial communication
  minibot.serialWrite(L("MiniBot Firebase örneği başlıyor...", "MiniBot Firebase example starting..."));

  // 1) WiFi'ye bağlan (~15 sn'ye kadar dener) / connect to WiFi (tries for up to ~15 s)
  minibot.wifiStartAndConnect(WIFI_SSID, WIFI_PASS);

  // 2) Bağlandıysa Firebase'i başlat; bağlanamadıysa loop() beklemeye devam eder.
  // 2) If connected, start Firebase; otherwise loop() keeps waiting.
  if (WiFi.status() == WL_CONNECTED) startFirebase();
  printHelp();
}

void loop() {
  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) WiFi henüz yoksa saniyede bir kontrol et (kart donmaz).
  // 2) If there is no WiFi yet, check once per second (the board does not freeze).
  if (!firebaseStarted) {
    if (millis() - lastWifiCheckMs >= 1000) {
      lastWifiCheckMs = millis();
      if (WiFi.status() == WL_CONNECTED) {
        minibot.serialWrite(L("WiFi bağlandı!", "WiFi connected!"));
        startFirebase();
      } else if (!waitingMessageShown) {
        waitingMessageShown = true;
        minibot.serialWrite(L("WiFi yok - arka planda bağlanmayı bekliyorum (SSID/şifreyi kontrol edin).",
                              "No WiFi - waiting to connect in the background (check SSID/password)."));
      }
    }
    return;
  }

  // 3) Aralık dolunca Firebase'den oku (delay yok) / read Firebase when the interval is up (no delay)
  if (millis() - lastReadMs >= readIntervalMs) {
    lastReadMs = millis();
    readFirebase();
  }
}
