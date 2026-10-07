/*
 * TR: E-POSTA GÖNDERİCİ ÖRNEĞİ
 *  - MINIBOT WiFi'ye bağlanır ve açılışta bir deneme e-postası gönderir. Sonra B1
 *    butonuna her bastığınızda (ya da seri porttan "gonder" yazdığınızda) yeni bir
 *    e-posta gönderir. Gönderim sırasında mavi LED yanar.
 *  - Gmail kullanıyorsanız normal şifreniz ÇALIŞMAZ: Google Hesabı > Güvenlik >
 *    "Uygulama şifreleri" bölümünden 16 haneli bir uygulama şifresi alın.
 *  - Gönderim birkaç saniye sürer; o sırada kart başka işe cevap vermez (normaldir).
 *  - B1 bu örnekte "e-posta gönder" butonudur.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help     -> komut listesi
 *      gonder / send     -> e-posta gönder
 *      durum  / status   -> WiFi durumu ve gönderilen e-posta sayısı
 *      dil    / lang     -> dili değiştir (Türkçe <-> English)
 *
 * EN: EMAIL SENDER EXAMPLE
 *  - MINIBOT joins WiFi and sends a test email at startup. After that, every press of
 *    the B1 button (or typing "send" on the serial port) sends a new email. The blue
 *    LED is on while sending.
 *  - If you use Gmail your normal password WON'T work: get a 16-character app password
 *    from Google Account > Security > "App passwords".
 *  - Sending takes a few seconds; the board does not respond to anything else meanwhile
 *    (that's normal).
 *  - In this example B1 is the "send email" button.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim   -> command list
 *      send   / gonder   -> send an email
 *      status / durum    -> WiFi state and number of emails sent
 *      lang   / dil      -> switch language (Turkish <-> English)
 */

#define USE_EMAIL
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

// WiFi bilgileri / WiFi credentials
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// E-posta bilgileri / Email credentials
#define SMTP_HOST "smtp.gmail.com"
#define SMTP_PORT 465
#define AUTHOR_EMAIL "YOUR_EMAIL@gmail.com"
#define AUTHOR_PASSWORD "YOUR_APP_PASSWORD"
#define RECIPIENT_EMAIL "RECIPIENT_EMAIL@example.com"

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

uint32_t emailCount = 0;
uint32_t lastSendMs = 0;
const uint32_t kMinGapMs = 10000; // İki e-posta arası en az 10 sn (yanlışlıkla çoklu gönderim olmasın)
                                  // at least 10 s between emails (no accidental floods)

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
// E-posta ve mesajlar / Email and messages
// ---------------------------------------------------------------------------
void sendEmailNow() {
  if (WiFi.status() != WL_CONNECTED) {
    minibot.serialWrite(L("WiFi bağlı değil - e-posta gönderilemez.", "WiFi not connected - can't send email."));
    return;
  }
  if (emailCount > 0 && millis() - lastSendMs < kMinGapMs) {
    minibot.serialWrite(L("Biraz bekleyin (iki e-posta arası en az 10 sn).", "Please wait (at least 10 s between emails)."));
    return;
  }
  lastSendMs = millis();
  emailCount++;
  minibot.ledWrite(true);
  minibot.serialWrite(L("E-posta gönderiliyor...", "Sending email..."));
  String subject = String(L("MINIBOT test e-postası #", "MINIBOT test email #")) + emailCount;
  String body = String(L("Merhaba, bu e-posta MINIBOT'tan gönderildi! Kart ", "Hello, this email was sent from MINIBOT! The board has been on for ")) +
                (millis() / 1000) + L(" saniyedir açık.", " seconds.");
  minibot.sendEmail(SMTP_HOST, SMTP_PORT, AUTHOR_EMAIL, AUTHOR_PASSWORD, RECIPIENT_EMAIL, subject, body);
  minibot.ledWrite(false);
}

void printHelp() {
  minibot.serialWrite(L("---- E-POSTA - Komutlar ----", "---- EMAIL - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  gonder        : e-posta gönder", "  send          : send an email"));
  minibot.serialWrite(L("  durum         : WiFi ve sayaç", "  status        : WiFi and counter"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu     : e-posta gönder", "  B1 button     : send an email"));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "gonder" || cmd == "send") {
    sendEmailNow();
  } else if (cmd == "durum" || cmd == "status") {
    minibot.serialWrite(String(WiFi.status() == WL_CONNECTED ? L("WiFi: bağlı", "WiFi: connected") : L("WiFi: bağlı DEĞİL", "WiFi: NOT connected")) +
                        L("  |  gönderilen e-posta: ", "  |  emails sent: ") + emailCount);
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
  minibot.serialWrite(L("E-posta Gönderici Örneği", "Email Sender Example"));

  minibot.wifiStartAndConnect(WIFI_SSID, WIFI_PASSWORD);
  if (minibot.wifiConnectionControl()) {
    sendEmailNow(); // Açılışta bir deneme e-postası / one test email at startup
  }
  printHelp();
}

void loop() {
  // 1) B1 -> e-posta gönder / B1 -> send an email
  if (b1Pressed()) sendEmailNow();

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);
}
