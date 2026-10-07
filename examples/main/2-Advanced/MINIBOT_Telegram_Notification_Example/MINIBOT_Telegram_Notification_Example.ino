/*
 * TR: TELEGRAM BİLDİRİM ÖRNEĞİ
 *  - MINIBOT WiFi'ye bağlanır. B1 butonuna basınca Telegram botunuz size
 *    "MINIBOT: Butona basıldı!" mesajı gönderir. Seri porttan kendi mesajınızı da
 *    yazıp gönderebilirsiniz. Gönderim sırasında mavi LED yanar.
 *  - Kurulum: Telegram'da @BotFather ile bir bot oluşturup TOKEN'ı alın; @userinfobot
 *    ile kendi CHAT ID'nizi öğrenin; sonra botunuza bir kez "merhaba" yazın.
 *  - B1 bu örnekte "bildirim gönder" butonudur (iki mesaj arası en az 5 sn).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim       / help        -> komut listesi
 *      gonder       / send        -> hazır mesajı gönder (B1 gibi)
 *      mesaj Selam! / msg Hi!     -> yazdığınız mesajı gönder
 *      durum        / status      -> WiFi durumu ve gönderilen mesaj sayısı
 *      dil          / lang        -> dili değiştir (Türkçe <-> English)
 *
 * EN: TELEGRAM NOTIFICATION EXAMPLE
 *  - MINIBOT joins WiFi. When you press B1, your Telegram bot sends you the message
 *    "MINIBOT: Button pressed!". You can also type your own message on the serial port
 *    and send it. The blue LED is on while sending.
 *  - Setup: create a bot with @BotFather in Telegram and get its TOKEN; find your own
 *    CHAT ID with @userinfobot; then write "hello" to your bot once.
 *  - In this example B1 is the "send notification" button (at least 5 s between messages).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help        / yardim       -> command list
 *      send        / gonder       -> send the ready message (like B1)
 *      msg Hi!     / mesaj Selam! -> send the message you typed
 *      status      / durum        -> WiFi state and number of messages sent
 *      lang        / dil          -> switch language (Turkish <-> English)
 */

#define USE_TELEGRAM
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// Telegram bot token'ı (@BotFather'dan) / Telegram Bot Token (from @BotFather)
#define BOT_TOKEN "YOUR_BOT_TOKEN"
// Sizin chat ID'niz (@userinfobot'tan) / your Chat ID (from @userinfobot)
#define CHAT_ID "YOUR_CHAT_ID"

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

uint32_t lastSendMs = 0;
const uint32_t kMinGapMs = 5000; // İki mesaj arası en az 5 sn / at least 5 s between messages
uint32_t sentCount = 0;

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
// cmdRaw: komutun harfleri değiştirilmemiş hali (mesaj metni için).
// cmdRaw: the command with its letters untouched (for the message text).
// ---------------------------------------------------------------------------
String cmdBuffer;
String cmdRaw;
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
// Telegram ve mesajlar / Telegram and messages
// ---------------------------------------------------------------------------
// Mesajı olduğu gibi verin: sendTelegram() boşluk, &, ? ve Türkçe harfleri adres (URL)
// için kendisi kodlar. / Pass the message as it is: sendTelegram() encodes spaces, &, ?
// and Turkish letters for the address (URL) by itself.
void sendMessage(const String &message) {
  if (WiFi.status() != WL_CONNECTED) {
    minibot.serialWrite(L("WiFi bağlı değil - mesaj gönderilemez.", "WiFi not connected - can't send the message."));
    return;
  }
  if (sentCount > 0 && millis() - lastSendMs < kMinGapMs) {
    minibot.serialWrite(L("Biraz bekleyin (iki mesaj arası en az 5 sn).", "Please wait (at least 5 s between messages)."));
    return;
  }
  lastSendMs = millis();
  sentCount++;
  minibot.ledWrite(true);
  minibot.serialWrite(String(L("Telegram mesajı gönderiliyor: ", "Sending Telegram message: ")) + message);
  minibot.sendTelegram(BOT_TOKEN, CHAT_ID, message); // Birkaç saniye sürebilir / may take a few seconds
  minibot.ledWrite(false);
}

void printHelp() {
  minibot.serialWrite(L("---- TELEGRAM - Komutlar ----", "---- TELEGRAM - Commands ----"));
  minibot.serialWrite(L("  yardim          : bu liste", "  help            : this list"));
  minibot.serialWrite(L("  gonder          : hazır mesajı gönder", "  send            : send the ready message"));
  minibot.serialWrite(L("  mesaj <metin>   : yazdığınızı gönder", "  msg <text>      : send what you typed"));
  minibot.serialWrite(L("  durum           : WiFi ve mesaj sayısı", "  status          : WiFi and message count"));
  minibot.serialWrite(L("  dil             : English'e geç", "  lang            : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu       : hazır mesajı gönder", "  B1 button       : send the ready message"));
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "gonder" || word == "send") {
    sendMessage(L("MINIBOT: Butona basıldı!", "MINIBOT: Button pressed!"));
  } else if ((word == "mesaj" || word == "msg" || word == "message") && hasValue) {
    String text = cmdRaw.substring(cmdRaw.indexOf(' ') + 1);
    text.trim();
    sendMessage(String("MINIBOT: ") + text);
  } else if (word == "durum" || word == "status") {
    minibot.serialWrite(String(WiFi.status() == WL_CONNECTED ? L("WiFi: bağlı", "WiFi: connected") : L("WiFi: bağlı DEĞİL", "WiFi: NOT connected")) +
                        L("  |  gönderilen mesaj: ", "  |  messages sent: ") + sentCount);
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
  minibot.ledWrite(false);
  minibot.serialWrite(L("Telegram Bildirim Örneği", "Telegram Notification Example"));
  minibot.wifiStartAndConnect(WIFI_SSID, WIFI_PASSWORD);
  printHelp();
}

void loop() {
  // 1) B1 -> bildirim gönder / B1 -> send a notification
  if (b1Pressed()) {
    minibot.serialWrite(L("Butona basıldı! Telegram mesajı gönderiliyor...", "Button pressed! Sending Telegram message..."));
    sendMessage(L("MINIBOT: Butona basıldı!", "MINIBOT: Button pressed!"));
  }

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);
}
