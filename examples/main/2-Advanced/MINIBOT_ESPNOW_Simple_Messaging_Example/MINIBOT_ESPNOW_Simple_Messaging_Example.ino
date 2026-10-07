/*
 * TR: ÇOCUKLAR İÇİN BASİT KABLOSUZ MESAJLAŞMA
 *  - Bu örnek her 2 saniyede bir artan bir sayı ("sayac") ve kısa bir metin mesajı
 *    yayınlar (broadcast). Aynı anda başka bir CODLAI kartından (IOTBOT/MINIBOT/
 *    ROLEBOT) gelen metin ve sayı mesajlarını da seri porta yazar. İki kart arasında
 *    (router/WiFi ağı OLMADAN) "sohbet" etmenin en basit yolu budur.
 *  - Seri porttan kendi mesajınızı da yazıp gönderebilirsiniz!
 *  - Kullanılan fonksiyonlar: espNowBegin, espNowSendText, espNowSendNumber,
 *    espNowAvailable, espNowReadText, espNowReadName, espNowReadNumber.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim       / help          -> komut listesi
 *      mesaj Selam  / msg Hello     -> yazdığınız metni gönder (en fazla 31 harf)
 *      sayi 42      / number 42     -> "sayi" adıyla bir sayı gönder
 *      dur          / pause         -> otomatik gönderimi duraklat (dinleme sürer)
 *      devam        / resume        -> otomatik gönderime devam et
 *      dil          / lang          -> dili değiştir (Türkçe <-> English)
 *
 * EN: SIMPLE WIRELESS MESSAGING FOR KIDS
 *  - This example broadcasts an increasing number ("sayac") and a short text message
 *    every 2 seconds. At the same time it prints the text and number messages that
 *    another CODLAI board (IOTBOT/MINIBOT/ROLEBOT) sends. This is the simplest way for
 *    two boards to "chat" (with NO router/WiFi network needed).
 *  - You can also type your own message on the serial port and send it!
 *  - Functions used: espNowBegin, espNowSendText, espNowSendNumber, espNowAvailable,
 *    espNowReadText, espNowReadName, espNowReadNumber.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help        / yardim         -> command list
 *      msg Hello   / mesaj Selam    -> send the text you typed (max 31 letters)
 *      number 42   / sayi 42        -> send a number named "sayi"
 *      pause       / dur            -> pause automatic sending (listening continues)
 *      resume      / devam          -> resume automatic sending
 *      lang        / dil            -> switch language (Turkish <-> English)
 */

#define USE_ESPNOW
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

uint32_t lastSendMs = 0;
int counter = 0;
bool autoSend = true;       // Otomatik gönderim açık mı / automatic sending on?
uint32_t ledOffAtMs = 0;

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

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "SAYI" -> "sayi"
// Lower-cases and simplifies Turkish letters: "SAYI" -> "sayi"
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
// Mesajlar ve komutlar / Messages and commands
// ---------------------------------------------------------------------------
void printHelp() {
  minibot.serialWrite(L("---- ESP-NOW MESAJLAŞMA - Komutlar ----", "---- ESP-NOW MESSAGING - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  mesaj <metin> : metni gönder", "  msg <text>    : send the text"));
  minibot.serialWrite(L("  sayi 42       : sayı gönder", "  number 42     : send a number"));
  minibot.serialWrite(L("  dur / devam   : otomatik gönderimi duraklat / sürdür", "  pause / resume: pause / resume automatic sending"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if ((word == "mesaj" || word == "msg" || word == "message") && hasValue) {
    // Metni orijinal haliyle al (büyük/küçük harf ve Türkçe harfler korunur).
    // Take the text as typed (case and Turkish letters are kept).
    String text = cmdRaw.substring(cmdRaw.indexOf(' ') + 1);
    text.trim();
    minibot.espNowSendText(text); // 31 bayttan uzunsa kesilir / cut if longer than 31 bytes
    minibot.serialWrite(String(L("Gönderildi -> metin: ", "Sent -> text: ")) + text);
  } else if ((word == "sayi" || word == "number") && hasValue) {
    float value = cmd.substring(space + 1).toFloat();
    minibot.espNowSendNumber("sayi", value);
    minibot.serialWrite(String(L("Gönderildi -> sayi = ", "Sent -> sayi = ")) + String(value, 2));
  } else if (word == "dur" || word == "pause" || word == "stop") {
    autoSend = false;
    minibot.serialWrite(L("Otomatik gönderim duraklatıldı (dinleme sürüyor).", "Automatic sending paused (still listening)."));
  } else if (word == "devam" || word == "resume") {
    autoSend = true;
    minibot.serialWrite(L("Otomatik gönderim sürüyor.", "Automatic sending resumed."));
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
  minibot.espNowBegin(1);      // Kanal 1 - dinleyen diğer kartla AYNI kanal olmalı / channel 1 - must match the other board
  minibot.ledWrite(false);
  minibot.serialWrite(L("Basit ESP-NOW mesajlaşma hazır.", "Simple ESP-NOW messaging ready."));
  printHelp();
}

void loop() {
  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) Her 2 saniyede bir mesaj gönder / send a message every 2 seconds
  if (autoSend && millis() - lastSendMs >= 2000) {
    lastSendMs = millis();
    counter++;
    minibot.espNowSendText(L("Merhaba!", "Hello!"));
    minibot.espNowSendNumber("sayac", counter);
    minibot.serialWrite(String(L("Gönderildi -> sayac: ", "Sent -> counter: ")) + counter);
  }

  // 3) Gelen mesaj var mı? Metinse metni, sayıysa adını ve değerini yaz.
  // 3) Any incoming message? Print the text, or the name and value of a number.
  if (minibot.espNowAvailable()) {
    if (minibot.receivedData.deviceType == 20) {
      String text = minibot.espNowReadText();
      if (text.length() > 0) minibot.serialWrite(String(L("Metin alındı: ", "Text received: ")) + text);
    } else {
      float value = minibot.espNowReadNumber(); // Sayı mesajı / number message
      String name = minibot.espNowReadName();
      minibot.serialWrite(String(L("Sayı alındı: ", "Number received: ")) + name + " = " + String(value, 2));
    }
    minibot.ledWrite(true);      // Kısa LED flaşı / short LED flash
    ledOffAtMs = millis() + 50;
  }

  // 4) Mavi LED'i söndür / turn the blue LED off
  if (ledOffAtMs != 0 && (int32_t)(millis() - ledOffAtMs) >= 0) {
    ledOffAtMs = 0;
    minibot.ledWrite(false);
  }
}
