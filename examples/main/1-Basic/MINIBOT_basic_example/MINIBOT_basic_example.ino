/*
 * TR: MINIBOT TEMEL ÖRNEK - Buton, LED ve buzzer
 *  - Kart üzerindeki B1 butonuna basılı tuttuğunuz sürece mavi LED yanar,
 *    bırakınca söner. Her basışta kısa bir bip duyulur ve seri porta
 *    "Butona basıldı" yazılır (mesaj sadece durum DEĞİŞİNCE yazılır).
 *  - B1 butonu (GPIO0) basılıyken LOW okunur: button1Read() == false -> basılı.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help    -> komut listesi
 *      ac     / on      -> mavi LED'i yak (butona basılınca yine butonu izler)
 *      kapat  / off     -> mavi LED'i söndür
 *      bip    / beep    -> buzzer'dan kısa bir ses
 *      durum  / status  -> buton durumu ve kaç kez basıldığı
 *      dil    / lang    -> dili değiştir (Türkçe <-> English)
 *
 * EN: MINIBOT BASIC EXAMPLE - Button, LED and buzzer
 *  - While you hold the onboard B1 button the blue LED is on; it turns off when you
 *    release it. Each press beeps briefly and prints "Button pressed" (messages are
 *    printed only when the state CHANGES).
 *  - B1 (GPIO0) reads LOW while pressed: button1Read() == false -> pressed.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim  -> command list
 *      on     / ac      -> blue LED on (follows the button again on the next press)
 *      off    / kapat   -> blue LED off
 *      beep   / bip     -> a short sound from the buzzer
 *      status / durum   -> button state and how many times it was pressed
 *      lang   / dil     -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. / NO extra module needed.
 */

#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

bool lastPressed = false;     // Butonun son durumu / last button state
uint32_t lastChangeMs = 0;    // Son değişim zamanı (titreşim filtresi) / last change (debounce)
uint32_t pressCount = 0;      // Basma sayısı / press count

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "AÇ" -> "ac"
// Lower-cases and simplifies Turkish letters: "AÇ" -> "ac"
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
void printHelp() {
  minibot.serialWrite(L("---- MINIBOT TEMEL - Komutlar ----", "---- MINIBOT BASIC - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  ac / kapat    : mavi LED'i yak / söndür", "  on / off      : blue LED on / off"));
  minibot.serialWrite(L("  bip           : kısa bir ses", "  beep          : a short sound"));
  minibot.serialWrite(L("  durum         : buton durumu", "  status        : button state"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu     : basılı tutunca LED yanar", "  B1 button     : hold it to light the LED"));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "ac" || cmd == "on") {
    minibot.ledWrite(true);
    minibot.serialWrite(L("LED yandı.", "LED on."));
  } else if (cmd == "kapat" || cmd == "off") {
    minibot.ledWrite(false);
    minibot.serialWrite(L("LED söndü.", "LED off."));
  } else if (cmd == "bip" || cmd == "beep") {
    minibot.buzzerPlay(1000, 150); // 1000 Hz, 150 ms (arka planda çalar) / plays in the background
    minibot.serialWrite(L("Bip!", "Beep!"));
  } else if (cmd == "durum" || cmd == "status") {
    minibot.serialWrite(String(lastPressed ? L("Buton: BASILI", "Button: PRESSED") : L("Buton: serbest", "Button: released")) +
                        L("  |  basma sayısı: ", "  |  press count: ") + pressCount);
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
  minibot.playIntro();         // Mavi LED 3 kez yanıp söner / the blue LED blinks 3 times
  minibot.serialStart(115200); // Seri haberleşme (115200 baud) / Serial communication (115200 baud)
  minibot.serialWrite(L("MINIBOT'a hoş geldiniz! B1 butonuna basın.", "Welcome to MINIBOT! Press the B1 button."));
  printHelp();
}

void loop() {
  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) Butonu oku: button1Read() == false -> basılı.
  //    Sadece durum değişince (ve 40 ms titreşim filtresinden sonra) işlem yap.
  // 2) Read the button: button1Read() == false -> pressed.
  //    Act only when the state changes (after a 40 ms debounce).
  bool pressed = !minibot.button1Read();
  if (pressed != lastPressed && millis() - lastChangeMs > 40) {
    lastChangeMs = millis();
    lastPressed = pressed;
    minibot.ledWrite(pressed); // Basılıyken LED yanar / LED on while pressed
    if (pressed) {
      pressCount++;
      minibot.buzzerPlay(1500, 50);
      minibot.serialWrite(String(L("Butona basıldı! (", "Button pressed! (")) + pressCount + ")");
    } else {
      minibot.serialWrite(L("Buton bırakıldı.", "Button released."));
    }
  }
}
