/*
 * TR: IFTTT WEBHOOK ÖRNEĞİ
 *  - MINIBOT'tan IFTTT Webhooks olayı tetikler: açılışta bir "Açıldı" olayı, sonra
 *    B1 butonuna her basışta (ya da seri porttan "tetikle" yazınca) bir "Butona
 *    basıldı" olayı gönderir. IFTTT bu olayla telefonunuza bildirim, Google Sheets'e
 *    satır, Discord mesajı vb. gönderebilir. Gönderim sırasında mavi LED yanar.
 *  - Hızlı kurulum:
 *    1. https://ifttt.com/create adresinde "Webhooks" tetikleyicisini seçin; Event Name
 *       alanını aşağıdaki iftttEventName ile aynı yapın.
 *    2. İstediğiniz hedef servisi seçerek appleti tamamlayın (Google Sheets, Discord...).
 *    3. https://ifttt.com/maker_webhooks sayfasındaki "Documentation" bölümünden key
 *       değerini kopyalayıp iftttWebhookKey'e yapıştırın.
 *    4. value1 / value2 / value3 alanlarını applette "ingredient" olarak kullanabilirsiniz.
 *  - USE_IFTTT tanımlanınca WiFi yardımcıları otomatik açılır; ayrıca USE_WIFI gerekmez.
 *  - B1 bu örnekte "olayı tetikle" butonudur (iki tetikleme arası en az 4 sn).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim  / help      -> komut listesi
 *      tetikle / trigger   -> olayı şimdi tetikle
 *      durum   / status    -> WiFi durumu ve gönderilen olay sayısı
 *      dil     / lang      -> dili değiştir (Türkçe <-> English)
 *
 * EN: IFTTT WEBHOOK EXAMPLE
 *  - Triggers IFTTT Webhooks events from MINIBOT: a "Boot" event at startup, then a
 *    "Button pressed" event on every press of B1 (or typing "trigger" on the serial
 *    port). IFTTT can then send a phone notification, add a Google Sheets row, post a
 *    Discord message, etc. The blue LED is on while sending.
 *  - Quick setup:
 *    1. At https://ifttt.com/create choose the "Webhooks" trigger; make Event Name the
 *       same as iftttEventName below.
 *    2. Finish the applet with any target service (Google Sheets, Discord...).
 *    3. Copy the key from the "Documentation" section of https://ifttt.com/maker_webhooks
 *       and paste it into iftttWebhookKey.
 *    4. You can use value1 / value2 / value3 as "ingredients" in your applet.
 *  - Defining USE_IFTTT turns on the WiFi helpers automatically; no USE_WIFI needed.
 *  - In this example B1 is the "trigger the event" button (at least 4 s between triggers).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help    / yardim    -> command list
 *      trigger / tetikle   -> trigger the event now
 *      status  / durum     -> WiFi state and number of events sent
 *      lang    / dil       -> switch language (Turkish <-> English)
 */

#define USE_IFTTT
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

const char *ssid = "YOUR_WIFI_SSID";
const char *password = "YOUR_WIFI_PASSWORD";

String iftttEventName = "YOUR_EVENT_NAME"; // Örnek / example: "minibot_button"
String iftttWebhookKey = "YOUR_IFTTT_KEY"; // https://ifttt.com/maker_webhooks

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

uint32_t lastTriggerMs = 0;
const uint32_t triggerIntervalMs = 4000; // İki tetikleme arası en az 4 sn / at least 4 s between triggers
uint32_t eventCount = 0;
bool triggeredOnce = false;

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
// IFTTT ve mesajlar / IFTTT and messages
// ---------------------------------------------------------------------------
void triggerButtonEvent(const char *sourceTr, const char *sourceEn) {
  if (WiFi.status() != WL_CONNECTED) {
    minibot.serialWrite(L("WiFi bağlı değil - olay gönderilemez.", "WiFi not connected - can't send the event."));
    return;
  }
  if (triggeredOnce && millis() - lastTriggerMs < triggerIntervalMs) {
    minibot.serialWrite(L("Biraz bekleyin (iki tetikleme arası en az 4 sn).", "Please wait (at least 4 s between triggers)."));
    return;
  }
  triggeredOnce = true;
  lastTriggerMs = millis();
  minibot.ledWrite(true); // Gönderim sırasında LED yanar / LED on while sending

  // value1 = kaynak, value2 = olay, value3 = zaman / value1 = source, value2 = event, value3 = time
  String payload = String("{\"value1\":\"") + L(sourceTr, sourceEn) + "\",\"value2\":\"" +
                   L("Basildi", "Pressed") + "\",\"value3\":\"Millis:" + String(millis()) + "\"}";
  bool ok = minibot.triggerIFTTTEvent(iftttEventName, iftttWebhookKey, payload);
  if (ok) eventCount++;
  minibot.serialWrite(ok ? L("[IFTTT] Olay iletildi.", "[IFTTT] Event delivered.") : L("[IFTTT] Olay gönderilemedi.", "[IFTTT] Event failed."));

  minibot.ledWrite(false);
}

void printHelp() {
  minibot.serialWrite(L("---- IFTTT WEBHOOK - Komutlar ----", "---- IFTTT WEBHOOK - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  tetikle       : olayı şimdi tetikle", "  trigger       : trigger the event now"));
  minibot.serialWrite(L("  durum         : WiFi ve olay sayısı", "  status        : WiFi and event count"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu     : olayı tetikle", "  B1 button     : trigger the event"));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "tetikle" || cmd == "trigger" || cmd == "gonder" || cmd == "send") {
    triggerButtonEvent("Seri port", "Serial port");
  } else if (cmd == "durum" || cmd == "status") {
    minibot.serialWrite(String(WiFi.status() == WL_CONNECTED ? L("WiFi: bağlı", "WiFi: connected") : L("WiFi: bağlı DEĞİL", "WiFi: NOT connected")) +
                        L("  |  iletilen olay: ", "  |  events delivered: ") + eventCount);
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
  minibot.serialWrite(L("MINIBOT IFTTT Webhook Örneği", "MINIBOT IFTTT Webhook Example"));
  minibot.serialWrite(L("WiFi'ye bağlanılıyor...", "Connecting to WiFi..."));
  minibot.wifiStartAndConnect(ssid, password);

  if (minibot.wifiConnectionControl()) {
    minibot.serialWrite(L("WiFi bağlandı. IFTTT'ye açılış olayı gönderiliyor...", "WiFi connected. Sending boot event to IFTTT..."));
    String bootPayload = String("{\"value1\":\"MINIBOT\",\"value2\":\"") + L("Acildi", "Boot") + "\",\"value3\":\"Online\"}";
    bool ok = minibot.triggerIFTTTEvent(iftttEventName, iftttWebhookKey, bootPayload);
    if (ok) eventCount++;
    minibot.serialWrite(ok ? L("[IFTTT] Açılış olayı iletildi.", "[IFTTT] Boot event delivered.") : L("[IFTTT] Açılış olayı gönderilemedi.", "[IFTTT] Boot event failed."));
  } else {
    minibot.serialWrite(L("WiFi bağlantısı başarısız. SSID/şifreyi kontrol edin.", "WiFi connection failed. Check SSID/password."));
  }
  printHelp();
}

void loop() {
  // 1) B1 -> olayı tetikle / B1 -> trigger the event
  if (b1Pressed()) triggerButtonEvent("Buton 1", "Button 1");

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);
}
