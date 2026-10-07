/*
 * TR: VİKİPEDİ ARAMA ÖRNEĞİ
 *  - MINIBOT WiFi'ye bağlanır, Vikipedi'de "Robot" konusunu arar ve kısa özetini
 *    seri porta yazar. Sonra seri porttan istediğiniz konuyu aratabilirsiniz.
 *  - Arama, seçili dilin Vikipedi'sinde yapılır: Türkçe -> tr.wikipedia.org,
 *    English -> en.wikipedia.org ("dil" komutu ikisini de değiştirir).
 *  - Konu adı Vikipedi'deki başlıkla aynı olmalıdır (ör. "Mustafa Kemal Atatürk",
 *    "Ay"). Boşluklar otomatik olarak "_" yapılır.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim     / help          -> komut listesi
 *      ara Kedi   / search Cat    -> konuyu ara ve özetini yaz
 *      dil        / lang          -> dili değiştir (Türkçe <-> English)
 *
 * EN: WIKIPEDIA SEARCH EXAMPLE
 *  - MINIBOT joins WiFi, looks up "Robot" on Wikipedia and prints its short summary.
 *    After that you can search any topic from the serial port.
 *  - The search uses the Wikipedia of the selected language: Turkish ->
 *    tr.wikipedia.org, English -> en.wikipedia.org (the "lang" command switches both).
 *  - The topic must match the Wikipedia title (e.g. "Albert Einstein", "Moon").
 *    Spaces are turned into "_" automatically.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help       / yardim        -> command list
 *      search Cat / ara Kedi      -> search the topic and print its summary
 *      lang       / dil           -> switch language (Turkish <-> English)
 */

#define USE_WIKIPEDIA
#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// cmdRaw: komutun harfleri değiştirilmemiş hali (aranan konu için).
// cmdRaw: the command with its letters untouched (for the search topic).
// ---------------------------------------------------------------------------
String cmdBuffer;
String cmdRaw;
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
// Vikipedi ve mesajlar / Wikipedia and messages
// ---------------------------------------------------------------------------
// Konuyu olduğu gibi verin: getWikipedia() boşlukları "_" yapar, Türkçe harfleri ve
// işaretleri adres (URL) için kendisi kodlar. / Pass the topic as it is: getWikipedia()
// turns spaces into "_" and encodes Turkish letters and symbols by itself.
void search(const String &topic) {
  if (WiFi.status() != WL_CONNECTED) {
    minibot.serialWrite(L("WiFi bağlı değil - arama yapılamıyor.", "WiFi not connected - can't search."));
    return;
  }
  minibot.serialWrite(String(L("Aranıyor: '", "Searching for: '")) + topic + "' (" + L("tr", "en") + ".wikipedia.org)...");
  String summary = minibot.getWikipedia(topic, L("tr", "en")); // Birkaç saniye sürebilir / may take a few seconds
  if (summary == "No Summary Found") summary = L("Özet bulunamadı (başlığı kontrol edin).", "No summary found (check the title).");
  minibot.serialWrite(L("Özet:", "Summary:"));
  minibot.serialWrite(summary);
}

void printHelp() {
  minibot.serialWrite(L("---- VİKİPEDİ - Komutlar ----", "---- WIKIPEDIA - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  ara <konu>    : konuyu ara (ör. ara Kedi)", "  search <topic>: search a topic (e.g. search Cat)"));
  minibot.serialWrite(L("  dil           : English'e geç (en.wikipedia)", "  lang          : switch to Turkish (tr.wikipedia)"));
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if ((word == "ara" || word == "search") && hasValue) {
    String topic = cmdRaw.substring(cmdRaw.indexOf(' ') + 1);
    topic.trim();
    search(topic);
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    minibot.serialWrite(L("Dil: Türkçe (aramalar tr.wikipedia.org'da)", "Language: English (searches on en.wikipedia.org)"));
    printHelp();
  } else {
    minibot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  minibot.begin();             // MINIBOT başlatılıyor / Initialize MINIBOT
  minibot.serialStart(115200); // Seri haberleşme / Serial communication
  minibot.serialWrite(L("Vikipedi Arama Örneği", "Wikipedia Search Example"));

  minibot.wifiStartAndConnect(WIFI_SSID, WIFI_PASSWORD);
  if (minibot.wifiConnectionControl()) {
    search("Robot");
  }
  printHelp();
}

void loop() {
  // Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);
}
