/*
 * TR: GERÇEK PROJE - Refleks Oyunu ("Ne kadar hızlısın?")
 *  - B1 butonuna basıp oyunu başlatın. Kart 2-5 saniye arası RASTGELE bir süre
 *    bekler, sonra mavi LED yanar ve buzzer "bip" der - o anda B1'e olabildiğince
 *    hızlı basın! Tepki süreniz milisaniye (ms) olarak ölçülür, seri porta bir not
 *    ile yazılır ve en iyi skorunuz saklanır. LED yanmadan basarsanız
 *    "ERKEN BASTIN!" olur - hile yok :)
 *  - B1 bu projede OYUN butonudur.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim  / help     -> komut listesi
 *      basla   / start    -> yeni tur başlat (B1 gibi)
 *      rekor   / best     -> en iyi skoru yaz
 *      sifirla / reset    -> en iyi skoru sil
 *      dil     / lang     -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Reflex Game ("How fast are you?")
 *  - Press B1 to start. The board waits a RANDOM time between 2 and 5 seconds, then
 *    the blue LED lights up and the buzzer beeps - press B1 as fast as you can! Your
 *    reaction time is measured in milliseconds (ms), printed with a grade, and your
 *    best score is kept. Press before the LED lights up and it is a "FALSE START!" -
 *    no cheating :)
 *  - In this project B1 is the GAME button.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help  / yardim     -> command list
 *      start / basla      -> start a new round (like B1)
 *      best  / rekor      -> print the best score
 *      reset / sifirla    -> clear the best score
 *      lang  / dil        -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ - sadece kartın B1 butonu, mavi LED'i ve
 * buzzer'ı kullanılır. / NO extra module needed - only the board's B1 button, blue
 * LED and buzzer are used.
 */

#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

const uint32_t kMinWaitMs = 2000;  // En kısa rastgele bekleme / shortest random wait
const uint32_t kMaxWaitMs = 5000;  // En uzun rastgele bekleme / longest random wait
const uint32_t kTooSlowMs = 2000;  // Bundan sonra "çok yavaş" / after this: "too slow"

enum State { IDLE, WAITING, GO };
State state = IDLE;
uint32_t goAtMs = 0;               // LED'in yanacağı an / when the LED will light up
uint32_t goStartMs = 0;            // LED'in yandığı an / when the LED lit up
uint32_t bestMs = 0;               // En iyi skor (0 = henüz yok), sadece RAM'de / best score (0 = none yet), RAM only

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

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "BAŞLA" -> "basla"
// Lower-cases and simplifies Turkish letters: "BAŞLA" -> "basla"
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
// Oyun / Game
// ---------------------------------------------------------------------------
const char *grade(uint32_t ms) {
  if (ms < 200) return L("MUHTEŞEM - jet pilotu gibi!", "AMAZING - like a jet pilot!");
  if (ms < 280) return L("ÇOK İYİ - yarışçı refleksi!", "VERY GOOD - racer reflexes!");
  if (ms < 380) return L("İYİ - ortalama bir insan", "GOOD - an average person");
  return L("Biraz yavaş - tekrar dene!", "A bit slow - try again!");
}

void startRound() {
  // Butona basma anınız her seferinde farklı -> iyi bir rastgele tohum.
  // The moment you press is different every time -> a good random seed.
  randomSeed(micros());
  goAtMs = millis() + random(kMinWaitMs, kMaxWaitMs + 1);
  state = WAITING;
  minibot.serialWrite(L("Hazır ol... LED yanınca B1'e bas!", "Get ready... press B1 when the LED lights up!"));
}

void printHelp() {
  minibot.serialWrite(L("---- REFLEKS OYUNU - Komutlar ----", "---- REFLEX GAME - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  basla         : yeni tur (B1 gibi)", "  start         : new round (like B1)"));
  minibot.serialWrite(L("  rekor         : en iyi skor", "  best          : best score"));
  minibot.serialWrite(L("  sifirla       : en iyi skoru sil", "  reset         : clear the best score"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu     : başlat / LED yanınca bas", "  B1 button     : start / press when the LED is on"));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "basla" || cmd == "start") {
    if (state == IDLE) startRound();
    else minibot.serialWrite(L("Tur zaten sürüyor - LED'i bekleyin!", "A round is already running - wait for the LED!"));
  } else if (cmd == "rekor" || cmd == "best") {
    if (bestMs == 0) minibot.serialWrite(L("Henüz skor yok.", "No score yet."));
    else minibot.serialWrite(String(L("En iyi skor: ", "Best score: ")) + bestMs + " ms");
  } else if (cmd == "sifirla" || cmd == "reset") {
    bestMs = 0;
    minibot.serialWrite(L("En iyi skor silindi.", "Best score cleared."));
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
  minibot.serialWrite(L("REFLEKS OYUNU - Başlamak için B1'e basın.", "REFLEX GAME - Press B1 to start."));
  printHelp();
}

void loop() {
  bool pressed = b1Pressed();
  uint32_t now = millis();

  // Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  switch (state) {
    case IDLE:
      if (pressed) startRound();
      break;

    case WAITING:
      if (pressed) {                 // LED yanmadan basıldı / pressed before the LED
        minibot.buzzerPlay(300, 400);
        minibot.serialWrite(L("ERKEN BASTIN! Tekrar denemek için B1.", "FALSE START! Press B1 to try again."));
        state = IDLE;
      } else if ((int32_t)(now - goAtMs) >= 0) {
        minibot.ledWrite(true);
        minibot.buzzerPlay(2000, 80);
        goStartMs = millis();
        state = GO;
      }
      break;

    case GO:
      if (pressed) {
        uint32_t reactionMs = now - goStartMs;
        minibot.ledWrite(false);
        bool newBest = (bestMs == 0 || reactionMs < bestMs);
        if (newBest) bestMs = reactionMs;
        minibot.serialWrite(String(L("Tepki süresi: ", "Reaction time: ")) + reactionMs + " ms -> " + grade(reactionMs));
        if (newBest) {
          minibot.serialWrite(L("*** YENİ REKOR! ***", "*** NEW RECORD! ***"));
          minibot.buzzerPlay(1500, 300);
        } else {
          minibot.serialWrite(String(L("En iyi skor: ", "Best score: ")) + bestMs + " ms");
        }
        minibot.serialWrite(L("Tekrar oynamak için B1.", "Press B1 to play again."));
        state = IDLE;
      } else if (now - goStartMs >= kTooSlowMs) {
        minibot.ledWrite(false);
        minibot.serialWrite(L("Çok yavaş! (2 sn geçti) Tekrar için B1.", "Too slow! (2 s passed) Press B1 to retry."));
        state = IDLE;
      }
      break;
  }
}
