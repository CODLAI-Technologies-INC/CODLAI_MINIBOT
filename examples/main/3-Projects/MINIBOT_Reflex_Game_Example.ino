// TR: GERCEK PROJE - Refleks Oyunu ("Ne kadar hizlisin?"). Buton 1'e basip
// oyunu baslatin. Kart 2-5 saniye arasi RASTGELE bir sure bekler, sonra
// mavi LED yanar ve buzzer "bip" der - o anda Buton 1'e olabildigince hizli
// basin! Tepki suren milisaniye (ms) olarak olculur, Seri Port'a bir not
// (derece) ile yazilir ve en iyi skorun saklanir. LED yanmadan basarsan
// "ERKEN BASTIN!" olur - hile yok :)
// EN: A REAL PROJECT - Reflex Game ("How fast are you?"). Press Button 1 to
// start. The board waits a RANDOM time between 2 and 5 seconds, then the
// blue LED lights up and the buzzer beeps - press Button 1 as fast as you
// can! Your reaction time is measured in milliseconds (ms), printed to
// Serial with a grade, and your best score is kept. Press before the LED
// lights up and it is a "FALSE START!" - no cheating :)
//
// Baglanti / Wiring: Ek modul GEREKMEZ - sadece kartin Buton 1'i, mavi LED'i
// ve buzzer'i kullanilir. Sonuclari gormek icin Seri Monitor'u (115200)
// acin. / NO extra module needed - only the board's Button 1, blue LED and
// buzzer are used. Open the Serial Monitor (115200) to see the results.

#include <MINIBOT.h>

MINIBOT minibot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

namespace {
  constexpr uint32_t kMinWaitMs = 2000;    // En kisa rastgele bekleme / shortest random wait
  constexpr uint32_t kMaxWaitMs = 5000;    // En uzun rastgele bekleme / longest random wait
  constexpr uint32_t kTooSlowMs = 2000;    // Bundan sonra "cok yavas" / after this: "too slow"
  constexpr uint32_t kDebounceMs = 40;

  enum State { IDLE, WAITING, GO };
  State state = IDLE;
  uint32_t goAtMs = 0;                     // LED'in yanacagi an / when the LED will light up
  uint32_t goStartMs = 0;                  // LED'in yandigi an / when the LED lit up
  uint32_t bestMs = 0;                     // En iyi skor (0 = henuz yok), sadece RAM'de / best score (0 = none yet), RAM only
  bool lastButton = true;                  // HIGH = birakilmis / released
  uint32_t lastEdgeMs = 0;

  void say(const String &turkishText, const String &englishText) {
    minibot.serialWrite(turkish ? turkishText : englishText);
  }

  // Buton "basildi" anini bir kez dondurur (debounce'lu). / Returns the press moment once (debounced).
  bool buttonPressed() {
    bool button = minibot.button1Read(); // false = basili / pressed
    bool edge = lastButton && !button && millis() - lastEdgeMs > kDebounceMs;
    if (button != lastButton) lastEdgeMs = millis();
    lastButton = button;
    return edge;
  }

  const char *grade(uint32_t ms) {
    if (ms < 200) return turkish ? "MUHTESEM - jet pilotu gibi!" : "AMAZING - like a jet pilot!";
    if (ms < 280) return turkish ? "COK IYI - yarisci refleksi!" : "VERY GOOD - racer reflexes!";
    if (ms < 380) return turkish ? "IYI - ortalama bir insan" : "GOOD - an average person";
    return turkish ? "Biraz yavas - tekrar dene!" : "A bit slow - try again!";
  }
}

void setup() {
  minibot.begin();
  minibot.serialStart(115200);
  minibot.ledWrite(false);
  say("REFLEKS OYUNU - Baslamak icin Buton 1'e basin.", "REFLEX GAME - Press Button 1 to start.");
}

void loop() {
  bool pressed = buttonPressed();
  uint32_t now = millis();

  switch (state) {
    case IDLE:
      if (pressed) {
        // Butona basma aniniz her seferinde farkli -> iyi bir rastgele tohum.
        // The moment you press is different every time -> a good random seed.
        randomSeed(micros());
        goAtMs = now + random(kMinWaitMs, kMaxWaitMs + 1);
        state = WAITING;
        say("Hazir ol... LED yaninca bas!", "Get ready... press when the LED lights up!");
      }
      break;

    case WAITING:
      if (pressed) {                       // LED yanmadan basildi / pressed before the LED
        minibot.buzzerPlay(300, 400);
        say("ERKEN BASTIN! Tekrar denemek icin Buton 1.", "FALSE START! Press Button 1 to try again.");
        state = IDLE;
      } else if (now >= goAtMs) {
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
        say("Tepki suresi: " + String(reactionMs) + " ms -> " + grade(reactionMs),
            "Reaction time: " + String(reactionMs) + " ms -> " + grade(reactionMs));
        say(newBest ? String("*** YENI REKOR! ***") : "En iyi skor: " + String(bestMs) + " ms",
            newBest ? String("*** NEW RECORD! ***") : "Best score: " + String(bestMs) + " ms");
        if (newBest) minibot.buzzerPlay(1500, 300);
        say("Tekrar oynamak icin Buton 1.", "Press Button 1 to play again.");
        state = IDLE;
      } else if (now - goStartMs >= kTooSlowMs) {
        minibot.ledWrite(false);
        say("Cok yavas! (2 sn gecti) Tekrar icin Buton 1.", "Too slow! (2 s passed) Press Button 1 to retry.");
        state = IDLE;
      }
      break;
  }

  delay(1); // Kisa bekleme = hassas olcum / short delay = precise timing
}
