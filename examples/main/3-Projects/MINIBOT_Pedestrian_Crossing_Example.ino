// TR: GERCEK PROJE - Butonlu Yaya Gecidi. Arabalar icin isik normalde
// YESILDIR. Yaya Buton 1'e basinca istek kaydedilir (mavi LED yanar =
// "BEKLEYINIZ"). Arabalar en az 8 saniye yesil gormeden isik degismez;
// sonra SARI (2 sn) -> KIRMIZI (6 sn, yayalar gecer) -> tekrar YESIL.
// Kirmizi sirasinda buzzer "tik tik" yapar, son 2 saniyede hizlanir - tipki
// gorme engelliler icin sesli yaya gecitleri gibi. Kod hic beklemeden
// (millis ile) calisir, buton her an cevap verir.
// EN: A REAL PROJECT - Push-Button Pedestrian Crossing. The car light is
// normally GREEN. When a pedestrian presses Button 1, the request is stored
// (blue LED on = "PLEASE WAIT"). The light never changes before cars have
// had at least 8 seconds of green; then YELLOW (2 s) -> RED (6 s, pedestrians
// cross) -> GREEN again. During red the buzzer ticks, faster in the last 2
// seconds - just like talking crossings for visually impaired people. The
// code never blocks (uses millis), so the button always responds.
//
// Baglanti / Wiring: Trafik isigi modulu sabit pinler kullanir (KIRMIZI=IO13,
// SARI=IO5, YESIL=IO4) - herhangi bir soket secmenize gerek yok. / The
// traffic light module uses fixed pins (RED=IO13, YELLOW=IO5, GREEN=IO4) -
// no socket to choose.
// NOT: Kartin buzzer'i da IO5'i (SARI isik) kullanir. Bu yuzden tikler sadece
// KIRMIZI evrede (sari sonukken) calinir; her tikte sari isik hafifce parlarsa
// bu normaldir. / NOTE: The board's buzzer also uses IO5 (YELLOW light). So
// ticks are only played in the RED phase (while yellow is off); if the yellow
// light glows faintly on each tick, that is normal.

#include <MINIBOT.h>

MINIBOT minibot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

namespace {
  constexpr uint32_t kMinGreenMs = 8000;   // Arabalar icin en kisa yesil / shortest green for cars
  constexpr uint32_t kYellowMs = 2000;     // Sari suresi / yellow time
  constexpr uint32_t kRedMs = 6000;        // Yaya gecis suresi / pedestrian crossing time
  constexpr uint32_t kFastTickLastMs = 2000; // Son 2 sn hizli tik / fast ticks in the last 2 s

  enum Phase { CAR_GREEN, CAR_YELLOW, CAR_RED };
  Phase phase = CAR_GREEN;
  uint32_t phaseStartMs = 0;
  bool requested = false;                  // Yaya istegi var mi / pedestrian request waiting?
  bool lastButton = true;                  // HIGH = birakilmis / released
  uint32_t lastPressMs = 0, lastTickMs = 0;

  void enterPhase(Phase p) {
    phase = p;
    phaseStartMs = millis();
    // Isigi SADECE evre degisince yaz: surekli yazmak IO5'teki buzzer sesini keserdi.
    // Write the light ONLY on a phase change: writing it constantly would cut the buzzer tone on IO5.
    if (p == CAR_GREEN) {
      minibot.moduleTraficLightWrite(false, false, true);
      minibot.serialWrite(turkish ? "Arabalar: YESIL (yaya bekler)" : "Cars: GREEN (pedestrians wait)");
    } else if (p == CAR_YELLOW) {
      minibot.moduleTraficLightWrite(false, true, false);
      minibot.serialWrite(turkish ? "Arabalar: SARI (yavasla!)" : "Cars: YELLOW (slow down!)");
    } else {
      minibot.moduleTraficLightWrite(true, false, false);
      requested = false;
      minibot.ledWrite(false);
      lastTickMs = 0;
      minibot.serialWrite(turkish ? "Arabalar: KIRMIZI -> Yayalar GECEBILIR" : "Cars: RED -> Pedestrians may CROSS");
    }
  }
}

void setup() {
  minibot.begin();
  minibot.serialStart(115200);
  minibot.ledWrite(false);
  minibot.serialWrite(turkish ? "Yaya gecidi hazir. Karsiya gecmek icin Buton 1'e basin."
                              : "Pedestrian crossing ready. Press Button 1 to cross.");
  enterPhase(CAR_GREEN);
}

void loop() {
  uint32_t now = millis();

  // 1) Yaya butonu (sadece yesil/sari evrede istek kaydedilir).
  // 1) Pedestrian button (a request is stored only in the green/yellow phase).
  bool button = minibot.button1Read(); // false = basili / pressed
  if (lastButton && !button && now - lastPressMs > 250) { // 250 ms debounce
    lastPressMs = now;
    if (phase != CAR_RED && !requested) {
      requested = true;
      minibot.ledWrite(true); // "BEKLEYINIZ" isigi / "PLEASE WAIT" light
      minibot.serialWrite(turkish ? "Istek alindi - BEKLEYINIZ..." : "Request received - PLEASE WAIT...");
    }
  }
  lastButton = button;

  uint32_t inPhase = now - phaseStartMs;
  switch (phase) {
    case CAR_GREEN:
      // Istek var VE arabalar en az 8 sn yesil gorduyse sariya gec.
      // Request waiting AND cars have had at least 8 s of green -> go yellow.
      if (requested && inPhase >= kMinGreenMs) enterPhase(CAR_YELLOW);
      break;

    case CAR_YELLOW:
      if (inPhase >= kYellowMs) enterPhase(CAR_RED);
      break;

    case CAR_RED: {
      // Normalde saniyede 1 tik, son 2 saniyede saniyede 4 tik.
      // Normally 1 tick per second, 4 ticks per second in the last 2 seconds.
      uint32_t tickGap = (inPhase >= kRedMs - kFastTickLastMs) ? 250 : 1000;
      if (lastTickMs == 0 || now - lastTickMs >= tickGap) {
        lastTickMs = now;
        minibot.buzzerPlay(2000, 40); // Kisa "tik" (arka planda) / short "tick" (in the background)
      }
      if (inPhase >= kRedMs) enterPhase(CAR_GREEN);
      break;
    }
  }

  delay(10);
}
