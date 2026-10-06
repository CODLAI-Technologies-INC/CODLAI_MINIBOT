// TR: MUZIK/MELODI OZELLIKLERI. Bu ornek onboard buzzer ile: 1) tek tek
// nota adlariyla ("C4", "D#5" gibi) ozel bir melodi calmayi, 2) hazir
// melodilerden (Dogum Gunu, Twinkle Twinkle, Jingle Bells, Baslangic
// Melodisi, Daha Dun Annemizin) birini calmayi, 3) tempoyu (BPM)
// degistirmeyi gosterir.
// EN: MUSIC/MELODY FEATURES. This example shows, using the onboard
// buzzer: 1) playing a custom melody note-by-note using note names
// (like "C4", "D#5"), 2) playing one of the preset melodies (Happy
// Birthday, Twinkle Twinkle, Jingle Bells, Startup Jingle, Daha Dun
// Annemizin), 3) changing the tempo (BPM).
//
// NOT / NOTE: Melodi 5 ("Daha Dun Annemizin") sarkinin tam halidir (kita +
// nakarat); ezgisi "Ah! Vous dirai-je, Maman" oldugu icin ilk iki satiri
// Melodi 2 (Twinkle Twinkle) ile aynidir. / Melody 5 ("Daha Dun Annemizin")
// is the full song (verse + chorus); it uses the "Ah! Vous dirai-je,
// Maman" tune, so its first two lines match Melody 2 (Twinkle Twinkle).

#include <MINIBOT.h>

MINIBOT minibot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

void setup() {
  minibot.begin();
  minibot.serialStart(115200);

  // 1) Ozel nota nota melodi / custom note-by-note melody
  minibot.serialWrite(turkish ? "Ozel melodi: C4-E4-G4-C5" : "Custom melody: C4-E4-G4-C5");
  minibot.buzzerPlayNote("C4", 300);
  minibot.buzzerPlayNote("E4", 300);
  minibot.buzzerPlayNote("G4", 300);
  minibot.buzzerPlayNote("C5", 500);
  delay(500);

  // 2) Hazir melodiler / preset melodies
  minibot.buzzerSetTempo(120);
  minibot.serialWrite(turkish ? "Melodi 1: Dogum Gunu" : "Melody 1: Happy Birthday");
  minibot.buzzerPlayMelody(1);
  delay(500);

  minibot.serialWrite(turkish ? "Melodi 2: Twinkle Twinkle" : "Melody 2: Twinkle Twinkle");
  minibot.buzzerPlayMelody(2);
  delay(500);

  minibot.serialWrite(turkish ? "Melodi 3: Jingle Bells" : "Melody 3: Jingle Bells");
  minibot.buzzerPlayMelody(3);
  delay(500);

  // 3) Tempoyu degistirip baslangic melodisini tekrar cal / change tempo, replay the startup jingle
  minibot.buzzerSetTempo(200); // Daha hizli / faster
  minibot.serialWrite(turkish ? "Melodi 4 (hizli tempo): Baslangic Melodisi" : "Melody 4 (fast tempo): Startup Jingle");
  minibot.buzzerPlayMelody(4);

  minibot.serialWrite(turkish ? "Bitti!" : "Done!");
}

void loop() {
}
