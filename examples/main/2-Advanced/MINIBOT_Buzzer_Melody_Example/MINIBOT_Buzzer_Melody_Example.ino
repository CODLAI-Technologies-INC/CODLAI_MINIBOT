/*
 * TR: MÜZİK / MELODİ ÖZELLİKLERİ - Otomatik demo + Manuel kontrol
 *  - Açılışta OTOMATİK mod çalışır: kartın buzzer'ı sırayla şunları çalar:
 *    özel melodi (C4-E4-G4-C5), Doğum Günü, Twinkle Twinkle, Jingle Bells ve hızlı
 *    tempoda (200 BPM) Başlangıç Melodisi; sonra baştan başlar.
 *  - B1 butonuna basınca MANUEL moda geçer: müzik durur, ne çalınacağını seri
 *    porttan siz seçersiniz. B1'e tekrar basınca otomatik demoya döner.
 *  - Notalar "C4", "D#5", "Bb3" gibi adlarla yazılır (C=Do, D=Re, E=Mi, F=Fa,
 *    G=Sol, A=La, B=Si; sayı = oktav). Tempo BPM = dakikadaki vuruş sayısı.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim      / help         -> komut listesi
 *      oto         / auto         -> otomatik demo
 *      manuel      / manual       -> manuel mod
 *      melodi 1-5  / melody 1-5   -> melodi çal (0 = özel melodi) - beklemeden çalar
 *      nota C4     / note C4      -> tek nota çal (minibot.buzzerPlayNote)
 *      tempo 150                  -> tempo (BPM, 20-300) (minibot.buzzerSetTempo)
 *      kutuphane 1 / library 1    -> melodiyi kütüphanenin tek satırlık fonksiyonuyla
 *                                    çal: minibot.buzzerPlayMelody(1). DİKKAT: bu
 *                                    fonksiyon melodi bitene kadar programı bekletir.
 *      dur         / stop         -> çalmayı durdur
 *      dil         / lang         -> dili değiştir (Türkçe <-> English)
 *    (Çalma komutları otomatik moddaysa manuel moda geçirir.)
 *  - Melodi 5 ("Daha Dün Annemizin") şarkının tam halidir (kıta + nakarat); ezgisi
 *    "Ah! Vous dirai-je, Maman" olduğu için ilk iki satırı Melodi 2 ile aynıdır.
 *
 * EN: MUSIC / MELODY FEATURES - Automatic demo + Manual control
 *  - At startup AUTO mode runs: the board's buzzer plays in turn: a custom melody
 *    (C4-E4-G4-C5), Happy Birthday, Twinkle Twinkle, Jingle Bells and the Startup
 *    Jingle at a fast tempo (200 BPM); then it starts again.
 *  - Press B1 to switch to MANUAL mode: the music stops and you choose what to play
 *    from the serial port. Press B1 again to go back to the auto demo.
 *  - Notes are written by name like "C4", "D#5", "Bb3" (C=Do, D=Re, E=Mi, F=Fa,
 *    G=Sol, A=La, B=Si; the number = octave). Tempo BPM = beats per minute.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help       / yardim        -> command list
 *      auto       / oto           -> auto demo
 *      manual     / manuel        -> manual mode
 *      melody 1-5 / melodi 1-5    -> play a melody (0 = custom melody) - without blocking
 *      note C4    / nota C4       -> play one note (minibot.buzzerPlayNote)
 *      tempo 150                  -> tempo (BPM, 20-300) (minibot.buzzerSetTempo)
 *      library 1  / kutuphane 1   -> play the melody with the library's one-line
 *                                    function: minibot.buzzerPlayMelody(1). NOTE: this
 *                                    function blocks the program until the melody ends.
 *      stop       / dur           -> stop playing
 *      lang       / dil           -> switch language (Turkish <-> English)
 *    (Play commands switch to manual mode if auto mode is running.)
 *  - Melody 5 ("Daha Dün Annemizin") is the full song (verse + chorus); it uses the
 *    "Ah! Vous dirai-je, Maman" tune, so its first two lines match Melody 2.
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ - kartın buzzer'ı (IO5) kullanılır.
 * NO extra module needed - the board's buzzer (IO5) is used.
 *
 * NEDEN KENDİ ÇALARIMIZ? / WHY OUR OWN PLAYER?
 *  minibot.buzzerPlayMelody() çok kolaydır ama melodi bitene kadar (10-20 sn) her şeyi
 *  bekletir: B1 ve seri komutlar o sırada çalışmaz. Bu örnek notaları millis() ile
 *  tek tek çalar; böylece buton ve komutlar her an cevap verir.
 *  minibot.buzzerPlayMelody() is very easy but it blocks everything until the melody
 *  ends (10-20 s): B1 and serial commands don't work meanwhile. This example plays
 *  the notes one by one with millis(), so the button and commands always respond.
 */

#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

MINIBOT minibot; // MINIBOT nesnesi / MINIBOT object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// ---------------------------------------------------------------------------
// Melodiler: nota adı + vuruş (1 = çeyrek nota) / Melodies: note name + beats (1 = quarter note)
// ---------------------------------------------------------------------------
struct Note {
  const char *name;
  float beats;
};

const Note MELODY_CUSTOM[] = {{"C4", 0.6}, {"E4", 0.6}, {"G4", 0.6}, {"C5", 1}};

const Note MELODY_BIRTHDAY[] = {
    {"C4", 0.75}, {"C4", 0.25}, {"D4", 1}, {"C4", 1}, {"F4", 1}, {"E4", 2},
    {"C4", 0.75}, {"C4", 0.25}, {"D4", 1}, {"C4", 1}, {"G4", 1}, {"F4", 2},
    {"C4", 0.75}, {"C4", 0.25}, {"C5", 1}, {"A4", 1}, {"F4", 1}, {"E4", 1}, {"D4", 1},
    {"A#4", 0.75}, {"A#4", 0.25}, {"A4", 1}, {"F4", 1}, {"G4", 1}, {"F4", 2}};

const Note MELODY_TWINKLE[] = {
    {"C4", 1}, {"C4", 1}, {"G4", 1}, {"G4", 1}, {"A4", 1}, {"A4", 1}, {"G4", 2},
    {"F4", 1}, {"F4", 1}, {"E4", 1}, {"E4", 1}, {"D4", 1}, {"D4", 1}, {"C4", 2}};

const Note MELODY_JINGLE[] = {
    {"E4", 1}, {"E4", 1}, {"E4", 2}, {"E4", 1}, {"E4", 1}, {"E4", 2},
    {"E4", 1}, {"G4", 1}, {"C4", 1.5}, {"D4", 0.5}, {"E4", 4}};

const Note MELODY_STARTUP[] = {{"C4", 0.5}, {"E4", 0.5}, {"G4", 0.5}, {"C5", 1}};

const Note MELODY_DAHA_DUN[] = {
    {"C4", 1}, {"C4", 1}, {"G4", 1}, {"G4", 1}, {"A4", 1}, {"A4", 1}, {"G4", 2},
    {"F4", 1}, {"F4", 1}, {"E4", 1}, {"E4", 1}, {"D4", 1}, {"D4", 1}, {"C4", 2},
    {"G4", 1}, {"G4", 1}, {"F4", 1}, {"F4", 1}, {"E4", 1}, {"E4", 1}, {"D4", 2},
    {"G4", 1}, {"G4", 1}, {"F4", 1}, {"F4", 1}, {"E4", 1}, {"E4", 1}, {"D4", 2},
    {"C4", 1}, {"C4", 1}, {"G4", 1}, {"G4", 1}, {"A4", 1}, {"A4", 1}, {"G4", 2},
    {"F4", 1}, {"F4", 1}, {"E4", 1}, {"E4", 1}, {"D4", 1}, {"D4", 1}, {"C4", 2}};

#define COUNT_OF(a) (int)(sizeof(a) / sizeof(a[0]))

struct Melody {
  const Note *notes;
  int count;
  const char *nameTr;
  const char *nameEn;
};

const Melody MELODIES[] = {
    {MELODY_CUSTOM, COUNT_OF(MELODY_CUSTOM), "Özel melodi: C4-E4-G4-C5", "Custom melody: C4-E4-G4-C5"},
    {MELODY_BIRTHDAY, COUNT_OF(MELODY_BIRTHDAY), "Doğum Günü", "Happy Birthday"},
    {MELODY_TWINKLE, COUNT_OF(MELODY_TWINKLE), "Twinkle Twinkle", "Twinkle Twinkle"},
    {MELODY_JINGLE, COUNT_OF(MELODY_JINGLE), "Jingle Bells", "Jingle Bells"},
    {MELODY_STARTUP, COUNT_OF(MELODY_STARTUP), "Başlangıç Melodisi", "Startup Jingle"},
    {MELODY_DAHA_DUN, COUNT_OF(MELODY_DAHA_DUN), "Daha Dün Annemizin", "Daha Dun Annemizin"}};
const int MELODY_COUNT = COUNT_OF(MELODIES);

// ---------------------------------------------------------------------------
// Çalar durumu / Player state
// ---------------------------------------------------------------------------
bool manualMode = false;   // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
int bpm = 120;             // Tempo (dakikadaki vuruş) / tempo (beats per minute)
int playing = -1;          // Çalan melodi (-1 = yok) / melody playing (-1 = none)
int noteIndex = 0;         // Sıradaki nota / next note
uint32_t nextNoteMs = 0;   // Sıradaki notanın zamanı / when the next note starts
int autoIndex = -1;        // Otomatik demodaki sıra / position in the auto demo
uint32_t melodyEndMs = 0;  // Son melodinin bittiği an / when the last melody ended

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

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "KÜTÜPHANE" -> "kutuphane"
// Lower-cases and simplifies Turkish letters: "KÜTÜPHANE" -> "kutuphane"
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
// Nota -> frekans (Hz). "A4" = 440 Hz; her yarım ses 2^(1/12) kat.
// Note -> frequency (Hz). "A4" = 440 Hz; each semitone is 2^(1/12) times.
// Tanınmayan ad 0 döndürür. / An unknown name returns 0.
// ---------------------------------------------------------------------------
int noteFreq(const char *note) {
  if (!note || !note[0]) return 0;
  static const int semitoneFromC[7] = {9, 11, 0, 2, 4, 5, 7}; // A,B,C,D,E,F,G
  char letter = toupper(note[0]);
  if (letter < 'A' || letter > 'G') return 0;
  int semitone = semitoneFromC[letter - 'A'];
  int pos = 1;
  if (note[pos] == '#') { semitone++; pos++; }
  else if (note[pos] == 'b' || note[pos] == 'B') { semitone--; pos++; }
  if (note[pos] != '\0' && !isdigit(note[pos])) return 0;
  int octave = (note[pos] != '\0') ? atoi(&note[pos]) : 4;
  int fromA4 = (octave - 4) * 12 + (semitone - 9);
  return (int)round(440.0 * pow(2.0, fromA4 / 12.0));
}

void startMelody(int id) {
  playing = id;
  noteIndex = 0;
  nextNoteMs = millis();
  minibot.serialWrite(String(L("Çalıyor: ", "Playing: ")) + id + " - " + L(MELODIES[id].nameTr, MELODIES[id].nameEn) +
                      " (" + bpm + " BPM)");
}

// Sırası gelen notayı başlatır; buzzerPlay() sesi arka planda çalar, hiç beklemez.
// Starts the note whose turn it is; buzzerPlay() plays in the background, never waits.
void updatePlayer() {
  if (playing < 0 || (int32_t)(millis() - nextNoteMs) < 0) return;
  const Melody &m = MELODIES[playing];
  if (noteIndex >= m.count) {
    playing = -1;
    melodyEndMs = millis();
    return;
  }
  const Note &n = m.notes[noteIndex++];
  uint32_t durationMs = (uint32_t)(n.beats * 60000.0f / bpm);
  int freq = noteFreq(n.name);
  if (freq > 0) minibot.buzzerPlay(freq, durationMs);
  nextNoteMs = millis() + durationMs + durationMs / 10; // Notalar arası kısa boşluk / short gap between notes
}

// ---------------------------------------------------------------------------
// Mesajlar ve komutlar / Messages and commands
// ---------------------------------------------------------------------------
void printHelp() {
  minibot.serialWrite(L("---- MELODİ - Komutlar ----", "---- MELODY - Commands ----"));
  minibot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  minibot.serialWrite(L("  oto / manuel  : otomatik demo / manuel mod", "  auto / manual : auto demo / manual mode"));
  minibot.serialWrite(L("  melodi 0-5    : melodi çal (beklemeden)", "  melody 0-5    : play a melody (non-blocking)"));
  for (int i = 0; i < MELODY_COUNT; i++) {
    minibot.serialWrite(String("     ") + i + " = " + L(MELODIES[i].nameTr, MELODIES[i].nameEn));
  }
  minibot.serialWrite(L("  nota C4       : tek nota (ör. D#5, Bb3)", "  note C4       : one note (e.g. D#5, Bb3)"));
  minibot.serialWrite(L("  tempo 20-300  : tempo (BPM)", "  tempo 20-300  : tempo (BPM)"));
  minibot.serialWrite(L("  kutuphane 1-5 : buzzerPlayMelody (bekletir!)", "  library 1-5   : buzzerPlayMelody (blocks!)"));
  minibot.serialWrite(L("  dur           : çalmayı durdur", "  stop          : stop playing"));
  minibot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  minibot.serialWrite(L("  B1 butonu     : OTOMATİK <-> MANUEL", "  B1 button     : AUTO <-> MANUAL"));
}

void setMode(bool manual) {
  manualMode = manual;
  playing = -1; // Çalan melodiyi bırak (son nota kendiliğinden biter) / drop the melody (the last note ends by itself)
  minibot.ledWrite(manual); // Mavi LED yanıyorsa MANUEL / blue LED on = MANUAL
  if (manual) {
    minibot.serialWrite(L(">> MANUEL mod: \"melodi 1\" ya da \"nota C4\" yazın.", ">> MANUAL mode: type \"melody 1\" or \"note C4\"."));
    minibot.buzzerPlay(1500, 60); // Kısa bip / short beep
  } else {
    minibot.serialWrite(L(">> OTOMATİK mod: melodiler sırayla çalıyor.", ">> AUTO mode: the melodies play in turn."));
    autoIndex = -1;
    melodyEndMs = millis() - 10000; // Hemen başla / start right away
  }
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  String arg = (space < 0) ? "" : cmd.substring(space + 1);
  bool hasValue = arg.length() > 0;
  int value = arg.toInt();

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oto" || word == "otomatik" || word == "auto") {
    setMode(false);
  } else if (word == "manuel" || word == "manual") {
    setMode(true);
  } else if ((word == "melodi" || word == "melody") && hasValue) {
    if (value < 0 || value >= MELODY_COUNT) {
      minibot.serialWrite(L("Melodi numarası 0-5 olmalı.", "Melody number must be 0-5."));
      return;
    }
    if (!manualMode) setMode(true);
    startMelody(value);
  } else if ((word == "nota" || word == "note") && hasValue) {
    if (noteFreq(arg.c_str()) == 0) {
      minibot.serialWrite(L("Bilinmeyen nota. Örnek: nota C4, nota D#5, nota Bb3", "Unknown note. Example: note C4, note D#5, note Bb3"));
      return;
    }
    if (!manualMode) setMode(true);
    playing = -1;
    minibot.serialWrite(String(L("Nota: ", "Note: ")) + arg + " = " + noteFreq(arg.c_str()) + " Hz");
    minibot.buzzerPlayNote(arg.c_str(), 400); // Nota adıyla çal (400 ms bekler) / play by name (waits 400 ms)
  } else if (word == "tempo" && hasValue) {
    bpm = constrain(value, 20, 300);
    minibot.buzzerSetTempo(bpm); // Kütüphanenin melodi temposu da aynı olsun / keep the library's tempo the same
    minibot.serialWrite(String(L("Tempo: ", "Tempo: ")) + bpm + " BPM");
  } else if ((word == "kutuphane" || word == "library") && hasValue) {
    if (value < 1 || value > 5) {
      minibot.serialWrite(L("Kütüphane melodisi 1-5 olmalı.", "Library melody must be 1-5."));
      return;
    }
    if (!manualMode) setMode(true);
    playing = -1;
    minibot.serialWrite(L("minibot.buzzerPlayMelody() çalıyor - bitene kadar buton/komut çalışmaz...",
                          "minibot.buzzerPlayMelody() playing - button/commands wait until it ends..."));
    minibot.buzzerPlayMelody(value); // Tek satır ama bekletir / one line, but blocks
    minibot.serialWrite(L("Bitti.", "Done."));
  } else if (word == "dur" || word == "stop") {
    playing = -1;
    if (!manualMode) setMode(true);
    minibot.serialWrite(L("Durduruldu.", "Stopped."));
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
  minibot.buzzerSetTempo(bpm);
  minibot.serialWrite(L("Melodi örneği başladı.", "Melody example started."));
  printHelp();
  melodyEndMs = millis() - 10000; // Otomatik demo hemen başlasın / start the auto demo right away
}

void loop() {
  // 1) B1 -> mod değiştir / B1 -> toggle mode
  if (b1Pressed()) setMode(!manualMode);

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Otomatik demo: bir melodi bitince 1,5 sn sonra sıradakine geç.
  //    Başlangıç Melodisi (4) hızlı tempoda (200 BPM) çalar.
  // 3) Auto demo: 1.5 s after a melody ends, go to the next one.
  //    The Startup Jingle (4) plays at a fast tempo (200 BPM).
  if (!manualMode && playing < 0 && millis() - melodyEndMs >= 1500) {
    autoIndex = (autoIndex + 1) % 5; // 0..4
    bpm = (autoIndex == 4) ? 200 : 120;
    startMelody(autoIndex);
  }

  // 4) Sıradaki notayı çal (beklemeden) / play the next note (non-blocking)
  updatePlayer();
}
