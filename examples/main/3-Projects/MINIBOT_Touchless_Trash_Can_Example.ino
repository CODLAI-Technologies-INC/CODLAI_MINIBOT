// TR: GERCEK PROJE - Temassiz Cop Kutusu. Elinizi (ya da copu) kapagin
// uzerine 15 cm'den yakina getirince ultrasonik sensor bunu gorur ve servo
// motor kapagi KENDILIGINDEN acar. Eliniz gittikten 3 saniye sonra kapak
// yavasca kapanir. Kapak acikken mavi LED yanar. Hic dokunmadan - mikrop
// tasimadan - cop atmanin yolu! Servo yumusak hareket eder ama kod hic
// beklemez (millis), sensor her an okunmaya devam eder.
// EN: A REAL PROJECT - Touchless Trash Can. Bring your hand (or the trash)
// closer than 15 cm above the lid: the ultrasonic sensor sees it and the
// servo motor opens the lid BY ITSELF. 3 seconds after your hand is gone,
// the lid closes slowly. The blue LED is on while the lid is open. A way to
// throw trash away without touching anything - no germs! The servo moves
// smoothly but the code never blocks (millis), so the sensor keeps reading.
//
// Baglanti / Wiring: Ultrasonik sensor sabit pinler kullanir (TRIG=IO12,
// ECHO=IO13) - soket secmenize gerek yok. Servo motoru Port B'ye (IO4)
// takin (ultrasonik pinleriyle cakismaz). Servo kolunu kapaga baglayin. /
// The ultrasonic sensor uses fixed pins (TRIG=IO12, ECHO=IO13) - no socket
// to choose. Plug the servo motor into Port B (IO4) (it does not clash with
// the ultrasonic pins). Attach the servo arm to the lid.

#define USE_SERVO
#include <MINIBOT.h>

MINIBOT minibot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define SERVO_PIN IO4 // Port B

namespace {
  constexpr int kOpenDistanceCm = 15;      // Bu mesafeden yakin = el var / closer than this = hand present
  constexpr int kClosedAngle = 0;          // Kapak kapali acisi / lid closed angle
  constexpr int kOpenAngle = 100;          // Kapak acik acisi (kutunuza gore ayarlayin) / lid open angle (adjust to your bin)
  constexpr uint32_t kStayOpenMs = 3000;   // El gittikten sonra acik kalma suresi / stay open after the hand leaves
  constexpr uint32_t kMeasureEveryMs = 100; // Olcum araligi / measurement interval
  constexpr uint32_t kOpenStepMs = 5;      // Acilis hizi (derece basina ms) / opening speed (ms per degree)
  constexpr uint32_t kCloseStepMs = 15;    // Kapanis daha yavas (parmak sikismasin) / closing is slower (no pinched fingers)

  bool lidOpen = false;
  int nearCount = 0;                       // Arka arkaya "yakin" olcum sayisi / consecutive "near" readings
  uint32_t lastSeenMs = 0, lastMeasureMs = 0, lastStepMs = 0;
  int currentAngle = kClosedAngle;
  int targetAngle = kClosedAngle;

  void say(const String &turkishText, const String &englishText) {
    minibot.serialWrite(turkish ? turkishText : englishText);
  }
}

void setup() {
  minibot.begin();
  minibot.serialStart(115200);
  minibot.moduleServoGoAngle(SERVO_PIN, kClosedAngle, 0); // Kapali baslat / start closed
  minibot.ledWrite(false);
  say("Temassiz cop kutusu hazir. Elinizi kapaga yaklastirin.", "Touchless trash can ready. Bring your hand near the lid.");
}

void loop() {
  uint32_t now = millis();

  // 1) Mesafeyi olc. 0 = yansima yok (uzak ya da gecersiz).
  // 1) Measure the distance. 0 = no echo (far away or invalid).
  if (now - lastMeasureMs >= kMeasureEveryMs) {
    lastMeasureMs = now;
    int distance = minibot.moduleUltrasonicDistanceRead();
    bool near = distance > 0 && distance < kOpenDistanceCm;
    // Tek bir hatali olcum kapagi acmasin: arka arkaya 2 "yakin" olcum iste.
    // One bad reading must not open the lid: require 2 "near" readings in a row.
    nearCount = near ? nearCount + 1 : 0;

    if (nearCount >= 2) {
      lastSeenMs = now;
      if (!lidOpen) {
        lidOpen = true;
        targetAngle = kOpenAngle;
        minibot.ledWrite(true);
        minibot.buzzerPlay(1800, 60);
        say("Kapak ACILIYOR (" + String(distance) + " cm)", "Lid OPENING (" + String(distance) + " cm)");
      }
    }
  }

  // 2) El 3 saniyedir yoksa kapat. / Close if there has been no hand for 3 seconds.
  if (lidOpen && now - lastSeenMs >= kStayOpenMs) {
    lidOpen = false;
    targetAngle = kClosedAngle;
    minibot.ledWrite(false);
    say("Kapak kapaniyor.", "Lid closing.");
  }

  // 3) Servoyu 1'er derece hedefe tasi (beklemeden). / Step the servo 1 degree toward the target (non-blocking).
  uint32_t stepMs = lidOpen ? kOpenStepMs : kCloseStepMs;
  if (currentAngle != targetAngle && now - lastStepMs >= stepMs) {
    lastStepMs = now;
    currentAngle += (targetAngle > currentAngle) ? 1 : -1;
    minibot.moduleServoGoAngle(SERVO_PIN, currentAngle, 0);
  }

  delay(1);
}
