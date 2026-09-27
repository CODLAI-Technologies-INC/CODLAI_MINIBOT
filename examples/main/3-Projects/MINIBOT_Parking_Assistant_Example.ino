// TR: GERCEK PROJE - Park Sensoru. Ultrasonik mesafe sensoru bir cisme
// (ornegin bir duvara) yaklastikca onboard buzzer GIDEREK HIZLANAN bir
// "bip" sesi cikartir - tipki arabalardaki park sensoru gibi. Cok
// yaklasinca (10cm alti) ses surekli/sabit hale gelir ve mavi LED yanar.
// EN: A REAL PROJECT - Parking Sensor. As the ultrasonic distance sensor
// gets closer to an object (e.g. a wall), the onboard buzzer beeps
// FASTER AND FASTER - just like a real car's parking sensor. When very
// close (under 10cm) the sound becomes constant and the blue LED turns on.
//
// Baglanti / Wiring: Ultrasonik sensoru herhangi bir P soketine takmaniza
// GEREK YOK - bu modul sabit pinler kullanir. / You do NOT need to plug
// the ultrasonic sensor into any P socket - this module uses fixed pins.

#include <MINIBOT.h>

MINIBOT minibot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

namespace {
  constexpr int kStopDistanceCm = 10;       // Bu mesafenin altinda surekli ses / below this: constant tone
  constexpr int kMaxUsefulDistanceCm = 100; // Bu mesafenin ustunde sessiz / above this: silent
}

void setup() {
  minibot.begin();
  minibot.serialStart(115200);
  minibot.ledWrite(false);
  minibot.serialWrite(turkish ? "Park sensoru hazir." : "Parking sensor ready.");
}

void loop() {
  int distance = minibot.moduleUltrasonicDistanceRead();
  bool valid = distance > 0 && distance < 400;

  if (!valid || distance > kMaxUsefulDistanceCm) {
    // Cok uzak ya da okuma gecersiz - sessiz / too far or invalid reading - stay quiet
    minibot.ledWrite(false);
    delay(200);
    return;
  }

  if (distance <= kStopDistanceCm) {
    // Cok yakin: surekli ses + LED yak / very close: continuous beep + LED on
    minibot.ledWrite(true);
    minibot.serialWrite(turkish ? ("DUR! " + String(distance) + "cm") : ("STOP! " + String(distance) + "cm"));
    minibot.buzzerPlay(1800, 300);
  } else {
    // Mesafeye gore bip hizini ayarla: yaklastikca daha sik bip
    // beep rate scales with distance: closer = faster beeping
    minibot.ledWrite(false);
    minibot.serialWrite(turkish ? ("Mesafe: " + String(distance) + "cm") : ("Distance: " + String(distance) + "cm"));
    int beepGapMs = map(distance, kStopDistanceCm, kMaxUsefulDistanceCm, 60, 600);
    minibot.buzzerPlay(1800, 60);
    delay(beepGapMs);
    return;
  }
  delay(100);
}
