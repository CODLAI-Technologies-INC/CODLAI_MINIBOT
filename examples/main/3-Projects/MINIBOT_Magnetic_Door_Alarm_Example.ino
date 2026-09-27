// TR: GERCEK PROJE - Kapi/Pencere Alarmi. Manyetik sensorun iki parcasini
// (mikanat + sensor) bir kapinin/pencerenin acilan ve sabit tarafina
// yerlestirin. Sistem "kurulu" (armed) haldeyken kapi acilirsa (manyetik
// baglanti koparsa) onboard buzzer alarm calar ve mavi LED yanip soner.
// Buton 1'e basarak sistemi kurma (arm) / etkisizlestirme (disarm)
// arasinda gecis yapabilirsiniz - tipki gercek bir ev alarmi gibi.
// EN: A REAL PROJECT - Door/Window Alarm. Place the magnetic sensor's two
// parts (magnet + sensor) on a door/window's moving and fixed sides.
// While the system is "armed", opening the door (breaking the magnetic
// contact) makes the onboard buzzer sound an alarm and the blue LED
// flash. Press Button 1 to switch the system between armed/disarmed -
// just like a real home alarm system.
//
// Baglanti / Wiring: Manyetik sensoru P soketlerinden BIRINE takin ve
// asagidaki MAGNETIC_PIN degerini o soketin sinyaline gore ayarlayin. /
// Plug the magnetic sensor into ONE of the P sockets and set
// MAGNETIC_PIN below to match that socket's signal.

#include <MINIBOT.h>

MINIBOT minibot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define MAGNETIC_PIN IO12 // Manyetik sensorun bagli oldugu pin / Pin the magnetic sensor is connected to
// Desteklenen pinler: IO4 - IO5 - IO12 - IO13 - IO14
// Supported pins: IO4 - IO5 - IO12 - IO13 - IO14

namespace {
  bool armed = false;
  bool alarmActive = false;
  bool ledState = false;
  uint32_t lastBeepMs = 0;
  bool lastButtonState = true; // digitalRead: HIGH = birakilmis / released
}

void setup() {
  minibot.begin();
  minibot.serialStart(115200);
  minibot.ledWrite(false);
  minibot.serialWrite(turkish ? "Kapi alarmi hazir. Kurmak icin B1'e basin."
                              : "Door alarm ready. Press B1 to arm.");
}

void loop() {
  bool buttonState = minibot.button1Read(); // false = basili / pressed

  // Sadece basma anini yakala (kenar algila) / edge-detect the press only
  if (lastButtonState == true && buttonState == false) {
    armed = !armed;
    alarmActive = false;
    minibot.ledWrite(false);
    if (armed) {
      minibot.serialWrite(turkish ? "Sistem KURULDU." : "System ARMED.");
      minibot.buzzerPlay(1500, 100);
    } else {
      minibot.serialWrite(turkish ? "Sistem ETKISIZLESTIRILDI." : "System DISARMED.");
      minibot.buzzerPlay(800, 200);
    }
    delay(300); // debounce
  }
  lastButtonState = buttonState;

  if (armed) {
    bool doorClosed = minibot.moduleMagneticRead(MAGNETIC_PIN);
    if (!doorClosed && !alarmActive) {
      alarmActive = true;
      minibot.serialWrite(turkish ? "ALARM: kapi/pencere acildi!" : "ALARM: door/window opened!");
    }
  }

  if (alarmActive) {
    if (millis() - lastBeepMs >= 250) {
      lastBeepMs = millis();
      ledState = !ledState;
      minibot.ledWrite(ledState);
      minibot.buzzerPlay(2200, 120);
    }
  }

  delay(30);
}
