// TR: GERCEK PROJE - Titresim/Darbe Alarmi ("Kutuma Dokunma!"). Titresim
// sensoru bir cismin uzerine sarsinti/darbe hissettiginde onboard buzzer
// kisa bir alarm patlamasi calar ve mavi LED yanip soner - bisiklet
// kilidi ya da kutu hirsizlik alarmi gibi dusunun. Buton 1 ile sistemi
// kurma (arm) / etkisizlestirme (disarm) arasinda gecis yaparsiniz.
// EN: A REAL PROJECT - Vibration/Shock Alarm ("Don't touch my box!").
// When the vibration sensor feels a shake/impact, the onboard buzzer
// sounds a short alarm burst and the blue LED flashes - think of a bike
// lock or an anti-theft box alarm. Press Button 1 to switch the system
// between armed/disarmed.
//
// Baglanti / Wiring: Titresim sensorunu P soketlerinden BIRINE takin ve
// asagidaki VIBRATION_PIN degerini o soketin sinyaline gore ayarlayin. /
// Plug the vibration sensor into ONE of the P sockets and set
// VIBRATION_PIN below to match that socket's signal.

#include <MINIBOT.h>

MINIBOT minibot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define VIBRATION_PIN IO12 // Titresim sensorunun bagli oldugu pin / Pin the vibration sensor is connected to
// Desteklenen pinler: IO4 - IO5 - IO12 - IO13 - IO14
// Supported pins: IO4 - IO5 - IO12 - IO13 - IO14

namespace {
  bool armed = false;
  bool lastButtonState = true; // digitalRead: HIGH = birakilmis / released
  uint32_t alarmUntilMs = 0;   // Alarmin ne zamana kadar surecegi / when the alarm should stop
  constexpr uint32_t kAlarmDurationMs = 4000; // Her darbede alarm kac ms sursun / how long each alarm burst lasts
  uint32_t lastBeepMs = 0;
  bool ledState = false;
}

void setup() {
  minibot.begin();
  minibot.serialStart(115200);
  minibot.ledWrite(false);
  minibot.serialWrite(turkish ? "Titresim alarmi hazir. Kurmak icin B1'e basin."
                              : "Vibration alarm ready. Press B1 to arm.");
}

void loop() {
  bool buttonState = minibot.button1Read(); // false = basili / pressed

  if (lastButtonState == true && buttonState == false) {
    armed = !armed;
    alarmUntilMs = 0;
    minibot.ledWrite(false);
    if (armed) {
      minibot.serialWrite(turkish ? "Sistem KURULDU. Kutuya dokunmayin!" : "System ARMED. Do not touch!");
      minibot.buzzerPlay(1500, 100);
    } else {
      minibot.serialWrite(turkish ? "Sistem ETKISIZLESTIRILDI." : "System DISARMED.");
      minibot.buzzerPlay(800, 200);
    }
    delay(300); // debounce
  }
  lastButtonState = buttonState;

  if (armed && millis() >= alarmUntilMs) {
    if (minibot.moduleVibrationDigitalRead(VIBRATION_PIN)) {
      alarmUntilMs = millis() + kAlarmDurationMs;
      minibot.serialWrite(turkish ? "ALARM: darbe/titresim algilandi!" : "ALARM: impact/vibration detected!");
    }
  }

  if (millis() < alarmUntilMs) {
    if (millis() - lastBeepMs >= 150) {
      lastBeepMs = millis();
      ledState = !ledState;
      minibot.ledWrite(ledState);
      minibot.buzzerPlay(2400, 100);
    }
  }

  delay(20);
}
