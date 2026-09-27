// TR: GERCEK PROJE - Guvenlik Alarmi. PIR hareket sensoru bir hareket
// algiladiginda: onboard buzzer surekli alarm sesi calar ve mavi LED
// yanip soner. Buton 1'e basarak alarmi susturabilirsiniz - tipki
// gercek bir alarm sisteminin "iptal" tusu gibi.
// EN: A REAL PROJECT - Security Alarm. When the PIR motion sensor
// detects movement: the onboard buzzer sounds a continuous alarm and the
// blue LED flashes. Press Button 1 to silence the alarm - just like the
// "cancel" button on a real alarm system.
//
// Baglanti / Wiring: PIR sensorunu P soketlerinden BIRINE takin ve
// asagidaki PIR_PIN degerini o soketin sinyaline gore ayarlayin. / Plug
// the PIR sensor into ONE of the P sockets and set PIR_PIN below to
// match that socket's signal.

#include <MINIBOT.h>

MINIBOT minibot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define PIR_PIN IO12 // PIR sensorunun bagli oldugu pin / Pin the PIR sensor is connected to
// Desteklenen pinler: IO4 - IO5 - IO12 - IO13 - IO14
// Supported pins: IO4 - IO5 - IO12 - IO13 - IO14

namespace {
  bool alarmActive = false;
  bool ledState = false;
  uint32_t lastBeepMs = 0;
}

void setup() {
  minibot.begin();
  minibot.serialStart(115200);
  minibot.ledWrite(false);
  minibot.serialWrite(turkish ? "Guvenlik sistemi aktif." : "Security system armed.");
}

void loop() {
  bool motionDetected = minibot.moduleMotionRead(PIR_PIN);

  if (motionDetected && !alarmActive) {
    alarmActive = true;
    minibot.serialWrite(turkish ? "ALARM: hareket algilandi! Susturmak icin B1'e basin."
                                : "ALARM: motion detected! Press B1 to silence.");
  }

  if (alarmActive) {
    if (!minibot.button1Read()) {
      // Alarmi sustur / silence the alarm
      alarmActive = false;
      minibot.ledWrite(false);
      minibot.serialWrite(turkish ? "Alarm susturuldu. Sistem aktif." : "Alarm silenced. System armed.");
      delay(500); // Buton birakilana kadar bekle / debounce
    } else if (millis() - lastBeepMs >= 300) {
      lastBeepMs = millis();
      ledState = !ledState;
      minibot.ledWrite(ledState);
      minibot.buzzerPlay(2000, 150);
    }
  }

  delay(50);
}
