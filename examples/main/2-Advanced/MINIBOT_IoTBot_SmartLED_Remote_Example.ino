// TR: EGLENCELI KABLOSUZ ORNEK - Bu MINIBOT'un butonuna her bastiginizda,
// uzaktaki bir IOTBOT'un akilli LED serisine (NeoPixel) "bir sonraki
// efekte gec" komutu gonderilir. Hicbir kablo yok - sadece ESP-NOW. Once
// IOTBOT_MiniBot_SmartLED_Remote_Example.ino dosyasini bir IOTBOT'a,
// sonra bu kodu bir MINIBOT'a yukleyin.
// EN: A FUN WIRELESS EXAMPLE - every time you press this MINIBOT's
// button, a "switch to the next effect" command is sent to a remote
// IOTBOT's smart LED strip (NeoPixel). No wire - just ESP-NOW. Upload
// IOTBOT_MiniBot_SmartLED_Remote_Example.ino to an IOTBOT first, then
// upload this code to a MINIBOT.
//
// MINIBOT'ta LCD ekran YOK; tum bilgiler Seri Port (USB) uzerinden verilir.
// / MINIBOT has NO LCD screen; all feedback is given through Serial (USB).

#define USE_ESPNOW
#include <MINIBOT.h>

MINIBOT minibot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

// ONEMLI: Kendi IOTBOT'unuzun MAC adresiyle degistirin (IOTBOT_MiniBot_
// SmartLED_Remote_Example.ino Seri Port'a kendi MAC'ini yazdirir).
// IMPORTANT: Replace with your own IOTBOT's MAC address (the IOTBOT-side
// sketch prints its own MAC to Serial).
uint8_t kPeerMac[] = {0x30, 0x83, 0x98, 0x46, 0x3A, 0xB8};

namespace {
  uint8_t effectIndex = 0;

  void say(const char* turkishText, const char* englishText) {
    minibot.serialWrite(turkish ? turkishText : englishText);
  }
}

void setup() {
  minibot.begin();
  minibot.serialStart(115200);
  minibot.initESPNow();

  say("Uzaktan LED kumandasi hazir - B1'e basarak efekt degistirin!",
      "Remote LED control ready - press B1 to change the effect!");
}

void loop() {
  static bool lastPressed = false;
  bool pressed = !minibot.button1Read(); // button1Read() aktif-LOW / active-LOW

  if (pressed && !lastPressed) {
    effectIndex = (effectIndex + 1) % 4;

    CodlaiESPNowMessage outgoing;
    outgoing.deviceType = 20; // 20 = MINIBOT
    outgoing.axis1 = 0;
    outgoing.axis2 = 0;
    outgoing.axis3 = 0;
    outgoing.gripper = 0;
    outgoing.action = effectIndex;
    minibot.sendESPNow(kPeerMac, (uint8_t *)&outgoing, sizeof(outgoing));

    Serial.print(turkish ? "Efekt komutu gonderildi: " : "Effect command sent: ");
    Serial.println(effectIndex);
    minibot.ledWrite(true);
    delay(80);
    minibot.ledWrite(false);
  }
  lastPressed = pressed;
}
