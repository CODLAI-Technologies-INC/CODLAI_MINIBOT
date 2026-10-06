// TR: GERCEK PROJE - Kablosuz Servo Alicisi. Bu MINIBOT, IOTBOT kumanda
// panelinin ESP-NOW ile yayinladigi komutlari dinler: "servo" gelince Port
// B'deki servo motoru o aciya YUMUSAKCA dondurur, "led" gelince mavi LED'i
// acar/kapatir. Aldigi her yeni komutu Seri Port'a yazar. Once IOTBOT'a
// IOTBOT_ESPNOW_Remote_Control_Panel_Example.ino dosyasini yukleyin; istersen
// bir ROLEBOT'a da ROLEBOT_ESPNOW_Remote_Relay_Receiver_Example.ino dosyasini
// yukleyip rolelerini ayni panelden yonetin. Potansiyometreyi cevirin!
// EN: A REAL PROJECT - Wireless Servo Receiver. This MINIBOT listens to the
// commands the IOTBOT control panel broadcasts over ESP-NOW: on "servo" it
// turns the servo on Port B SMOOTHLY to that angle, on "led" it switches the
// blue LED on/off. It prints every new command to Serial. First upload
// IOTBOT_ESPNOW_Remote_Control_Panel_Example.ino to an IOTBOT; optionally
// upload ROLEBOT_ESPNOW_Remote_Relay_Receiver_Example.ino to a ROLEBOT and
// control its relays from the same panel. Turn the potentiometer!
//
// Baglanti / Wiring: Servo motoru Port B'ye (IO4) takin. / Plug the servo
// motor into Port B (IO4).

#define USE_ESPNOW
#define USE_SERVO
#include <MINIBOT.h>

MINIBOT minibot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define SERVO_PIN IO4 // Port B

namespace {
  constexpr int kEspNowChannel = 1;     // IOTBOT ile AYNI kanal / SAME channel as the IOTBOT
  constexpr uint32_t kStepEveryMs = 8;  // Her 8 ms'de 1 derece (yumusak hareket) / 1 degree every 8 ms (smooth motion)

  int targetAngle = 90;
  int currentAngle = 90;
  uint32_t lastStepMs = 0;
  int ledState = -1;                    // -1 = henuz komut yok / no command yet

  void say(const String &turkishText, const String &englishText) {
    minibot.serialWrite(turkish ? turkishText : englishText);
  }
}

void setup() {
  minibot.begin();
  minibot.serialStart(115200);
  minibot.espNowBegin(kEspNowChannel);
  minibot.moduleServoGoAngle(SERVO_PIN, currentAngle, 0); // 0 = hemen git (bekleme yok) / 0 = go instantly (no waiting)
  minibot.ledWrite(false);
  say("Servo alicisi hazir - IOTBOT panelinden komut bekleniyor...",
      "Servo receiver ready - waiting for commands from the IOTBOT panel...");
}

void loop() {
  // 1) Gelen mesaj. NOT: espNowReadName() mesaji "okundu" isaretler; bu yuzden
  // sayiyi ONCE receivedData.value'dan aliyoruz, adi SONRA okuyoruz.
  // 1) Incoming message. NOTE: espNowReadName() marks the message as read, so
  // we take the number FIRST from receivedData.value and read the name AFTER.
  if (minibot.espNowAvailable()) {
    minibot.espNowReadText();                    // Metin mesajiysa at / drop it if it is a text message
    float value = minibot.receivedData.value;
    String name = minibot.espNowReadName();

    if (name == "servo") {
      int angle = constrain((int)value, 0, 180);
      if (angle != targetAngle) {                // Panel ayni degeri tekrarlar - sadece degisince yaz / the panel repeats values - print only on change
        targetAngle = angle;
        say("Servo hedefi: " + String(angle) + " derece", "Servo target: " + String(angle) + " degrees");
      }
    } else if (name == "led") {
      int on = value > 0.5f ? 1 : 0;
      if (on != ledState) {
        ledState = on;
        minibot.ledWrite(on);
        say(on ? "LED ACIK" : "LED KAPALI", on ? "LED ON" : "LED OFF");
      }
    }
    // "role1"/"role2" ROLEBOT icindir, burada yok sayilir. / "role1"/"role2" are for the ROLEBOT and are ignored here.
  }

  // 2) Servoyu hedefe dogru her seferinde 1 derece yaklastir - loop() hic
  // durmadan (moduleServoGoAngle'in yavas modu loop'u bekletirdi).
  // 2) Move the servo 1 degree toward the target each step - loop() never
  // stops (moduleServoGoAngle's slow mode would block the loop).
  if (currentAngle != targetAngle && millis() - lastStepMs >= kStepEveryMs) {
    lastStepMs = millis();
    currentAngle += (targetAngle > currentAngle) ? 1 : -1;
    minibot.moduleServoGoAngle(SERVO_PIN, currentAngle, 0);
  }

  delay(1);
}
