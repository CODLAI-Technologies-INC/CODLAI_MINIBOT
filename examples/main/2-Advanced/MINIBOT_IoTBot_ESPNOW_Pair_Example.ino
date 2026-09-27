// TR: MINIBOT'u, ayni odadaki bir IOTBOT ile ROUTER/WIFI AGI OLMADAN
// (ESP-NOW ile) dogrudan haberlestirir. MINIBOT kendi butonunun durumunu
// ve artan bir sayaci IOTBOT'a gonderir; IOTBOT'un potansiyometre degerini
// geri alip kendi LED'ini o degere gore yanip sondurur (IOTBOT'un B3
// butonuna basiliysa LED sabit yanar).
// EN: Talks directly (peer-to-peer, no router/WiFi network needed) with an
// IOTBOT in the same room over ESP-NOW. Sends MINIBOT's own button state
// and an increasing counter to the IOTBOT; receives the IOTBOT's
// potentiometer value back and blinks its own LED at a rate based on it
// (LED stays solid on while the IOTBOT's B3 button is held).
//
// MINIBOT'ta LCD ekran YOK; tum bilgiler Seri Port (USB) uzerinden verilir.
// / MINIBOT has NO LCD screen; all feedback is given through the Serial
// (USB) monitor.
//
// Eslenecek IOTBOT'a bu klasordeki IOTBOT_MiniBot_ESPNOW_Pair_Example.ino
// dosyasini yukleyin / Upload IOTBOT_MiniBot_ESPNOW_Pair_Example.ino (in
// the CODLAI_IOTBOT library's examples) to the IOTBOT you want to pair with.
//
// ONEMLI / IMPORTANT: Asagidaki kPeerMac dizisini, GERCEK IOTBOT'unuzun
// MAC adresiyle degistirin (IOTBOT tarafindaki kod Seri Port'a kendi
// MAC'ini yazdirir). / Replace kPeerMac below with your actual IOTBOT's
// MAC address (the IOTBOT-side sketch prints its own MAC to Serial).

#define USE_ESPNOW
#include <MINIBOT.h>

MINIBOT minibot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

// Test icin kullanilan gercek bir IOTBOT'un MAC adresi - kendi kartiniza
// gore degistirin. / A real IOTBOT's MAC address used for testing -
// change this to match your own board.
uint8_t kPeerMac[] = {0x30, 0x83, 0x98, 0x46, 0x3A, 0xB8};

namespace {
  uint32_t lastSendMs = 0;
  constexpr uint32_t kSendIntervalMs = 500;
  uint32_t counter = 0;

  uint32_t lastBlinkMs = 0;
  bool ledState = false;
  int blinkIntervalMs = 500; // IOTBOT'un pot degerine gore guncellenir / updated from IOTBOT's pot value
  bool remoteButtonHeld = false;

  void say(const char* turkishText, const char* englishText) {
    minibot.serialWrite(turkish ? turkishText : englishText);
  }
}

void setup() {
  minibot.begin();
  minibot.serialStart(115200);
  say("ESP-NOW baslatiliyor...", "Starting ESP-NOW...");

  minibot.initESPNow();
  minibot.setWiFiChannel(1); // Iki taraf da AYNI kanalda olmali / Both sides must use the SAME channel.
  minibot.startListening();  // Gelen mesajlari minibot.receivedData'ya yazar / Fills minibot.receivedData on arrival.

  String myMac = WiFi.macAddress();
  say(("Benim MAC adresim: " + myMac).c_str(), ("My MAC address: " + myMac).c_str());
  say("ESP-NOW hazir, IOTBOT ile eslesme aktif.", "ESP-NOW ready, paired with IOTBOT.");
}

void loop() {
  // ---- Gonderim: buton durumu + sayac / Sending: button state + counter ----
  if (millis() - lastSendMs >= kSendIntervalMs) {
    lastSendMs = millis();
    counter++;
    CodlaiESPNowMessage outgoing;
    outgoing.deviceType = 20; // 20 = MINIBOT (bu ornekte kullanilan kimlik / id used in this example)
    outgoing.axis1 = counter;
    outgoing.axis2 = 0;
    outgoing.axis3 = 0;
    outgoing.gripper = 0;
    outgoing.action = minibot.button1Read() ? 0 : 1; // button1Read() aktif-LOW / active-LOW
    minibot.sendESPNow(kPeerMac, (uint8_t *)&outgoing, sizeof(outgoing));
  }

  // ---- Alis: IOTBOT'tan gelen veri / Receiving: data from the IOTBOT ----
  if (minibot.newData) {
    minibot.newData = false;
    int iotbotPot = minibot.receivedData.axis1;
    remoteButtonHeld = minibot.receivedData.action == 1;
    // Pot degeri (0-4095) yanip sonme araligina (900ms..80ms) esleniyor -
    // pot ne kadar yuksekse LED o kadar hizli yanip soner.
    // Pot value (0-4095) maps to a blink interval (900ms..80ms) - the
    // higher the pot, the faster the LED blinks.
    blinkIntervalMs = map(iotbotPot, 0, 4095, 900, 80);
    Serial.print(turkish ? "IOTBOT pot degeri: " : "IOTBOT pot value: ");
    Serial.println(iotbotPot);
  }

  // ---- LED geri bildirimi / LED feedback ----
  if (remoteButtonHeld) {
    minibot.ledWrite(true);
  } else if (millis() - lastBlinkMs >= (uint32_t)blinkIntervalMs) {
    lastBlinkMs = millis();
    ledState = !ledState;
    minibot.ledWrite(ledState);
  }
}
