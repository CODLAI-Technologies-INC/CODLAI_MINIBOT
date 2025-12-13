/*
 * MINIBOT IFTTT Webhook Example / MINIBOT IFTTT Webhook Örneği
 *
 * Demonstrates how to trigger IFTTT Webhooks from the ESP8266-based MiniBot.
 * Configure your WiFi credentials, IFTTT event name, and Maker Webhooks key
 * before uploading the sketch.
 *
 * ESP8266 tabanlı MiniBot'tan IFTTT Webhook çağrılarının nasıl yapılacağını
 * gösterir. WiFi bilgilerinizi, IFTTT olay adını ve Maker Webhooks anahtarını
 * girmeyi unutmayın.
 *
 * Quick setup steps / Hızlı kurulum adımları:
 * 1. https://ifttt.com/create adresinde Webhooks tetikleyicisini seçin ve Event Name alanını `iftttEventName`
 *    ile aynı yapın.
 * 2. Dilediğiniz hedef servisi seçerek appleti tamamlayın (Google Sheets, Discord vb.).
 * 3. https://ifttt.com/maker_webhooks sayfasındaki "Documentation" bölümünden key değerini kopyalayın ve
 *    `iftttWebhookKey` değişkenine yapıştırın.
 * 4. Maker Webhooks servisinde `value1/2/3` alanlarını ingredient olarak kullanabilirsiniz.
 *
 * Not: `USE_IFTTT` tanımlandığında WiFi yardımcıları otomatik olarak açılır, koda ayrıca `#define USE_WIFI`
 * eklemeniz gerekmez.
 */

#define USE_IFTTT
#include "MINIBOT.h"

MINIBOT minibot;

const char *ssid = "YOUR_WIFI_SSID";
const char *password = "YOUR_WIFI_PASSWORD";

String iftttEventName = "YOUR_EVENT_NAME";   // Örnek: "minibot_button"
String iftttWebhookKey = "YOUR_IFTTT_KEY";   // https://ifttt.com/maker_webhooks

unsigned long lastTrigger = 0;
const unsigned long triggerInterval = 4000; // 4 seconds debounce window

void setup() {
  minibot.serialStart(115200);
  minibot.begin();

  Serial.println("MINIBOT IFTTT Webhook Example");
  Serial.println("Connecting to WiFi...");
  minibot.wifiStartAndConnect(ssid, password);

  if (minibot.wifiConnectionControl()) {
    Serial.println("WiFi connected. Sending boot event to IFTTT...");
    String bootPayload = "{\"value1\":\"MINIBOT\",\"value2\":\"Boot\",\"value3\":\"Online\"}";
    bool ok = minibot.triggerIFTTTEvent(iftttEventName, iftttWebhookKey, bootPayload);
    Serial.println(ok ? "[IFTTT] Boot event delivered" : "[IFTTT] Boot event failed");
  } else {
    Serial.println("WiFi connection failed. Check SSID/PASS.");
  }
}

void loop() {
  bool buttonPressed = minibot.button1Read();
  unsigned long now = millis();

  if (buttonPressed && (now - lastTrigger) > triggerInterval) {
    lastTrigger = now;

    // Optional: flash the LED for user feedback.
    minibot.ledWrite(true);

    String payload = "{\"value1\":\"Button 1\",\"value2\":\"Pressed\",\"value3\":\"Millis:";
    payload += String(now);
    payload += "\"}";

    bool ok = minibot.triggerIFTTTEvent(iftttEventName, iftttWebhookKey, payload);
    Serial.println(ok ? "[IFTTT] Button event delivered" : "[IFTTT] Button event failed");

    minibot.ledWrite(false);
  }

  delay(50);
}
