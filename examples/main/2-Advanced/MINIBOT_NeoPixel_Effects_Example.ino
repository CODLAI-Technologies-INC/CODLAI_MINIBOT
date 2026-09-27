// TR: YENI NEOPIXEL EFEKTLERI. Bu ornek akilli LED (NeoPixel) icin yeni
// kolaylik fonksiyonlarini gosterir: tum seridi tek renge boyama
// (Fill), sondurme (Clear), parlaklik ayarlama (SetBrightness), yanip
// sondurme (Blink) ve "nefes alma" efekti (Breathe).
// EN: NEW NEOPIXEL EFFECTS. This example shows new convenience
// functions for the smart LED (NeoPixel): filling the whole strip with
// one color (Fill), turning it off (Clear), adjusting brightness
// (SetBrightness), blinking on/off (Blink), and a "breathing" effect
// (Breathe).
//
// Baglanti / Wiring: Akilli LED seridini P soketlerinden BIRINE takin ve
// asagidaki LED_PIN degerini o soketin sinyaline gore ayarlayin. / Plug
// the smart LED strip into ONE of the P sockets and set LED_PIN below to
// match that socket's signal.

#define USE_NEOPIXEL
#include <MINIBOT.h>

MINIBOT minibot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define LED_PIN IO12 // Akilli LED'in bagli oldugu pin / Pin the smart LED is connected to
// Desteklenen pinler: IO4 - IO5 - IO12 - IO13 - IO14
// Supported pins: IO4 - IO5 - IO12 - IO13 - IO14

void setup() {
  minibot.begin();
  minibot.serialStart(115200);
  minibot.moduleSmartLEDPrepare(LED_PIN);
  minibot.serialWrite(turkish ? "NeoPixel efektleri hazir." : "NeoPixel effects ready.");
}

void loop() {
  minibot.serialWrite(turkish ? "Fill: Kirmizi" : "Fill: Red");
  minibot.moduleSmartLEDFill(255, 0, 0);
  delay(1000);

  minibot.serialWrite(turkish ? "Fill: Yesil" : "Fill: Green");
  minibot.moduleSmartLEDFill(0, 255, 0);
  delay(1000);

  minibot.serialWrite(turkish ? "Clear: Sondu" : "Clear: Off");
  minibot.moduleSmartLEDClear();
  delay(1000);

  minibot.serialWrite(turkish ? "Parlaklik: dusuk (mavi)" : "Brightness: low (blue)");
  minibot.moduleSmartLEDSetBrightness(40);
  minibot.moduleSmartLEDFill(0, 0, 255);
  delay(1000);

  minibot.serialWrite(turkish ? "Parlaklik: yuksek (mavi)" : "Brightness: high (blue)");
  minibot.moduleSmartLEDSetBrightness(255);
  delay(1000);

  minibot.serialWrite(turkish ? "Blink: sari, 3 kez" : "Blink: yellow, 3 times");
  minibot.moduleSmartLEDBlink(255, 255, 0, 3, 200);

  minibot.serialWrite(turkish ? "Breathe: mor" : "Breathe: purple");
  minibot.moduleSmartLEDBreathe(150, 0, 255, 2000);

  minibot.moduleSmartLEDClear();
  delay(2000);
}
