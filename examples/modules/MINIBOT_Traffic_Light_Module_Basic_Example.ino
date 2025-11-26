#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

// MINIBOT nesnesi oluşturuluyor / Create a MINIBOT object
MINIBOT minibot;

#define SENSOR_PIN IO12 // Trafik ışıklarının bağlı olduğu pini seçin / Select the pin connected to the traffic lights
// Desteklenen pinler: IO4 - IO5 - IO12 - IO13 - IO14
// Supported pins: IO4 - IO5 - IO12 - IO13 - IO14

void setup()
{
    minibot.begin(); // MINIBOT başlatılıyor / Initialize MINIBOT

    minibot.playIntro(); // Giriş müziği çalınıyor / Play startup melody
    // MiniBot açıldığında kısa bir melodi çalar.
    // MiniBot plays a short melody when powered on.

    minibot.serialStart(115200); // Seri haberleşmeyi başlat / Start serial communication
    // Bilgisayar ile seri haberleşme için 115200 baud hızında başlatılır.
    // Starts serial communication at 115200 baud for computer connection.

    minibot.serialWrite("Welcome to MiniBot Traffic Lights Test Firmware!");
    // Hoşgeldiniz mesajını seri porta yazdır / Display welcome message on the serial port
}

void loop()
{
    // 1️⃣ **Kırmızı Işık - Red Light**
    minibot.moduleTraficLightWrite(true, false, false);
    minibot.serialWrite("Kirmizi Isik - Red Light");
    // Kırmızı ışığı yak ve seri porta mesaj yazdır / Turn on red light and print message
    delay(3000); // 3 saniye bekle / Wait for 3 seconds

    // 2️⃣ **Sarı Işık - Yellow Light**
    minibot.moduleTraficLightWrite(false, true, false);
    minibot.serialWrite("Sari Isik - Yellow Light");
    // Sarı ışığı yak ve seri porta mesaj yazdır / Turn on yellow light and print message
    delay(3000); // 3 saniye bekle / Wait for 3 seconds

    // 3️⃣ **Yeşil Işık - Green Light**
    minibot.moduleTraficLightWrite(false, false, true);
    minibot.serialWrite("Yesil Isik - Green Light");
    // Yeşil ışığı yak ve seri porta mesaj yazdır / Turn on green light and print message
    delay(3000); // 3 saniye bekle / Wait for 3 seconds
}
