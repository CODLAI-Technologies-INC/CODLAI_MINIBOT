#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

// MINIBOT nesnesi oluşturuluyor / Create a MINIBOT object
MINIBOT minibot;

#define SENSOR_PIN IO12 // Sensörün bağlı olduğu pini seçin / Select the sensor pin
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

    minibot.serialWrite("Welcome to MiniBot PIR Module Test Firmware!");
    // Hoşgeldiniz mesajını seri porta yazdır / Display welcome message on the serial port
}

void loop()
{
    // PIR sensöründen hareket algılama verisini oku / Read motion detection data from PIR sensor
    if (minibot.moduleMotionRead(SENSOR_PIN))
    {
        minibot.serialWrite("Hareket Algılandi! - Motion Detected!");
        // Hareket algılandıysa, mesajı seri porta yazdır
        // If motion is detected, print the message to the serial port
    }
    else
    {
        minibot.serialWrite("Hareket Yok! - No Motion!");
        // Hareket algılanmadıysa, mesajı seri porta yazdır
        // If no motion is detected, print the message to the serial port
    }

    delay(500); // 500 ms bekle, gereksiz tekrarları önlemek için / Wait 500 ms to prevent unnecessary repetitions
}
