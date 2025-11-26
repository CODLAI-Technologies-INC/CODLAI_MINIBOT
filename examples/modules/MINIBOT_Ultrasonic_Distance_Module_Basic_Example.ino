#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

// MINIBOT nesnesi oluşturuluyor / Create a MINIBOT object
MINIBOT minibot;

#define SENSOR_PIN IO12 // Ultrasonik sensörün bağlı olduğu pini seçin / Select the pin connected to the ultrasonic sensor
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

    minibot.serialWrite("Welcome to MiniBot Ultrasonic Distance Test Firmware!");
    // Hoşgeldiniz mesajını seri porta yazdır / Display welcome message on the serial port
}

void loop()
{
    int distance = minibot.moduleUltrasonicDistanceRead();
    // Ultrasonik sensör ile mesafe ölçülüyor / Measure distance using the ultrasonic sensor

    minibot.serialWrite("Mesafe / Distance: ");
    minibot.serialWrite(distance);
    // Ölçülen mesafeyi seri porta yazdır / Print measured distance to the serial port

    delay(1000);
    // Yeni okuma yapmadan önce 1 saniye bekle / Wait for 1 second before reading again
}
