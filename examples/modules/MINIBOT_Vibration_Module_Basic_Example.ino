#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

// MINIBOT nesnesi oluşturuluyor / Create a MINIBOT object
MINIBOT minibot;

#define SENSOR_PIN IO12 // Titreşim sensörünün bağlı olduğu pini seçin / Select the pin connected to the vibration sensor
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

    minibot.serialWrite("Welcome to MiniBot Vibration Test Firmware!");
    // Hoşgeldiniz mesajını seri porta yazdır / Display welcome message on the serial port
}

void loop()
{
    bool vibrationStatus = minibot.moduleVibrationDigitalRead(SENSOR_PIN);
    // Titreşim sensöründen veri okunuyor / Read vibration status from the sensor

    if (vibrationStatus)
    {
        minibot.serialWrite("Titresim Algilandi! / Vibration Detected!");
        // Eğer titreşim algılanırsa seri porta mesaj yazdır / Print message if vibration is detected
    }

    delay(100);
    // Yeni okuma yapmadan önce 100 ms bekle / Wait for 100 ms before reading again
}
