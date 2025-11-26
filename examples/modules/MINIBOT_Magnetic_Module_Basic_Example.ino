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

    minibot.serialWrite("Welcome to MiniBot Magnetic Test Firmware!");
    // Hoşgeldiniz mesajını seri porta yazdır / Display welcome message on the serial port
}

void loop()
{
    bool magneticStatus = minibot.moduleMagneticRead(SENSOR_PIN);
    // Manyetik sensör durumu okunuyor / Read magnetic sensor status

    minibot.serialWrite("Manyetik Sensör Durumu / Magnetic Sensor Status: ");
    minibot.serialWrite(magneticStatus ? "Tespit Edildi / Detected" : "Yok / Not Detected");
    // Manyetik alan algılandıysa "Tespit Edildi", aksi halde "Yok" yazdır
    // Print "Detected" if magnetic field is detected, otherwise print "Not Detected"

    delay(500);
    // Yeni okuma yapmadan önce 500 ms bekle / Wait for 500 ms before reading again
}
