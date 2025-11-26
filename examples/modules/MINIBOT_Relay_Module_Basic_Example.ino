#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library

// MINIBOT nesnesi oluşturuluyor / Create a MINIBOT object
MINIBOT minibot;

#define SENSOR_PIN IO12 // Rölenin bağlı olduğu pini seçin / Select the pin connected to the relay
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

    minibot.serialWrite("Welcome to MiniBot Relay Test Firmware!");
    // Hoşgeldiniz mesajını seri porta yazdır / Display welcome message on the serial port
}

void loop()
{
    // Röleyi aktif hale getir (ON) / Activate the relay (ON)
    minibot.moduleRelayWrite(SENSOR_PIN, true);
    minibot.serialWrite("Relay ON - Röle Devrede!");
    // Seri porta rölenin açıldığını bildir / Print relay activation message
    delay(3000); // 3 saniye bekle / Wait for 3 seconds

    // Röleyi pasif hale getir (OFF) / Deactivate the relay (OFF)
    minibot.moduleRelayWrite(SENSOR_PIN, false);
    minibot.serialWrite("Relay OFF - Röle Devre Dışı!");
    // Seri porta rölenin kapandığını bildir / Print relay deactivation message
    delay(3000); // 3 saniye bekle / Wait for 3 seconds
}
