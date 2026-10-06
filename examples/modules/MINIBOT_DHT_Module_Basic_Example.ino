/*
 * 📌 DHT Sensör özelliklerini kullanabilmek için, MINIBOT_Config.h dosyasında 'USE_DHT' tanımının başındaki
 * yorum satırlarını (//) kaldırın. Aksi takdirde, sensör işlevleri devre dışı kalacaktır.
 *
 * 📌 To enable DHT Sensor  features, remove the comment (//) before the 'USE_DHT' definition in
 * MINIBOT_Config.h. Otherwise, sensor functions will be disabled.
 */

#define USE_DHT
#include <MINIBOT.h> // MINIBOT kutuphanesi / MINIBOT library

// MINIBOT nesnesi olusturuluyor / Create a MINIBOT object
MINIBOT minibot;

#define SENSOR_PIN IO12 // Sensorun bagli oldugu pini secin / Select the sensor pin
// Desteklenen pinler: IO4 - IO5 - IO12 - IO13 - IO14
// Supported pins: IO4 - IO5 - IO12 - IO13 - IO14

void setup()
{
    minibot.begin(); // MINIBOT baslatiliyor / Initialize MINIBOT

    minibot.playIntro(); // Giris muzigi caliniyor / Play startup melody
    // MiniBot acildiginda kisa bir melodi calar.
    // MiniBot plays a short melody when powered on.

    minibot.serialStart(115200); // Seri haberlesmeyi baslat / Start serial communication
    // Bilgisayar ile seri haberlesme icin 115200 baud hizinda baslatilir.
    // Starts serial communication at 115200 baud for computer connection.

    minibot.serialWrite("Welcome to MiniBot Test Firmware!");
    // Hosgeldiniz mesajini seri porta yazdir / Display welcome message on the serial port
}

void loop()
{
    int temperature = minibot.moduleDhtTempReadC(SENSOR_PIN);
    // Sicaklik degeri okunuyor (°C) / Read temperature value (°C)

    int humidity = minibot.moduleDhtHumRead(SENSOR_PIN);
    // Nem degeri okunuyor (%) / Read humidity value (%)

    minibot.serialWrite("Sicaklik / Temperature: ");
    minibot.serialWrite(temperature);
    // Sicaklik seri porta yazdiriliyor / Print temperature to serial port

    minibot.serialWrite("Nem / Humidity: ");
    minibot.serialWrite(humidity);
    // Nem seri porta yazdiriliyor / Print humidity to serial port

    delay(2000);
    // Yeni veri okumadan once 2 saniye bekle / Wait for 2 seconds before reading new data
}
