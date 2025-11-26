/*
 * 📌 IR Sensör özelliklerini kullanabilmek için, MINIBOT_Config.h dosyasında 'USE_IR' tanımının başındaki
 * yorum satırlarını (//) kaldırın. Aksi takdirde, sensör işlevleri devre dışı kalacaktır.
 *
 * 📌 To enable IR Sensor  features, remove the comment (//) before the 'USE_IR' definition in
 * MINIBOT_Config.h. Otherwise, sensor functions will be disabled.
 */

#define USE_IR
#include <MINIBOT.h> // MINIBOT kutuphanesi / MINIBOT library

// MINIBOT nesnesi olusturuluyor / Create a MINIBOT object
MINIBOT minibot;

#define SENSOR_PIN IO12 // Sensorun bagli oldugu pini secin / Select the sensor pin
// Desteklenen pinler: IO4 - IO5 - IO12 - IO13 - IO14
// Supported pins: IO4 - IO5 - IO12 - IO13 - IO14

void setup()
{
    minibot.begin(); // MINIBOT baslat / Initialize MINIBOT

    minibot.serialStart(115200); // Seri haberlesmeyi 115200 baud hiziyla baslat / Start serial communication at 115200 baud
    // Bilgisayar ile seri haberlesme icin 115200 baud hizinda baslatilir.
    // Starts serial communication at 115200 baud for computer connection.

    minibot.serialWrite("IR Okuyucu testi baslatildi / IR Reader test started.");
    // IR sensor testinin basladigini seri porta yazdir / Print IR sensor test start message to the serial port

    delay(2000); // Baslangic gecikmesi (2 saniye) / Initial delay (2 seconds)
}

void loop()
{
    // IR sensorunden gelen Decimal verisini oku / Read Decimal data from IR sensor
    long value = minibot.moduleIRReadDecimalx32(SENSOR_PIN);
    // Alternatif olarak 8-bit formatinda deger almak icin asagidaki satiri kullanabilirsiniz:
    // Alternatively, use the following line to get an 8-bit formatted value:
    // long value = minibot.moduleIRReadDecimalx8(SENSOR_PIN);

    // Eger gecerli bir veri alindiysa (sifir degilse) / If valid data is received (not "0")
    if (value != 0)
    {
        // Seri port uzerinden Decimal kodunu gonder / Send Decimal code via serial port
        minibot.serialWrite("Decimal Code:");
        minibot.serialWrite(value);
    }
}
