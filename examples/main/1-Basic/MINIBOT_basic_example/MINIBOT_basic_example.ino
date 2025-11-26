#include <MINIBOT.h> // MINIBOT kutuphanesi / MINIBOT library

// MINIBOT nesnesi olusturuluyor / Create a MINIBOT object
MINIBOT minibot;

void setup()
{
  minibot.begin(); // MINIBOT baslatiliyor / Initialize MINIBOT

  minibot.playIntro(); // Giris mesaji caliniyor / Play startup message

  minibot.serialStart(115200); // Seri haberlesmeyi baslat / Start serial communication
  // Bilgisayar ile seri haberlesme icin 115200 baud hizinda baslatilir.
  // Starts serial communication at 115200 baud for computer connection.

  minibot.serialWrite("Welcome to MiniBot Test Firmware!");
  // Hosgeldiniz mesajini seri porta yazdir / Display welcome message on the serial port
}

void loop()
{
  // 1️⃣ **Buton Durumunu Oku / Read Button State**
  if (minibot.button1Read() == false) // Eger butona basildiysa / If button is pressed
  {
    minibot.serialWrite("Button Pressed!");
    // Butona basildigini seri porta yazdir / Print button pressed message to serial port

    minibot.ledWrite(HIGH);
    // LED'i ac / Turn on LED

    delay(200); // 200 ms bekle / Wait for 200 ms
  }
  else // Eger buton serbestse / If button is released
  {
    minibot.serialWrite("Button Free");
    // Butonun serbest oldugunu seri porta yazdir / Print button released message to serial port

    minibot.ledWrite(LOW);
    // LED'i kapat / Turn off LED

    delay(200); // 200 ms bekle / Wait for 200 ms
  }
}
