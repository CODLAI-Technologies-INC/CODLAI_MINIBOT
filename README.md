# MINIBOT Library User Guide
This library is specially designed and produced by the CODLAI developer team to control the MINIBOT product.
![alt text](<images/1.png>)

## Using with Arduino IDE

### Installation

1. Open the Arduino IDE.
2. Go to "Sketch" -> "Include Library" -> "Manage Libraries..." from the menu bar.
3. Type "MINIBOT" in the search box.
4. Find the MINIBOT library and click the "Install" button to complete the installation.

## About MiniBot

MiniBot is a development board that includes two parallel communication ports, a programmable button, and an LED light, allowing it to be used with IoTBot and other modules. You can power the device via the Type-C input and program it using C++ & MicroPython through the CODLAI Editor. Its built-in Wi-Fi connectivity enables the creation of internet-based applications. Wi-Fi can be used in remote control mode or to establish a local network.

You can connect CODLAI sensor modules to the two parallel ports and develop various local or network-based applications. Additionally, MiniBot can function as the main controller for mechanical robots like ArmBot and CarBot, creating Wi-Fi networks to control them remotely. The applications you can create are only limited by your imagination!

### Processor:
- ESP8266EX - 160 MHz Tensilica L106 32-bit RISC microcontroller

### Wireless Connectivity:
- 2.4 GHz Wi-Fi

### Programming Languages:
- C++, MicroPython (Text and Block-based)

### Built-in Sensors:
- Digital Button, Digital LED

### Input/Output Interfaces:
- 1 x Type-C Socket (Programming + Power)
- 2 x RJ45-Type Module Ports (Digital)

### Additional Features:
- Remote control of supported smart devices via Wi-Fi
- Reset button
- CODLAI editor support
- CODLAI platform compatibility
- On-device information sections
- Programmability with open-source editors
- Documentation for IoT application development

### Security:
- AES and SSL/TLS hardware accelerators

### Certifications:
- CE, ROSH, EMC

### Dependencies & Versions
- **Platform:** Espressif 8266 (ESP8266)
- **Framework:** Arduino
- **Libraries:**
  - Adafruit NeoPixel
  - Adafruit Unified Sensor
  - DHT sensor library
  - IRremoteESP8266
  - ESPAsyncWebServer
  - Firebase Arduino Client Library for ESP8266 and ESP32
  - ArduinoJson
- **Offline Libraries:**
  - `other_libraries.zip`: Contains all required library dependencies for offline installation.

# MINIBOT Kütüphanesi Kullanım Kılavuzu
Bu kütüphane CODLAI geliştirici ekibi tarafından MINIBOT ürününü kontrol etmek için özel olarak tasarlanmış ve üretilmiştir.
![alt text](<images/1.png>)

## Arduino IDE ile Kullanım

### Kurulum

1. Arduino IDE'yi açın.
2. Menü çubuğundan "Sketch" -> "Include Library" -> "Manage Libraries..." seçeneğine gidin.
3. Arama kutusuna "MINIBOT" yazın.
4. MINIBOT kütüphanesini bulun ve "Install" düğmesine tıklayarak kurulumu tamamlayın.

## MiniBot Hakkında

MiniBot, IoTBot ve diğer modüllerle birlikte kullanabilmek için üzerinde iki adet paralel iletişim portu, programlanabilir buton ve LED ışık barındıran bir geliştirme kartıdır. Type-C girişi ile cihaza gücü verebilir ve CODLAI Editör üzerinden C++ & MicroPython dillerinde programlama yapabilirsiniz. Sahip olduğu dahili Wi-Fi bağlantısı ile internet tabanlı uygulamalar yapmaya imkan sağlar. Wi-Fi bağlantısı ister uzaktan kontrol modunda istenirse de yerelde ağ oluşturarak kullanılabilir.

İki adet paralel portuna CODLAI sensör modülleri bağlayabilir ve ister yerel istenirse ağlar arası birçok farklı uygulamalar geliştirebilirsiniz. Bunun yanı sıra; ArmBot ve CarBot gibi diğer mekanik robotların ana bilgisayarı olarak kullanabilir, bu gibi robotları uzaktan kontrol etmek için Wi-Fi ağları oluşturabilirsiniz. Yapılacak uygulamalar tamamen hayal gücü ile sınırlıdır!

### İşlemci:
- ESP8266EX - 160 MHz Tensilica L106 32-bit RISC mikrodenetleyici

### Kablosuz Bağlantı:
- 2.4 GHz Wi-Fi

### Programlama Dilleri:
- C++, MicroPython (Metin ve Blok Tabanlı)

### Dahili Sensörler:
- Dijital Buton, Dijital LED

### Giriş/Çıkış Arabirimleri:
- 1 x Type-C Soket (Programlama + Güc)
- 2 x RJ45 Tip Modül Portu (Dijital)

### Ek Özellikler:
- Desteklenen akıllı cihazları Wi-Fi ile uzaktan kontrol etme
- Reset tuşu
- CODLAI editör desteği
- CODLAI platform desteği
- Cihaz üzerinde bilgilendirme bölümleri
- Açık kaynaklı editörler ile programlayabilme
- IoT uygulamalar yapmak için gerekli dokümantasyon

### Güvenlik:
- AES ve SSL/TLS için donanım hızlandırıcıları

### Sertifikalar:
- CE, ROSH, EMC

# Library Structure & Contributing / Kütüphane Yapısı ve Katkıda Bulunma
This library follows a modular design pattern to ensure efficiency.
- **Configuration:** Use `MINIBOT_Config.h` to enable/disable specific modules (e.g., WiFi, Firebase).
- **Extension:** To add new features, define a new flag in the config file and wrap your code in `#if defined(...)` blocks.

Bu kütüphane verimliliği sağlamak için modüler bir tasarım deseni izler.
- **Yapılandırma:** Belirli modülleri (örn. WiFi, Firebase) etkinleştirmek/devre dışı bırakmak için `MINIBOT_Config.h` dosyasını kullanın.
- **Genişletme:** Yeni özellikler eklemek için yapılandırma dosyasında yeni bir bayrak tanımlayın ve kodunuzu `#if defined(...)` blokları içine alın.

# Katkıda Bulunma / Contributing
Katkıda bulunmak isterseniz, lütfen GitHub deposuna Pull Request gönderin. / If you'd like to contribute, please send a Pull Request to the GitHub repository.

# Lisans / License
Bu kütüphane 2024 Yılında Samed KAYA tarafından lisanslanmıştır. Detaylar için LICENSE dosyasına bakınız. / Copyright (c) 2024 Samed KAYA. All right reserved.

See the LICENSE file for details.

![alt text](<images/2.png>)
![alt text](<images/3.png>)
![alt text](<images/4.png>)
![alt text](<images/5.png>)

