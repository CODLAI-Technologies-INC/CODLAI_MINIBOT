# CODLAI_MINIBOT Library Documentation / Kütüphane Dokümantasyonu

**EN:** `MINIBOT` is a versatile robot based on ESP8266 with various sensor capabilities.
**TR:** `MINIBOT`, ESP8266 tabanlı, çeşitli sensör yeteneklerine sahip çok yönlü bir robottur.

### Basic Control / Temel Kontrol
*   `void begin()`
    *   **EN:** Initializes basic pins.
    *   **TR:** Temel pinleri başlatır.
*   `void playIntro()`
    *   **EN:** Performs startup by blinking the LED.
    *   **TR:** LED'i yakıp söndürerek açılış yapar.
*   `bool button1Read()`
    *   **EN:** Reads the onboard button.
    *   **TR:** Üzerindeki butonu okur.
*   `void ledWrite(bool status)`
    *   **EN:** Controls the blue LED.
    *   **TR:** Mavi LED'i kontrol eder.
*   `void buzzerPlay(int frequency, int duration)`
    *   **EN:** Plays the buzzer.
    *   **TR:** Buzzer çalar.

### Sensors and Actuators / Sensörler ve Eyleyiciler
*   **Servo**: `void moduleServoGoAngle(int pin, int angle, int acceleration)`
*   **DHT**: `moduleDhtTempReadC`, `moduleDhtTempReadF`, `moduleDhtHumRead`, `moduleDthFeelingTempC/F`
*   **Distance / Mesafe**: `int moduleUltrasonicDistanceRead()`
*   **Motion / Hareket**: `bool moduleMotionRead(int pin)`
*   **Magnetic / Manyetik**: `bool moduleMagneticRead(int pin)`
*   **Vibration / Titreşim**: `bool moduleVibrationDigitalRead(int pin)`
*   **IR Receiver / IR Alıcı**: `moduleIRReadHex`, `moduleIRReadDecimalx32`, `moduleIRReadDecimalx8`
*   **Relay / Röle**: `void moduleRelayWrite(int pin, bool status)`
*   **Traffic Light / Trafik Işığı**: `moduleTraficLightWrite`, `moduleTraficLightWriteRed/Yellow/Green`
*   **Smart LED / Akıllı LED**: `moduleSmartLEDPrepare`, `moduleSmartLEDWrite`, `extendSmartLEDPrepare`, `extendSmartLEDFill`, and all effect functions (`Rainbow`, `TheaterChase`, `ColorWipe`).

### General Pin & EEPROM / Genel Pin ve EEPROM
*   `int digitalReadPin(int pin)`
    *   **EN:** Digital read.
    *   **TR:** Dijital okuma.
*   `void digitalWritePin(int pin, bool value)`
    *   **EN:** Digital write.
    *   **TR:** Dijital yazma.
*   `void eepromWriteInt(int address, int value)`
    *   **EN:** EEPROM write.
    *   **TR:** EEPROM yazma.
*   `int eepromReadInt(int address)`
    *   **EN:** EEPROM read.
    *   **TR:** EEPROM okuma.

### Communication / İletişim
*   `void serialStart(int baudrate)`
    *   **EN:** Start serial port.
    *   **TR:** Seri port başlatma.
*   `void serialWrite(...)`
    *   **EN:** Write to serial port.
    *   **TR:** Seri port yazma.
*   **WiFi**: `wifiStartAndConnect`, `wifiConnectionControl`, `wifiGetIPAddress`, `wifiGetMACAddress`.
*   **ESP-NOW**: `initESPNow`, `setWiFiChannel`, `sendESPNow`, `registerOnRecv`, `startListening`.
*   **Server / Sunucu**: `serverStart`, `serverCreateLocalPage`, `serverHandleDNS`, `serverContinue`.
*   **Cloud / Bulut**: `fbServerSetandStartWithUser` (Firebase), `sendTelegram`, `sendEmail`, `getWeather`, `getWikipedia`.
