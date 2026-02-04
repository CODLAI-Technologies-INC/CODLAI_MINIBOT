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
*   `bool eepromBegin(size_t size = 512)`
    *   **EN:** Initializes EEPROM. Required for `EEPROM.commit()` on ESP8266.
    *   **TR:** EEPROM'u başlatır. ESP8266 üzerinde `EEPROM.commit()` için gereklidir.
*   `bool eepromCommit()`
    *   **EN:** Commits pending changes to flash.
    *   **TR:** Bekleyen değişiklikleri flash'a yazar.
*   `void eepromEnd()`
    *   **EN:** Ends EEPROM usage.
    *   **TR:** EEPROM kullanımını sonlandırır.
*   `bool eepromWriteByte(int address, uint8_t value)` / `uint8_t eepromReadByte(int address, uint8_t defaultValue = 0)`
    *   **EN:** Reads/writes a single byte.
    *   **TR:** Tek bayt okur/yazar.
*   `bool eepromWriteInt32(int address, int32_t value)` / `int32_t eepromReadInt32(int address, int32_t defaultValue = 0)`
    *   **EN:** Reads/writes 32-bit integer.
    *   **TR:** 32-bit tam sayı okur/yazar.
*   `bool eepromWriteUInt32(int address, uint32_t value)` / `uint32_t eepromReadUInt32(int address, uint32_t defaultValue = 0)`
    *   **EN:** Reads/writes 32-bit unsigned integer.
    *   **TR:** 32-bit işaretsiz tam sayı okur/yazar.
*   `bool eepromWriteFloat(int address, float value)` / `float eepromReadFloat(int address, float defaultValue = 0.0f)`
    *   **EN:** Reads/writes float.
    *   **TR:** Float okur/yazar.
*   `bool eepromWriteString(int address, const String &value, uint16_t maxLen = 128)` / `String eepromReadString(int address, uint16_t maxLen = 128)`
    *   **EN:** Stores string as `[uint16 length][bytes...]`.
    *   **TR:** String'i `[uint16 uzunluk][baytlar...]` formatında saklar.
*   `bool eepromWriteBytes(int address, const uint8_t *data, size_t len)` / `bool eepromReadBytes(int address, uint8_t *data, size_t len)`
    *   **EN:** Reads/writes raw bytes.
    *   **TR:** Ham bayt verisi okur/yazar.
*   `bool eepromClear(int startAddress = 0, size_t length = 0, uint8_t fill = 0xFF)`
    *   **EN:** Fills a region (or whole EEPROM when length=0).
    *   **TR:** Bir bölgeyi (veya length=0 ise tüm EEPROM'u) doldurur.
*   `uint32_t eepromCrc32(const uint8_t *data, size_t len, uint32_t seed = 0xFFFFFFFF)`
    *   **EN:** Calculates CRC32 for raw bytes.
    *   **TR:** Ham bayt verisi için CRC32 hesaplar.
*   `bool eepromWriteRecord(int address, const uint8_t *data, uint16_t len, uint16_t version = 1)`
    *   **EN:** Stores a CRC-protected record.
    *   **TR:** CRC korumalı record kaydeder.
*   `bool eepromReadRecord(int address, uint8_t *out, uint16_t maxLen, uint16_t *outLen = nullptr, uint16_t *outVersion = nullptr)`
    *   **EN:** Reads a CRC-protected record (validates magic/len/crc).
    *   **TR:** CRC korumalı record okur (magic/len/crc kontrolü yapar).

### Communication / İletişim
*   `void serialStart(int baudrate)`
    *   **EN:** Start serial port.
    *   **TR:** Seri port başlatma.
*   `void serialWrite(...)`
    *   **EN:** Write to serial port.
    *   **TR:** Seri port yazma.
*   **WiFi**: `wifiStartAndConnect`, `wifiConnectionControl`, `wifiGetIPAddress`, `wifiGetMACAddress`.
*   **OTA (Over-The-Air)**: `otaBegin`, `otaHandle` (call after WiFi, keep `otaHandle()` in `loop()`).
*   **NTP Time / Saat Senkron**: `ntpBegin` (recommended), `ntpSync` (advanced), `ntpIsTimeValid`, `ntpGetEpoch`, `ntpGetDateTimeString`.
*   **ESP-NOW**: `initESPNow`, `setWiFiChannel`, `sendESPNow`, `registerOnRecv`, `startListening`.
*   **Server / Sunucu**: `serverStart`, `serverCreateLocalPage`, `serverHandleDNS`, `serverContinue`.
*   **Cloud / Bulut**: `fbServerSetandStartWithUser` (Firebase), `sendTelegram`, `sendEmail`, `getWeather`, `getWikipedia`.
    *   `bool triggerIFTTTEvent(const String &eventName, const String &webhookKey, const String &jsonPayload = "{}")`
        *   **EN:** Fires an IFTTT Webhook with optional JSON data and returns `true` when HTTP 200 is received.
        *   **TR:** Opsiyonel JSON verisiyle IFTTT Webhook'unu tetikler, HTTP 200 döndüğünde `true` verir.
