# CODLAI MINIBOT Library Reference / Kütüphane Referansı

## Introduction / Giriş
**EN:** The MINIBOT library is a versatile library for ESP8266-based robots. It supports various sensors (DHT, Ultrasonic, IR, etc.), actuators (Servo, Relay, LEDs), and connectivity (WiFi, Firebase, Telegram).
**TR:** MINIBOT kütüphanesi, ESP8266 tabanlı robotlar için çok yönlü bir kütüphanedir. Çeşitli sensörleri (DHT, Ultrasonik, IR vb.), eyleyicileri (Servo, Röle, LED'ler) ve bağlantı özelliklerini (WiFi, Firebase, Telegram) destekler.

## Functions / Fonksiyonlar

### begin
**EN:** Initializes basic pins (Button, LED).
**TR:** Temel pinleri (Buton, LED) başlatır.
**Syntax:** `void begin()`

### playIntro
**EN:** Plays a startup LED sequence.
**TR:** Başlangıç LED sekansını oynatır.
**Syntax:** `void playIntro()`

### serialStart
**EN:** Starts serial communication at the specified baud rate.
**TR:** Seri haberleşmeyi belirtilen baud hızında başlatır.
**Syntax:** `void serialStart(int baudrate)`

### serialWrite
**EN:** Writes data to the serial port (Overloaded for String, int, float, bool).
**TR:** Seri porta veri yazar (String, int, float, bool için aşırı yüklenmiştir).
**Syntax:** `void serialWrite(data)`

### button1Read
**EN:** Reads the state of the onboard button.
**TR:** Dahili butonun durumunu okur.
**Syntax:** `bool button1Read()`

### ledWrite
**EN:** Controls the onboard LED.
**TR:** Dahili LED'i kontrol eder.
**Syntax:** `void ledWrite(bool status)`

### moduleServoGoAngle
**EN:** Moves a servo connected to a specific pin to an angle with speed control.
**TR:** Belirli bir pine bağlı servoyu, hız kontrolü ile bir açıya hareket ettirir.
**Syntax:** `void moduleServoGoAngle(int pin, int angle, int acceleration)`

### moduleDhtTempReadC / F
**EN:** Reads temperature from a DHT sensor in Celsius or Fahrenheit.
**TR:** DHT sensöründen sıcaklığı Santigrat veya Fahrenhayt olarak okur.
**Syntax:** `int moduleDhtTempReadC(int pin)`

### moduleDhtHumRead
**EN:** Reads humidity from a DHT sensor.
**TR:** DHT sensöründen nemi okur.
**Syntax:** `int moduleDhtHumRead(int pin)`

### moduleUltrasonicDistanceRead
**EN:** Reads distance in cm using an ultrasonic sensor (Pins defined in library).
**TR:** Ultrasonik sensör kullanarak mesafeyi cm cinsinden okur (Pinler kütüphanede tanımlıdır).
**Syntax:** `int moduleUltrasonicDistanceRead()`

### moduleTraficLightWrite
**EN:** Controls a traffic light module (Red, Yellow, Green).
**TR:** Trafik ışığı modülünü kontrol eder (Kırmızı, Sarı, Yeşil).
**Syntax:** `void moduleTraficLightWrite(bool red, bool yellow, bool green)`

### moduleSmartLED... (NeoPixel)
**EN:** Functions to control NeoPixel LEDs (Prepare, Write, Effects like Rainbow, Theater Chase).
**TR:** NeoPixel LED'leri kontrol etme fonksiyonları (Hazırla, Yaz, Gökkuşağı, Tiyatro Kovalamaca gibi efektler).
**Syntax:** `void moduleSmartLEDPrepare(int pin)`, `void moduleSmartLEDWrite(...)`

### moduleIRReadHex
**EN:** Reads IR remote signal as a Hexadecimal string.
**TR:** IR kumanda sinyalini Onaltılık (Hex) dize olarak okur.
**Syntax:** `String moduleIRReadHex(int pin)`

### wifiStartAndConnect
**EN:** Connects to a WiFi network.
**TR:** Bir WiFi ağına bağlanır.
**Syntax:** `void wifiStartAndConnect(const char *ssid, const char *pass)`

### serverStart
**EN:** Starts a local web server.
**TR:** Yerel bir web sunucusu başlatır.
**Syntax:** `void serverStart(const char *mode, const char *ssid, const char *password)`

### fbServerSet... / fbServerGet...
**EN:** Functions to read/write data (Int, Float, String, JSON) to Firebase Realtime Database.
**TR:** Firebase Gerçek Zamanlı Veritabanına veri (Int, Float, String, JSON) okuma/yazma fonksiyonları.
**Syntax:** `void fbServerSetInt(const char *path, int value)`, `int fbServerGetInt(const char *path)`

### sendTelegram
**EN:** Sends a message via Telegram Bot API.
**TR:** Telegram Bot API üzerinden mesaj gönderir.
**Syntax:** `void sendTelegram(String token, String chatId, String message)`

### sendEmail
**EN:** Sends an email via SMTP.
**TR:** SMTP üzerinden e-posta gönderir.
**Syntax:** `void sendEmail(...)`

### getWeather
**EN:** Gets weather information for a city.
**TR:** Bir şehir için hava durumu bilgisini alır.
**Syntax:** `String getWeather(String city, String apiKey)`

### getWikipedia
**EN:** Gets a summary from Wikipedia.
**TR:** Wikipedia'dan bir özet alır.
**Syntax:** `String getWikipedia(String query, String lang)`
