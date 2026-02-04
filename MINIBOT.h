/*
 * CODLAI MINIBOT Library
 * 
 * Structure Information:
 * This library uses a modular structure to optimize memory usage and compilation time.
 * Features are enabled/disabled via definitions in the main sketch (e.g., #define USE_DHT).
 * 
 * IMPORTANT: Define feature flags BEFORE including this library in your sketch.
 */

#ifndef MINIBOT_H
#define MINIBOT_H

#include <Arduino.h>

#if defined(ESP8266)
#include <EEPROM.h>
#include <time.h>

#if defined(USE_SERVO)
#include <Servo.h>
#endif

#if defined(USE_DHT)
#include <DHT.h>
#endif

#if defined(USE_NEOPIXEL)
#include <Adafruit_NeoPixel.h>
#endif

#if defined(USE_IR)
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>
#endif

#if defined(USE_SERVER)
#ifndef USE_WIFI
#define USE_WIFI
#endif
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#endif

#if defined(USE_FIREBASE)
#ifndef USE_WIFI
#define USE_WIFI
#endif
#include <Firebase_ESP_Client.h>
#include <ArduinoJson.h>
#endif

#if defined(USE_WEATHER) || defined(USE_WIKIPEDIA) || defined(USE_TELEGRAM) || defined(USE_IFTTT)
#ifndef USE_WIFI
#define USE_WIFI
#endif
#endif

#if defined(USE_OTA)
#ifndef USE_WIFI
#define USE_WIFI
#endif
#include <ArduinoOTA.h>
#endif

#if defined(USE_WIFI)
#include <ESP8266WiFi.h>
#endif

#if defined(USE_ESPNOW)
#include <espnow.h>
#endif

#if defined(USE_EMAIL)
#ifndef USE_WIFI
#define USE_WIFI
#endif
#include <ESP_Mail_Client.h>
#endif

#if defined(USE_WEATHER) || defined(USE_WIKIPEDIA) || defined(USE_TELEGRAM) || defined(USE_IFTTT)
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>
#endif

// Structure to receive data via ESP-NOW
#ifndef CODLAI_ESPNOW_MESSAGE_DEFINED
#define CODLAI_ESPNOW_MESSAGE_DEFINED
typedef struct {
  uint8_t deviceType; // 1 = Armbot
  int axis1;
  int axis2;
  int axis3;
  int gripper;
  uint8_t action; // 0=None, 1=Horn, 2=Note
} CodlaiESPNowMessage;
#endif

// PinS
#define B1_BUTTON_PIN 0
#define BLUE_LED 16

#define IO4 4
#define IO5 5
#define IO12 12
#define IO13 13
#define IO14 14

class MINIBOT
{
public:
  MINIBOT();
  void begin();
  void playIntro();

  /*********************************** Serial Port ***********************************
   */
  void serialStart(int baundrate);
  void serialWrite(const char *message);
  void serialWrite(String message);
  void serialWrite(long value);
  void serialWrite(int value);
  void serialWrite(float value);
  void serialWrite(bool value);

  /*********************************** BUTTONS ***********************************/
  bool button1Read();

  /*********************************** LED ***********************************/
  void ledWrite(bool status);

  /*********************************** Servo Motor Sensor ***********************************
   */
#if defined(USE_SERVO)
  void moduleServoGoAngle(int pin, int angle, int acceleration);
#endif

  /*********************************** DHT Sensor ***********************************
   */
#if defined(USE_DHT)
  int moduleDhtTempReadC(int pin);
  int moduleDthFeelingTempC(int pin);
  int moduleDhtTempReadF(int pin);
  int moduleDthFeelingTempF(int pin);
  int moduleDhtHumRead(int pin);
#endif
  /*********************************** Magnetic Sensor ***********************************
   */
  bool moduleMagneticRead(int pin);

  /*********************************** Vibration Sensor ***********************************
   */
  bool moduleVibrationDigitalRead(int pin);

  /*********************************** Ultrasonic Distance Sensor ***********************************
   */
  int moduleUltrasonicDistanceRead();

  /*********************************** Trafic Ligh Sensor ***********************************
   */
  void moduleTraficLightWrite(bool red, bool yellow, bool green);
  void moduleTraficLightWriteRed(bool red);
  void moduleTraficLightWriteYellow(bool yellow);
  void moduleTraficLightWriteGreen(bool green);

  /*********************************** Smart LED Sensor ***********************************
   */
#if defined(USE_NEOPIXEL)
  void extendSmartLEDPrepare(int pin, int numLEDs);
  void extendSmartLEDFill(int startLED, int endLED, int red, int green, int blue);
  void moduleSmartLEDPrepare(int pin);                             // Initialize NeoPixel strip
  void moduleSmartLEDWrite(int led, int red, int green, int blue); // Write RGB values to specific LED
  void moduleSmartLEDRainbowEffect(int wait);                      // Rainbow effect
  void moduleSmartLEDRainbowTheaterChaseEffect(int wait);          // Rainbow theater chase effect
  void moduleSmartLEDTheaterChaseEffect(uint32_t color, int wait); // Theater chase effect
  void moduleSmartLEDColorWipeEffect(uint32_t color, int wait);    // Color wipe effect
  uint32_t getColor(int red, int green, int blue);                 // Helper function for creating colors
#endif
  /*********************************** Motion Sensor ***********************************
   */
  bool moduleMotionRead(int pin);

  /*********************************** IR Sensor ***********************************
   */
#if defined(USE_IR)
  String moduleIRReadHex(int pin);
  int moduleIRReadDecimalx32(int pin);
  int moduleIRReadDecimalx8(int pin);
#endif

  /*********************************** Relay Sensor ***********************************
   */
  void moduleRelayWrite(int pin, bool status);

  /*********************************** Buzzer ***********************************
   */
  void buzzerPlay(int frequency, int duration);

  /*********************************** OTHER PINS ***********************************
   */
  int digitalReadPin(int pin);
  void digitalWritePin(int pin, bool value);

  /*********************************** EEPROM  ***********************************
   */
  bool eepromBegin(size_t size = 512);
  bool eepromCommit();
  void eepromEnd();

  bool eepromWriteByte(int address, uint8_t value);
  uint8_t eepromReadByte(int address, uint8_t defaultValue = 0);

  // NOTE: eepromWriteInt/eepromReadInt store 16-bit (2 bytes) for backward compatibility.
  void eepromWriteInt(int address, int value);
  int eepromReadInt(int address);

  bool eepromWriteInt32(int address, int32_t value);
  int32_t eepromReadInt32(int address, int32_t defaultValue = 0);
  bool eepromWriteUInt32(int address, uint32_t value);
  uint32_t eepromReadUInt32(int address, uint32_t defaultValue = 0);
  bool eepromWriteFloat(int address, float value);
  float eepromReadFloat(int address, float defaultValue = 0.0f);

  // Stores: [uint16 length][bytes...]
  bool eepromWriteString(int address, const String &value, uint16_t maxLen = 128);
  String eepromReadString(int address, uint16_t maxLen = 128);

  bool eepromWriteBytes(int address, const uint8_t *data, size_t len);
  bool eepromReadBytes(int address, uint8_t *data, size_t len);
  bool eepromClear(int startAddress = 0, size_t length = 0, uint8_t fill = 0xFF);

  // Record helpers (recommended): Stores a blob with header + CRC32
  // Layout: [uint16 magic][uint16 version][uint16 len][uint32 crc32][bytes...]
  uint32_t eepromCrc32(const uint8_t *data, size_t len, uint32_t seed = 0xFFFFFFFF);
  bool eepromWriteRecord(int address, const uint8_t *data, uint16_t len, uint16_t version = 1);
  bool eepromReadRecord(int address, uint8_t *out, uint16_t maxLen, uint16_t *outLen = nullptr, uint16_t *outVersion = nullptr);

  /*********************************** WiFi  ***********************************
   */
#if defined(USE_WIFI)
  void wifiStartAndConnect(const char *ssid, const char *pass);
  bool wifiConnectionControl();
  String wifiGetMACAddress();
  String wifiGetIPAddress();
#endif

  /*********************************** OTA (Over-The-Air) ***********************************
   * TR: WiFi baglantisindan SONRA cagirilmalidir.
   * EN: Must be called AFTER WiFi connection is established.
   */
#if defined(USE_OTA)
  void otaBegin(const char *hostname = "CODLAI-MINIBOT", const char *password = "1234", uint16_t port = 3232);
  void otaHandle();
#endif

  /*********************************** NTP Time ***********************************
   * TR: WiFi baglantisindan SONRA cagirilmalidir.
   * EN: Must be called AFTER WiFi connection is established.
   */
  // TR/EN: Recommended simple setup for blocks.
  // timezoneHours: UTC offset in hours (e.g., Turkey is +3)
  bool ntpBegin(int timezoneHours = 0, const char *ntpServer = "pool.ntp.org", int daylightOffsetHours = 0, uint32_t timeoutMs = 10000);
  bool ntpSync(const char *ntpServer = "pool.ntp.org", long gmtOffsetSec = 0, int daylightOffsetSec = 0, uint32_t timeoutMs = 10000);
  bool ntpIsTimeValid(time_t minEpoch = 1609459200);
  time_t ntpGetEpoch();
  String ntpGetDateTimeString();

  /*********************************** Server  ***********************************
   */
#if defined(USE_SERVER)
  void serverStart(const char *mode, const char *ssid, const char *password);
  void serverCreateLocalPage(const char *url, const char *WEBPageScript, const char *WEBPageCSS, const char *WEBPageHTML, size_t bufferSize = 4096);
  void serverHandleDNS();
  void serverContinue();
#endif

  /*********************************** ESP-NOW ***********************************
   */
#if defined(USE_ESPNOW)
  void initESPNow();
  void setWiFiChannel(int channel);
  void sendESPNow(uint8_t *macAddr, uint8_t *data, int len);
  void registerOnRecv(esp_now_recv_cb_t cb);

  // ESP-NOW Data Handling
  CodlaiESPNowMessage receivedData;
  volatile bool newData = false;
  static MINIBOT* _instance;

  void startListening() {
      _instance = this;
      registerOnRecv([](uint8_t *mac, uint8_t *incomingData, uint8_t len) {
          if (_instance) {
            if (len == sizeof(CodlaiESPNowMessage)) {
                memcpy(&_instance->receivedData, incomingData, sizeof(CodlaiESPNowMessage));
                _instance->newData = true;
            } else {
                Serial.print("ESP-NOW Error: Received len ");
                Serial.print(len);
                Serial.print(" expected ");
                Serial.println(sizeof(CodlaiESPNowMessage));
            }
          }
      });
  }
#endif

  /*********************************** Email ***********************************
   */
#if defined(USE_EMAIL)
  void sendEmail(String smtpHost, int smtpPort, String authorEmail, String authorPassword, String recipientEmail, String subject, String message);
#endif

  /*********************************** Weather ***********************************
   */
#if defined(USE_WEATHER)
  String getWeather(String city, String apiKey);
#endif

  /*********************************** Wikipedia ***********************************
   */
#if defined(USE_WIKIPEDIA)
  String getWikipedia(String query, String lang = "en");
#endif

  /*********************************** Telegram ***********************************
   */
#if defined(USE_TELEGRAM)
  void sendTelegram(String token, String chatId, String message);
#endif

  /*********************************** IFTTT ***********************************
   */
#if defined(USE_IFTTT)
  bool triggerIFTTTEvent(const String &eventName, const String &webhookKey, const String &jsonPayload = "{}");
#endif

  /*********************************** Firebase Server  ***********************************
   */
#if defined(USE_FIREBASE)
  // 📡 Firebase Server Functions
  void fbServerSetandStartWithUser(const char *projectURL, const char *secretKey, const char *userMail, const char *mailPass); // projectURL: YOUR_FIREBASE_PROJECT_ID.firebaseio.com / secretKey: YOUR_FIREBASE_DATABASE_SECRET

  // 🔄 Firebase Database Write Functions
  void fbServerSetInt(const char *dataPath, int data);
  void fbServerSetFloat(const char *dataPath, float data);
  void fbServerSetString(const char *dataPath, String data);
  void fbServerSetDouble(const char *dataPath, double data);
  void fbServerSetBool(const char *dataPath, bool data);
  void fbServerSetJSON(const char *dataPath, String data);

  // 📥 Firebase Database Read Functions
  int fbServerGetInt(const char *dataPath);
  float fbServerGetFloat(const char *dataPath);
  String fbServerGetString(const char *dataPath);
  double fbServerGetDouble(const char *dataPath);
  bool fbServerGetBool(const char *dataPath);
  String fbServerGetJSON(const char *dataPath);
#endif
private:
  static constexpr uint16_t _EEPROM_RECORD_MAGIC = 0xCD1A;
  static constexpr time_t _NTP_VALID_EPOCH = 1609459200; // 2021-01-01

  bool _eepromReady = false;
  size_t _eepromSize = 0;

  bool _eepromEnsure(size_t minSize);

#if defined(USE_SERVO)
#ifndef MAX_SERVOS
#define MAX_SERVOS 6
#endif
  Servo servos[MAX_SERVOS]; 
  int servoPins[MAX_SERVOS];
  int currentAngles[MAX_SERVOS];
#endif

#if defined(USE_DHT)
  void initializeDht(int pin, uint8_t type);
  DHT *dhtSensor; // Pointer to DHT sensor object
#endif

#if defined(USE_NEOPIXEL)
  Adafruit_NeoPixel *pixels; // NeoPixel object pointer
#endif

#if defined(USE_IR)

  void initializeIR(int pin);
  IRrecv *irrecv = nullptr; // Pointer to IR receiver / IR alıcısı için pointer
  decode_results results;   // Stores received IR results / Alınan IR sinyallerini saklar
  int irPin;                // Store the IR receiver pin / IR alıcı pini sakla
  long irRawValue = 0;      // Stores last received IR value / En son alınan IR değerini saklar

#endif

#if defined(USE_SERVER)
  const IPAddress apIP = IPAddress(192, 168, 4, 1); // Sabit IP adresi tanımlanıyor / Define static IP address
  DNSServer dnsServer;                              // DNS sunucusu tanımlanıyor / Define DNS Server
  AsyncWebServer serverCODLAI{80};                  // Web server objesi
  AsyncWebSocket *serverCODLAIWebSocket;            // Pointer olarak tanımla
#endif

#if defined(USE_FIREBASE)
  FirebaseData firebaseData;     // Data object to handle Firebase communication
  FirebaseAuth firebaseAuth;     // Authentication credentials for user verification
  FirebaseConfig firebaseConfig; // Configuration settings for Firebase
  char uid[128] = "";            // User ID storage
#endif
};

/*********************************** IMPLEMENTATION ***********************************/

inline MINIBOT::MINIBOT()
{
#if defined(USE_SERVER)
  serverCODLAIWebSocket = new AsyncWebSocket("/serverCODLAIWebSocket");
#endif
#if defined(USE_SERVO)
  for(int i=0; i<MAX_SERVOS; i++) {
      servoPins[i] = -1;
      currentAngles[i] = 0;
  }
#endif
}

inline void MINIBOT::begin()
{
  pinMode(B1_BUTTON_PIN, INPUT_PULLUP);
  pinMode(BLUE_LED, OUTPUT);
}

inline void MINIBOT::playIntro()
{
  digitalWrite(BLUE_LED, HIGH);
  delay(100);
  digitalWrite(BLUE_LED, LOW);
  delay(100);
  digitalWrite(BLUE_LED, HIGH);
  delay(100);
  digitalWrite(BLUE_LED, LOW);
  delay(100);
  digitalWrite(BLUE_LED, HIGH);
  delay(100);
  digitalWrite(BLUE_LED, LOW);
  delay(100);
}

/*********************************** Serial Port ***********************************
 */
inline void MINIBOT::serialStart(int baudrate)
{
  Serial.begin(baudrate);
}

inline void MINIBOT::serialWrite(const char *message) // Overloaded function for const char* / `const char*` için fonksiyon
{
  Serial.println(message);
}

inline void MINIBOT::serialWrite(String message) // Overloaded function for String / `String` için özel fonksiyon
{
  Serial.println(message.c_str()); // Convert String to const char*
}

inline void MINIBOT::serialWrite(long value) // Overloaded function for long / `long` için özel fonksiyon
{
  Serial.println(String(value).c_str());
}

inline void MINIBOT::serialWrite(int value) // Overloaded function for int / `int` için fonksiyon
{
  Serial.println(String(value).c_str());
}

inline void MINIBOT::serialWrite(float value) // Overloaded function for float / `float` için fonksiyon
{
  Serial.println(String(value).c_str());
}

inline void MINIBOT::serialWrite(bool value) // Overloaded function for bool / `bool` için fonksiyon
{
  Serial.println(value ? "true" : "false");
}

/*********************************** BUTTONS ***********************************
 */
inline bool MINIBOT::button1Read()
{
  return digitalRead(B1_BUTTON_PIN);
}

/*********************************** LED ***********************************
 */
inline void MINIBOT::ledWrite(bool status)
{
  return digitalWrite(BLUE_LED, status);
}

/*********************************** Servo Angle Control ***********************************
 */
#ifdef USE_SERVO

inline void MINIBOT::moduleServoGoAngle(int pin, int angle, int acceleration)
{
  int idx = -1;
  // Find existing
  for(int i=0; i<MAX_SERVOS; i++) {
      if(servoPins[i] == pin) { idx = i; break; }
  }
  // If not found, find free slot
  if(idx == -1) {
      for(int i=0; i<MAX_SERVOS; i++) {
          if(servoPins[i] == -1) {
              idx = i;
              servoPins[i] = pin;
              servos[i].attach(pin, 500, 2500);
              break;
          }
      }
  }

  if(idx != -1) {
      // Ensure angle is within valid bounds (0 to 180 degrees)
      angle = constrain(angle, 0, 180);

      if (acceleration <= 0) {
          // Instant move
          servos[idx].write(angle);
          currentAngles[idx] = angle;
      } else {
          // Gradual move
          int currentAngle = currentAngles[idx];
          int step = (angle > currentAngle) ? 1 : -1;

          for (int pos = currentAngle; pos != angle; pos += step)
          {
            servos[idx].write(pos); 
            delay(acceleration);    
          }
          servos[idx].write(angle);
          currentAngles[idx] = angle;
      }
  }
}
#endif

/*********************************** DHT Sensor Initialization ***********************************
 */
#if defined(USE_DHT)
inline void MINIBOT::initializeDht(int pin, uint8_t type)
{
  if (!dhtSensor)
  {
    dhtSensor = new DHT(pin, type); // Create a new DHT object
    dhtSensor->begin();             // Initialize the sensor
  }
}

inline int MINIBOT::moduleDhtTempReadC(int pin) // Read Temperature
{
  initializeDht(pin, DHT11); // Ensure DHT11 is initialized
  float temp = dhtSensor->readTemperature();

  if (isnan(temp)) // Check if reading failed
    return -999;

  return static_cast<int>(temp);
}

inline int MINIBOT::moduleDthFeelingTempC(int pin) // Calculate Heat Index (Feeling Temperature)
{
  initializeDht(pin, DHT11); // Ensure DHT11 is initialized

  float temp = dhtSensor->readTemperature();
  float hum = dhtSensor->readHumidity();

  if (isnan(temp) || isnan(hum)) // Check if readings failed
    return -999;

  float heatIndex = dhtSensor->computeHeatIndex(temp, hum, false); // Calculate heat index in Celsius
  return static_cast<int>(heatIndex);
}

inline int MINIBOT::moduleDhtTempReadF(int pin) // Read Temperature in Fahrenheit
{
  initializeDht(pin, DHT11);                     // Ensure DHT11 is initialized
  float temp = dhtSensor->readTemperature(true); // **Fahrenheit sıcaklık okuma**

  if (isnan(temp)) // Check if reading failed
    return -999;

  return static_cast<int>(temp);
}

inline int MINIBOT::moduleDthFeelingTempF(int pin) // Calculate Heat Index (Feeling Temperature in Fahrenheit)
{
  initializeDht(pin, DHT11); // Ensure DHT11 is initialized

  float temp = dhtSensor->readTemperature(true); // **Fahrenheit sıcaklık okuma**
  float hum = dhtSensor->readHumidity();         // **Nem okuma**

  if (isnan(temp) || isnan(hum)) // Check if readings failed
    return -999;

  float heatIndex = dhtSensor->computeHeatIndex(temp, hum, true); // **Fahrenheit olarak hissedilen sıcaklık hesapla**
  return static_cast<int>(heatIndex);
}

inline int MINIBOT::moduleDhtHumRead(int pin) // Read Humidity
{
  initializeDht(pin, DHT11); // Ensure DHT11 is initialized
  float hum = dhtSensor->readHumidity();

  if (isnan(hum)) // Check if reading failed
    return -999;

  return static_cast<int>(hum);
}
#endif

/*********************************** Magnetic Sensor ***********************************
 */
inline bool MINIBOT::moduleMagneticRead(int pin)
{
  pinMode(pin, INPUT);
  return !digitalRead(pin);
}

/*********************************** Vibration Sensor ***********************************
 */
inline bool MINIBOT::moduleVibrationDigitalRead(int pin)
{
  pinMode(pin, INPUT);
  return digitalRead(pin);
}

/*********************************** Ultrasonic Distance Sensor ***********************************
 */
inline int MINIBOT::moduleUltrasonicDistanceRead()
{
#if defined(ESP8266)
  const int TRIG_PIN = IO12;
  const int ECHO_PIN = IO13;
#else
#error "Unsupported platform! Only ESP8266 are supported."
#endif

  // Maximum measurable distance for HC-SR04 (in centimeters)
  const int MAX_DISTANCE = 400;

  // Configure pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Send a 10-microsecond pulse on the TRIG_PIN
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(ECHO_PIN, HIGH, MAX_DISTANCE * 58); // Timeout for MAX_DISTANCE

  // If no echo is received, return 0 (out of range)
  if (duration == 0)
  {
    return 0; // Out of range or no object detected
  }

  // Calculate the distance in centimeters
  int distance = duration * 0.034 / 2; // Sound speed: 0.034 cm/µs, divide by 2 for round trip

  // If the calculated distance exceeds the maximum range, return 0
  if (distance > MAX_DISTANCE)
  {
    return 000;
  }

  return distance; // Return the measured distance
}

/*********************************** Trafic Ligh Sensor ***********************************
 */
inline void MINIBOT::moduleTraficLightWrite(bool red, bool yellow, bool green)
{
#if defined(ESP8266)
  const int RED_PIN = IO13;
  const int YELLOW_PIN = IO5;
  const int GREEN_PIN = IO4;
#else
#error "Unsupported platform! Only ESP8266 are supported."
#endif

  // Configure pins
  pinMode(RED_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);

  if (red)
  {
    digitalWrite(RED_PIN, HIGH);
  }
  else
  {
    digitalWrite(RED_PIN, LOW);
  }
  if (yellow)
  {
    digitalWrite(YELLOW_PIN, HIGH);
  }
  else
  {
    digitalWrite(YELLOW_PIN, LOW);
  }
  if (green)
  {
    digitalWrite(GREEN_PIN, HIGH);
  }
  else
  {
    digitalWrite(GREEN_PIN, LOW);
  }
}

inline void MINIBOT::moduleTraficLightWriteRed(bool red)
{
#if defined(ESP8266)
  const int RED_PIN = IO13;
#else
#error "Unsupported platform! Only ESP8266 are supported."
#endif

  // Configure pins
  pinMode(RED_PIN, OUTPUT);

  if (red)
  {
    digitalWrite(RED_PIN, HIGH);
  }
  else
  {
    digitalWrite(RED_PIN, LOW);
  }
}

inline void MINIBOT::moduleTraficLightWriteYellow(bool yellow)
{
#if defined(ESP8266)
  const int YELLOW_PIN = IO5;
#else
#error "Unsupported platform! Only ESP8266 are supported."
#endif

  // Configure pins
  pinMode(YELLOW_PIN, OUTPUT);

  if (yellow)
  {
    digitalWrite(YELLOW_PIN, HIGH);
  }
  else
  {
    digitalWrite(YELLOW_PIN, LOW);
  }
}

inline void MINIBOT::moduleTraficLightWriteGreen(bool green)
{
#if defined(ESP8266)
  const int GREEN_PIN = IO4;
#else
#error "Unsupported platform! Only ESP8266 are supported."
#endif

  // Configure pins

  pinMode(GREEN_PIN, OUTPUT);

  if (green)
  {
    digitalWrite(GREEN_PIN, HIGH);
  }
  else
  {
    digitalWrite(GREEN_PIN, LOW);
  }
}

/*********************************** Smart LED Sensor ***********************************
 */
#if defined(USE_NEOPIXEL)
inline void MINIBOT::extendSmartLEDPrepare(int pin, int numLEDs)
{
  // Create a new Adafruit_NeoPixel object dynamically
  pixels = new Adafruit_NeoPixel(numLEDs, pin, NEO_GRB + NEO_KHZ800);
  pixels->begin(); // Initialize the NeoPixel strip
  pixels->show();  // Turn off all LEDs initially
}

inline void MINIBOT::extendSmartLEDFill(int startLED, int endLED, int red, int green, int blue)
{
  if (pixels)
  {
    // Set the color for a range of LEDs
    for (int i = startLED; i <= endLED; i++)
    {
      pixels->setPixelColor(i, pixels->Color(red, green, blue));
    }
    pixels->show(); // Update the LEDs
  }
}

inline void MINIBOT::moduleSmartLEDPrepare(int pin)
{
  pixels = new Adafruit_NeoPixel(3, pin, NEO_GRB + NEO_KHZ800);
  pixels->begin();
  pixels->show(); // Clear all LEDs
}

inline void MINIBOT::moduleSmartLEDWrite(int led, int red, int green, int blue)
{
  if (pixels)
  {
    pixels->setPixelColor(led, pixels->Color(red, green, blue));
    pixels->show();
  }
}

inline uint32_t MINIBOT::getColor(int red, int green, int blue)
{
  return pixels->Color(red, green, blue);
}

inline void MINIBOT::moduleSmartLEDRainbowEffect(int wait)
{
  if (pixels)
  {
    for (long firstPixelHue = 0; firstPixelHue < 3 * 65536; firstPixelHue += 256)
    {
      for (int i = 0; i < pixels->numPixels(); i++)
      {
        int pixelHue = firstPixelHue + (i * 65536L / pixels->numPixels());
        pixels->setPixelColor(i, pixels->gamma32(pixels->ColorHSV(pixelHue)));
      }
      pixels->show();
      delay(wait);
    }
  }
}

inline void MINIBOT::moduleSmartLEDRainbowTheaterChaseEffect(int wait)
{
  if (pixels)
  {
    int firstPixelHue = 0;
    for (int a = 0; a < 30; a++)
    {
      for (int b = 0; b < 3; b++)
      {
        pixels->clear();
        for (int c = b; c < pixels->numPixels(); c += 3)
        {
          int hue = firstPixelHue + c * 65536L / pixels->numPixels();
          uint32_t color = pixels->gamma32(pixels->ColorHSV(hue));
          pixels->setPixelColor(c, color);
        }
        pixels->show();
        delay(wait);
        firstPixelHue += 65536 / 90;
      }
    }
  }
}

inline void MINIBOT::moduleSmartLEDTheaterChaseEffect(uint32_t color, int wait)
{
  if (pixels)
  {
    for (int a = 0; a < 10; a++)
    {
      for (int b = 0; b < 3; b++)
      {
        pixels->clear();
        for (int c = b; c < pixels->numPixels(); c += 3)
        {
          pixels->setPixelColor(c, color);
        }
        pixels->show();
        delay(wait);
      }
    }
  }
}

inline void MINIBOT::moduleSmartLEDColorWipeEffect(uint32_t color, int wait)
{
  if (pixels)
  {
    for (int i = 0; i < pixels->numPixels(); i++)
    {
      pixels->setPixelColor(i, color);
      pixels->show();
      delay(wait);
    }
  }
}
#endif

/*********************************** Motion Sensor ***********************************
 */
inline bool MINIBOT::moduleMotionRead(int pin)
{
  pinMode(pin, INPUT);
  return digitalRead(pin);
}

/*********************************** IR Sensor ***********************************
 */
#if defined(USE_IR)

// Initialize the IR module / IR modülünü başlat
inline void MINIBOT::initializeIR(int pin)
{
  if (!irrecv || irPin != pin)
  {                                           // Eğer IR alıcı yoksa veya pin değişmişse baştan başlat
    irPin = pin;                              // Store the IR receiver pin / IR alıcı pini sakla
    delete irrecv;                            // Önceki nesneyi temizle
    irrecv = new IRrecv(pin, 1024, 50, true); // Create a new IRrecv instance / Yeni bir IRrecv nesnesi oluştur
    irrecv->enableIRIn();                     // Start the IR receiver / IR alıcıyı başlat
  }
}

// Read IR signal in hexadecimal format / IR sinyalini HEX formatında oku
inline String MINIBOT::moduleIRReadHex(int pin)
{
  initializeIR(pin); // Ensure IR is initialized / IR'nin başlatıldığından emin ol
  if (irrecv->decode(&results))
  {
    String hexCode = "0x" + String(results.value, HEX); // Convert to HEX / HEX formatına çevir
    irrecv->resume();                                   // Continue receiving new data / Yeni veri almak için devam et
    return hexCode;
  }
  return "0"; // No signal received / Sinyal yoksa 0 döndür
}

// Read IR signal as a full 32-bit decimal value / IR sinyalini tam 32-bit ondalık formatta oku
inline int MINIBOT::moduleIRReadDecimalx32(int pin)
{
  initializeIR(pin); // Ensure IR is initialized / IR'nin başlatıldığından emin ol
  if (irrecv->decode(&results))
  {
    int decimalCode = results.value; // Return the full 32-bit value / Tam 32-bit değeri döndür
    irrecv->resume();                // Continue receiving new data / Yeni veri almak için devam et
    return decimalCode;
  }
  return 0; // No signal received / Sinyal yoksa 0 döndür
}

// Read IR signal as only the last 8 bits (for smaller values) / IR sinyalini sadece son 8 bit olarak oku (küçük değerler için)
inline int MINIBOT::moduleIRReadDecimalx8(int pin)
{
  initializeIR(pin); // Ensure IR is initialized / IR'nin başlatıldığından emin ol
  if (irrecv->decode(&results))
  {
    int smallCode = results.value & 0xFF; // Extract only the last 8 bits / Sadece son 8 biti al
    irrecv->resume();                     // Continue receiving new data / Yeni veri almak için devam et
    return smallCode;
  }
  return 0; // No signal received / Sinyal yoksa 0 döndür
}
#endif

/*********************************** Relay Sensor ***********************************
 */
inline void MINIBOT::moduleRelayWrite(int pin, bool status)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, status);
}

/*********************************** Buzzer ***********************************
 */
inline void MINIBOT::buzzerPlay(int frequency, int duration)
{
  #if defined(ESP8266)
  pinMode(IO5, OUTPUT);
  tone(IO5, frequency, duration);
  #endif
}

/*********************************** OTHER PINS ***********************************
 */

inline int MINIBOT::digitalReadPin(int pin)
{
  pinMode(pin, INPUT);
  return digitalRead(pin);
}

inline void MINIBOT::digitalWritePin(int pin, bool value)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, value);
}

/*********************************** EEPROM  ***********************************
 */
inline bool MINIBOT::_eepromEnsure(size_t minSize)
{
  if (_eepromReady && _eepromSize >= minSize)
  {
    return true;
  }

  size_t targetSize = _eepromSize;
  if (targetSize == 0)
  {
    targetSize = 512;
  }
  if (targetSize < minSize)
  {
    targetSize = minSize;
  }

  return eepromBegin(targetSize);
}

inline bool MINIBOT::eepromBegin(size_t size)
{
  if (size == 0)
  {
    size = 512;
  }

  if (_eepromReady && _eepromSize == size)
  {
    return true;
  }

  _eepromSize = size;
#if defined(ESP8266)
  EEPROM.begin(size);
  _eepromReady = true;
#else
  _eepromReady = EEPROM.begin(size);
#endif
  return _eepromReady;
}

inline bool MINIBOT::eepromCommit()
{
  if (!_eepromEnsure(2))
  {
    return false;
  }

  return EEPROM.commit();
}

inline void MINIBOT::eepromEnd()
{
  if (_eepromReady)
  {
    EEPROM.end();
  }
  _eepromReady = false;
  _eepromSize = 0;
}

inline bool MINIBOT::eepromWriteByte(int address, uint8_t value)
{
  if (address < 0)
  {
    return false;
  }

  if (!_eepromEnsure((size_t)address + 1))
  {
    return false;
  }

  EEPROM.write(address, value);
  return EEPROM.commit();
}

inline uint8_t MINIBOT::eepromReadByte(int address, uint8_t defaultValue)
{
  if (address < 0)
  {
    return defaultValue;
  }

  if (!_eepromEnsure((size_t)address + 1))
  {
    return defaultValue;
  }

  return EEPROM.read(address);
}

inline void MINIBOT::eepromWriteInt(int address, int value) // EEPROM'a güvenli bir şekilde int türünde veri yazmak için fonksiyon
{
  if (address < 0)
  {
    return;
  }

  if (!_eepromEnsure((size_t)address + 2))
  {
    return;
  }

  uint8_t hi = highByte(value); // int'in yüksek baytını al
  uint8_t lo = lowByte(value);  // int'in düşük baytını al

  EEPROM.write(address, hi);      // İlk baytı EEPROM'a yaz
  EEPROM.write(address + 1, lo);  // İkinci baytı EEPROM'a yaz
  EEPROM.commit();                    // Değişiklikleri kaydetmek için commit işlemi yapılmalıdır
}

inline int MINIBOT::eepromReadInt(int address) // EEPROM'dan int türünde veri okumak için fonksiyon
{
  if (address < 0)
  {
    return 0;
  }

  if (!_eepromEnsure((size_t)address + 2))
  {
    return 0;
  }

  uint8_t hi = EEPROM.read(address);     // İlk baytı oku
  uint8_t lo = EEPROM.read(address + 1); // İkinci baytı oku
  return word(hi, lo);                   // Yüksek ve düşük baytları birleştirerek int değeri oluştur
}

inline bool MINIBOT::eepromWriteInt32(int address, int32_t value)
{
  if (address < 0)
  {
    return false;
  }
  if (!_eepromEnsure((size_t)address + sizeof(int32_t)))
  {
    return false;
  }

  EEPROM.put(address, value);
  return EEPROM.commit();
}

inline int32_t MINIBOT::eepromReadInt32(int address, int32_t defaultValue)
{
  if (address < 0)
  {
    return defaultValue;
  }
  if (!_eepromEnsure((size_t)address + sizeof(int32_t)))
  {
    return defaultValue;
  }

  int32_t value;
  EEPROM.get(address, value);
  return value;
}

inline bool MINIBOT::eepromWriteUInt32(int address, uint32_t value)
{
  if (address < 0)
  {
    return false;
  }
  if (!_eepromEnsure((size_t)address + sizeof(uint32_t)))
  {
    return false;
  }

  EEPROM.put(address, value);
  return EEPROM.commit();
}

inline uint32_t MINIBOT::eepromReadUInt32(int address, uint32_t defaultValue)
{
  if (address < 0)
  {
    return defaultValue;
  }
  if (!_eepromEnsure((size_t)address + sizeof(uint32_t)))
  {
    return defaultValue;
  }

  uint32_t value;
  EEPROM.get(address, value);
  return value;
}

inline bool MINIBOT::eepromWriteFloat(int address, float value)
{
  if (address < 0)
  {
    return false;
  }
  if (!_eepromEnsure((size_t)address + sizeof(float)))
  {
    return false;
  }

  EEPROM.put(address, value);
  return EEPROM.commit();
}

inline float MINIBOT::eepromReadFloat(int address, float defaultValue)
{
  if (address < 0)
  {
    return defaultValue;
  }
  if (!_eepromEnsure((size_t)address + sizeof(float)))
  {
    return defaultValue;
  }

  float value;
  EEPROM.get(address, value);
  return value;
}

inline bool MINIBOT::eepromWriteBytes(int address, const uint8_t *data, size_t len)
{
  if (address < 0 || data == nullptr)
  {
    return false;
  }

  if (len == 0)
  {
    return true;
  }

  if (!_eepromEnsure((size_t)address + len))
  {
    return false;
  }

  for (size_t i = 0; i < len; i++)
  {
    EEPROM.write(address + (int)i, data[i]);
  }

  return EEPROM.commit();
}

inline bool MINIBOT::eepromReadBytes(int address, uint8_t *data, size_t len)
{
  if (address < 0 || data == nullptr)
  {
    return false;
  }

  if (len == 0)
  {
    return true;
  }

  if (!_eepromEnsure((size_t)address + len))
  {
    return false;
  }

  for (size_t i = 0; i < len; i++)
  {
    data[i] = EEPROM.read(address + (int)i);
  }

  return true;
}

inline bool MINIBOT::eepromWriteString(int address, const String &value, uint16_t maxLen)
{
  if (address < 0)
  {
    return false;
  }

  if (maxLen == 0)
  {
    return false;
  }

  uint16_t len = (uint16_t)value.length();
  if (len > maxLen)
  {
    len = maxLen;
  }

  size_t total = sizeof(uint16_t) + (size_t)len;
  if (!_eepromEnsure((size_t)address + total))
  {
    return false;
  }

  EEPROM.put(address, len);
  for (uint16_t i = 0; i < len; i++)
  {
    EEPROM.write(address + (int)sizeof(uint16_t) + (int)i, (uint8_t)value[i]);
  }

  return EEPROM.commit();
}

inline String MINIBOT::eepromReadString(int address, uint16_t maxLen)
{
  if (address < 0 || maxLen == 0)
  {
    return String("");
  }

  if (!_eepromEnsure((size_t)address + sizeof(uint16_t)))
  {
    return String("");
  }

  uint16_t len = 0;
  EEPROM.get(address, len);

  if (len > maxLen)
  {
    len = maxLen;
  }

  if (!_eepromEnsure((size_t)address + sizeof(uint16_t) + (size_t)len))
  {
    return String("");
  }

  String out;
  out.reserve(len);
  for (uint16_t i = 0; i < len; i++)
  {
    out += (char)EEPROM.read(address + (int)sizeof(uint16_t) + (int)i);
  }
  return out;
}

inline bool MINIBOT::eepromClear(int startAddress, size_t length, uint8_t fill)
{
  if (startAddress < 0)
  {
    return false;
  }

  if (length == 0)
  {
    length = (_eepromSize == 0) ? 512 : _eepromSize;
  }

  if (!_eepromEnsure((size_t)startAddress + length))
  {
    return false;
  }

  for (size_t i = 0; i < length; i++)
  {
    EEPROM.write(startAddress + (int)i, fill);
  }

  return EEPROM.commit();
}

inline uint32_t MINIBOT::eepromCrc32(const uint8_t *data, size_t len, uint32_t seed)
{
  if (data == nullptr)
  {
    return 0;
  }

  uint32_t crc = seed;
  for (size_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
    {
      if (crc & 1)
      {
        crc = (crc >> 1) ^ 0xEDB88320UL;
      }
      else
      {
        crc >>= 1;
      }
    }
  }
  return crc ^ 0xFFFFFFFFUL;
}

inline bool MINIBOT::eepromWriteRecord(int address, const uint8_t *data, uint16_t len, uint16_t version)
{
  if (address < 0 || data == nullptr)
  {
    return false;
  }

  const size_t headerSize = 2 + 2 + 2 + 4;
  if (!_eepromEnsure((size_t)address + headerSize + (size_t)len))
  {
    return false;
  }

  const uint16_t magic = _EEPROM_RECORD_MAGIC;
  const uint32_t crc = eepromCrc32(data, len);

  EEPROM.put(address, magic);
  EEPROM.put(address + 2, version);
  EEPROM.put(address + 4, len);
  EEPROM.put(address + 6, crc);

  for (uint16_t i = 0; i < len; i++)
  {
    EEPROM.write(address + (int)headerSize + (int)i, data[i]);
  }

  return EEPROM.commit();
}

inline bool MINIBOT::eepromReadRecord(int address, uint8_t *out, uint16_t maxLen, uint16_t *outLen, uint16_t *outVersion)
{
  if (address < 0 || out == nullptr || maxLen == 0)
  {
    return false;
  }

  const size_t headerSize = 2 + 2 + 2 + 4;
  if (!_eepromEnsure((size_t)address + headerSize))
  {
    return false;
  }

  uint16_t magic = 0;
  uint16_t version = 0;
  uint16_t len = 0;
  uint32_t storedCrc = 0;

  EEPROM.get(address, magic);
  if (magic != _EEPROM_RECORD_MAGIC)
  {
    return false;
  }

  EEPROM.get(address + 2, version);
  EEPROM.get(address + 4, len);
  EEPROM.get(address + 6, storedCrc);

  if (len > maxLen)
  {
    return false;
  }

  if (!_eepromEnsure((size_t)address + headerSize + (size_t)len))
  {
    return false;
  }

  for (uint16_t i = 0; i < len; i++)
  {
    out[i] = EEPROM.read(address + (int)headerSize + (int)i);
  }

  const uint32_t calcCrc = eepromCrc32(out, len);
  if (calcCrc != storedCrc)
  {
    return false;
  }

  if (outLen)
  {
    *outLen = len;
  }
  if (outVersion)
  {
    *outVersion = version;
  }
  return true;
}

inline bool MINIBOT::ntpSync(const char *ntpServer, long gmtOffsetSec, int daylightOffsetSec, uint32_t timeoutMs)
{
  if (ntpServer == nullptr || ntpServer[0] == '\0')
  {
    ntpServer = "pool.ntp.org";
  }

  configTime(gmtOffsetSec, daylightOffsetSec, ntpServer);

  const uint32_t startMs = millis();
  while ((millis() - startMs) < timeoutMs)
  {
    time_t now = time(nullptr);
    if (now >= _NTP_VALID_EPOCH)
    {
      return true;
    }
    delay(50);
  }

  return false;
}

inline bool MINIBOT::ntpBegin(int timezoneHours, const char *ntpServer, int daylightOffsetHours, uint32_t timeoutMs)
{
  const long gmtOffsetSec = (long)timezoneHours * 3600L;
  const int daylightOffsetSec = daylightOffsetHours * 3600;
  return ntpSync(ntpServer, gmtOffsetSec, daylightOffsetSec, timeoutMs);
}

inline bool MINIBOT::ntpIsTimeValid(time_t minEpoch)
{
  if (minEpoch <= 0)
  {
    minEpoch = _NTP_VALID_EPOCH;
  }
  return time(nullptr) >= minEpoch;
}

inline time_t MINIBOT::ntpGetEpoch()
{
  return time(nullptr);
}

inline String MINIBOT::ntpGetDateTimeString()
{
  time_t now = time(nullptr);
  if (now < _NTP_VALID_EPOCH)
  {
    return String("");
  }

  struct tm tmInfo;
  localtime_r(&now, &tmInfo);

  char buf[80];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           tmInfo.tm_year + 1900,
           tmInfo.tm_mon + 1,
           tmInfo.tm_mday,
           tmInfo.tm_hour,
           tmInfo.tm_min,
           tmInfo.tm_sec);
  return String(buf);
}

/*********************************** WiFi ***********************************/
#if defined(USE_WIFI)

inline void MINIBOT::wifiStartAndConnect(const char *ssid, const char *pass)
{
  Serial.printf("[WiFi]: Connection Starting!\r\n[WiFi]: SSID: %s\r\n[WiFi]: Pass: %s\r\n", ssid, pass);

  WiFi.begin(ssid, pass);
  int count = 0;
  while (count < 30)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.printf("\n[WiFi]: Connected!\r\n[WiFi]: Local IP: %s\r\n", WiFi.localIP().toString().c_str());
      Serial.printf("[WiFi]: MAC Address: %s\r\n", WiFi.macAddress().c_str());
      return;
    }
    Serial.print(".");
    delay(500);
    count++;
  }
  Serial.println("[WiFi]: Connection Timeout!");
}

inline bool MINIBOT::wifiConnectionControl()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("[WiFi]: Connection OK!");
    return true;
  }
  else
  {
    Serial.println("[WiFi]: Connection ERROR!");
    return false;
  }
}

inline String MINIBOT::wifiGetMACAddress()
{
  return WiFi.macAddress();
}

inline String MINIBOT::wifiGetIPAddress()
{
  return WiFi.localIP().toString();
}
#endif

/*********************************** OTA (Over-The-Air) ***********************************/
#if defined(USE_OTA)
inline void MINIBOT::otaBegin(const char *hostname, const char *password, uint16_t port)
{
  static String otaHost;
  if (hostname && strlen(hostname) > 0)
  {
    otaHost = hostname;
  }
  else
  {
    String mac = WiFi.macAddress();
    mac.replace(":", "");
    otaHost = String("MINIBOT-") + mac;
  }
  ArduinoOTA.setHostname(otaHost.c_str());

  if (password && strlen(password) > 0)
  {
    ArduinoOTA.setPassword(password);
  }
  else
  {
    ArduinoOTA.setPassword("1234");
  }
  else
  {
    ArduinoOTA.setPassword("1234");
  }

  ArduinoOTA.setPort(port);

  ArduinoOTA.onStart([]() {
    Serial.println("[OTA]: Start");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\n[OTA]: End");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (total > 0)
    {
      Serial.printf("[OTA]: Progress: %u%%\r", (progress * 100) / total);
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA]: Error[%u]\n", error);
  });

  ArduinoOTA.begin();
  Serial.println("[OTA]: Ready");
}

inline void MINIBOT::otaHandle()
{
  ArduinoOTA.handle();
}
#endif

/*********************************** Server ***********************************/
#if defined(USE_SERVER)
inline void MINIBOT::serverStart(const char *mode, const char *ssid, const char *password)
{
  if (strcmp(mode, "STA") == 0)
  {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    Serial.printf("\n[STA Mode]: Connecting to WiFi: %s\n", ssid);

    int retries = 30;
    while (WiFi.status() != WL_CONNECTED && retries > 0)
    {
      delay(1000);
      Serial.print(".");
      retries--;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("\n[STA Mode]: Connected!");
      Serial.printf("[STA Mode]: IP Address: http://%s\n", WiFi.localIP().toString().c_str());
    }
    else
    {
      Serial.println("\n[STA Mode]: Connection Failed! Switching to AP Mode...");
      serverStart("AP", ssid, password);
      return;
    }
  }
  else if (strcmp(mode, "AP") == 0)
  {
    WiFi.softAP(ssid, password);
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
    dnsServer.start(53, "*", IPAddress(192, 168, 4, 1));

    Serial.printf("\n[AP Mode]: Access Point Started!\n");
    Serial.printf("[AP Mode]: SSID: \"%s\"\n", ssid);
    Serial.printf("[AP Mode]: Password: \"%s\"\n", password);
    Serial.printf("[AP Mode]: AP IP Address: http://%s\n", WiFi.softAPIP().toString().c_str());
  }

  // 📌 Sayfaları tanımla
  serverCODLAI.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
                  {
      Serial.println("[Local Server]: Root URL Accessed!");
      request->send(200, "text/plain", "CODLAI Server is Running!"); });

  // 📌 404 Hatası
  serverCODLAI.onNotFound([](AsyncWebServerRequest *request)
                          {
      Serial.println("[Local Server]: Received an Unknown Request!");
      request->send(404, "text/plain", "Not Found"); });

  // 📌 **WebSocket Olaylarını Bağla**
  serverCODLAIWebSocket->onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
                                 {
      if (type == WS_EVT_CONNECT) {
          Serial.println("WebSocket Client Connected");
      } else if (type == WS_EVT_DISCONNECT) {
          Serial.println("WebSocket Client Disconnected");
      } });

  // 📌 WebSocket'i Sunucuya Bağla
  serverCODLAI.addHandler(serverCODLAIWebSocket);

  // 📌 **En son sunucuyu başlat!**
  serverCODLAI.begin();
  Serial.println("[Local Server]: Server Started! ✅");
}

inline void MINIBOT::serverCreateLocalPage(const char *url, const char *WEBPageScript, const char *WEBPageCSS, const char *WEBPageHTML, size_t bufferSize)
{
  // 📌 Sayfa içeriğini oluştur
  serverCODLAI.on(("/" + String(url)).c_str(), HTTP_GET, [WEBPageScript, WEBPageCSS, WEBPageHTML, bufferSize](AsyncWebServerRequest *request)
                  {
                    // Buffer boyutu kullanıcının belirttiği veya varsayılan değerle tanımlanır
                    char *buffer = new char[bufferSize];
                    int len = snprintf(buffer, bufferSize, WEBPageHTML, WEBPageScript, WEBPageCSS);

                    if ((size_t)len >= bufferSize)
                    {
                      Serial.println("[ERROR]: Buffer size insufficient, content truncated!");
                    }

                    request->send(200, "text/html", buffer);
                    delete[] buffer; // Dinamik olarak ayrılan belleği serbest bırakın
                  });

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.printf("[Local Server]: Page created at: http://%s/%s\n", WiFi.localIP().toString().c_str(), url);
  }
  else
  {
    Serial.printf("[Local Server]: Page created at: http://%s/%s\n", apIP.toString().c_str(), url);
  }
}

inline void MINIBOT::serverHandleDNS()
{
  dnsServer.processNextRequest();
}

inline void MINIBOT::serverContinue()
{
  if (WiFi.getMode() == WIFI_AP)
  {
    serverHandleDNS();
  }
}
#endif

/*********************************** Firebase Server Functions ***********************************/
#if defined(USE_FIREBASE)
// Initialize Firebase connection with SignUp Authentication
inline void MINIBOT::fbServerSetandStartWithUser(const char *projectURL, const char *secretKey, const char *userMail, const char *mailPass)
{
  firebaseData.setResponseSize(1024); // Optimize memory usage

  // Firebase Configuration Settings
  firebaseConfig.api_key = secretKey;
  firebaseConfig.database_url = projectURL;
  firebaseAuth.user.email = userMail;
  firebaseAuth.user.password = mailPass;

  // Zaman aşımı ayarları
  firebaseConfig.timeout.socketConnection = 10 * 1000; // 10 saniye bağlantı zaman aşımı

  // Token durumu izleme ayarı
  // firebaseConfig.token_status_callback = tokenStatusCallback;
  firebaseConfig.max_token_generation_retry = 5; // Daha fazla token yenileme denemesi

  // Wi-Fi bağlantısı kaybolduğunda otomatik yeniden bağlanma
  Firebase.reconnectWiFi(true);

  // Firebase başlat
  Firebase.begin(&firebaseConfig, &firebaseAuth);

  Serial.println("[Firebase]: Verifying user credentials...");
  uint8_t id_count = 0;
  while (firebaseAuth.token.uid == "" && id_count < 50)
  {
    Serial.print('.');
    delay(500);
    id_count++;
  }
  if (firebaseAuth.token.uid == "")
  {
    Serial.println("\n[ERROR]: Authentication timeout.");
  }
  else
  {
    if (Firebase.ready())
    {
      strncpy(uid, firebaseAuth.token.uid.c_str(), 128 - 1); // UID'yi kopyala ve taşma kontrolü yap
      uid[128 - 1] = '\0';                                   // Diziyi null karakter ile sonlandır
      Serial.print("\n[Info]: Doğrulanan Kimlik ID: ");
      Serial.println(uid);
    }
    else
    {
      Serial.print("[ERROR]: Sign-up failed. Reason: ");
      Serial.println(firebaseData.errorReason());
    }
  }
}

/*********************************** Firebase Write Functions ***********************************/

inline void MINIBOT::fbServerSetInt(const char *dataPath, int data)
{
  // Corrected function call
  if (Firebase.RTDB.setInt(&firebaseData, dataPath, data))
  {
    Serial.println("[SUCCESS]: Integer data sent successfully!");
  }
  else
  {
    Serial.print("[ERROR]: Failed to send integer data. ");
    Serial.printf("HTTP Code: %d\n", firebaseData.httpCode());
    Serial.println("Reason: " + firebaseData.errorReason());
  }
}

inline void MINIBOT::fbServerSetFloat(const char *dataPath, float data)
{
  if (Firebase.RTDB.setFloat(&firebaseData, dataPath, data))
  {
    Serial.println("[SUCCESS]: Float data sent successfully!");
  }
  else
  {
    Serial.print("[ERROR]: Failed to send float data. ");
    Serial.printf("HTTP Code: %d\n", firebaseData.httpCode());
    Serial.println("Reason: " + firebaseData.errorReason());
  }
}

inline void MINIBOT::fbServerSetString(const char *dataPath, String data)
{
  if (Firebase.RTDB.setString(&firebaseData, dataPath, data))
  {
    Serial.println("[SUCCESS]: String data sent successfully!");
  }
  else
  {
    Serial.print("[ERROR]: Failed to send string data. ");
    Serial.printf("HTTP Code: %d\n", firebaseData.httpCode());
    Serial.println("Reason: " + firebaseData.errorReason());
  }
}

inline void MINIBOT::fbServerSetDouble(const char *dataPath, double data)
{
  if (Firebase.RTDB.setDouble(&firebaseData, dataPath, data))
  {
    Serial.println("[SUCCESS]: Double data sent successfully!");
  }
  else
  {
    Serial.print("[ERROR]: Failed to send double data. ");
    Serial.printf("HTTP Code: %d\n", firebaseData.httpCode());
    Serial.println("Reason: " + firebaseData.errorReason());
  }
}

inline void MINIBOT::fbServerSetBool(const char *dataPath, bool data)
{
  if (Firebase.RTDB.setBool(&firebaseData, dataPath, data))
  {
    Serial.println("[SUCCESS]: Boolean data sent successfully!");
  }
  else
  {
    Serial.print("[ERROR]: Failed to send boolean data. ");
    Serial.printf("HTTP Code: %d\n", firebaseData.httpCode());
    Serial.println("Reason: " + firebaseData.errorReason());
  }
}

inline void MINIBOT::fbServerSetJSON(const char *dataPath, String data)
{
  FirebaseJson json;
  json.set(dataPath, data);

  if (Firebase.RTDB.setJSON(&firebaseData, dataPath, &json))
  {
    Serial.println("[SUCCESS]: JSON data sent successfully!");
  }
  else
  {
    Serial.print("[ERROR]: Failed to send JSON data. ");
    Serial.printf("HTTP Code: %d\n", firebaseData.httpCode());
    Serial.println("Reason: " + firebaseData.errorReason());
  }
}

/*********************************** Firebase Read Functions ***********************************/

inline int MINIBOT::fbServerGetInt(const char *dataPath)
{
  if (Firebase.RTDB.getInt(&firebaseData, dataPath))
  {
    Serial.println("[SUCCESS]: Integer data retrieved successfully!");
    return firebaseData.intData();
  }
  Serial.println("[ERROR]: Failed to retrieve integer data.");
  return -1;
}

inline float MINIBOT::fbServerGetFloat(const char *dataPath)
{
  if (Firebase.RTDB.getFloat(&firebaseData, dataPath))
  {
    Serial.println("[SUCCESS]: Float data retrieved successfully!");
    return firebaseData.floatData();
  }
  Serial.println("[ERROR]: Failed to retrieve float data.");
  return -1.0;
}

inline String MINIBOT::fbServerGetString(const char *dataPath)
{
  if (Firebase.RTDB.getString(&firebaseData, dataPath))
  {
    Serial.println("[SUCCESS]: String data retrieved successfully!");
    return firebaseData.stringData();
  }
  Serial.println("[ERROR]: Failed to retrieve string data.");
  return "";
}

inline double MINIBOT::fbServerGetDouble(const char *dataPath)
{
  if (Firebase.RTDB.getDouble(&firebaseData, dataPath))
  {
    Serial.println("[SUCCESS]: Double data retrieved successfully!");
    return firebaseData.doubleData();
  }
  Serial.println("[ERROR]: Failed to retrieve double data.");
  return -1.0;
}

inline bool MINIBOT::fbServerGetBool(const char *dataPath)
{
  if (Firebase.RTDB.getBool(&firebaseData, dataPath))
  {
    Serial.println("[SUCCESS]: Boolean data retrieved successfully!");
    return firebaseData.boolData();
  }
  Serial.println("[ERROR]: Failed to retrieve boolean data.");
  return false;
}

inline String MINIBOT::fbServerGetJSON(const char *dataPath)
{
  if (Firebase.RTDB.getJSON(&firebaseData, dataPath))
  {
    Serial.println("[SUCCESS]: JSON data retrieved successfully!");
    return firebaseData.jsonString();
  }
  Serial.println("[ERROR]: Failed to retrieve JSON data.");
  return "{}";
}
#endif

/*********************************** ESP-NOW ***********************************/
#if defined(USE_ESPNOW)
inline void MINIBOT::initESPNow()
{
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != 0)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Serial.println("ESP-NOW Initialized");
}

inline void MINIBOT::setWiFiChannel(int channel)
{
  wifi_set_channel(channel);
}

inline void MINIBOT::sendESPNow(uint8_t *macAddr, uint8_t *data, int len)
{
  if (!esp_now_is_peer_exist(macAddr))
  {
    if (esp_now_add_peer(macAddr, ESP_NOW_ROLE_SLAVE, 1, NULL, 0) != 0)
    {
      Serial.println("Failed to add peer");
      return;
    }
  }

  int result = esp_now_send(macAddr, data, len);
  if (result == 0)
  {
    Serial.println("Sent with success");
  }
  else
  {
    Serial.println("Error sending the data");
  }
}

inline void MINIBOT::registerOnRecv(esp_now_recv_cb_t cb)
{
  esp_now_register_recv_cb(cb);
}
#endif

/*********************************** Email ***********************************/
#if defined(USE_EMAIL)
inline void MINIBOT::sendEmail(String smtpHost, int smtpPort, String authorEmail, String authorPassword, String recipientEmail, String subject, String messageStr)
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi not connected!");
    return;
  }

  SMTPSession smtp;
  Session_Config config;
  config.server.host_name = smtpHost;
  config.server.port = smtpPort;
  config.login.email = authorEmail;
  config.login.password = authorPassword;
  config.login.user_domain = "";

  SMTP_Message message;
  message.sender.name = "MINIBOT";
  message.sender.email = authorEmail;
  message.subject = subject;
  message.addRecipient("User", recipientEmail);
  message.text.content = messageStr.c_str();

  smtp.connect(&config);
  if (!MailClient.sendMail(&smtp, &message))
    Serial.println("Error sending Email, " + smtp.errorReason());
  else
    Serial.println("Email sent successfully!");
}
#endif

/*********************************** Weather ***********************************/
#if defined(USE_WEATHER)
#include <ArduinoJson.h>
inline String MINIBOT::getWeather(String city, String apiKey)
{
  if (WiFi.status() != WL_CONNECTED)
    return "WiFi Error";

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  String url;

  if (apiKey == "" || apiKey == "YOUR_API_KEY") {
      Serial.println("[Weather]: Using wttr.in (Free Service)...");
      
      #if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
      client.setHandshakeTimeout(20000); 
      #endif

      url = "https://wttr.in/" + city + "?format=%t+%C";
      
      Serial.println("[Weather]: Requesting URL: " + url);
      
      http.begin(client, url);
      #if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
      http.setConnectTimeout(20000); 
      #endif
      
      http.setUserAgent("curl/7.68.0"); 

      Serial.println("[Weather]: Sending GET request...");
      int httpCode = http.GET();
      Serial.println("[Weather]: GET request finished. HTTP Code: " + String(httpCode));

      if (httpCode > 0) {
          String payload = http.getString();
          http.end();
          payload.trim(); 
          Serial.println("[Weather]: Data received: " + payload);
          return payload;
      } else {
          String errorStr = http.errorToString(httpCode);
          http.end();
          Serial.println("[Weather]: Error: " + errorStr);
          return "Error: " + errorStr;
      }
  } 
  else {
      url = "http://api.openweathermap.org/data/2.5/weather?q=" + city + "&appid=" + apiKey + "&units=metric";

      http.begin(client, url);
      int httpCode = http.GET();

      if (httpCode > 0)
      {
        String payload = http.getString();
        JsonDocument doc; 
        deserializeJson(doc, payload);
        float temp = doc["main"]["temp"];
        String weather = doc["weather"][0]["description"];
        http.end();
        return String(temp) + "C, " + weather;
      }
      else
      {
        http.end();
        return "Error (OWM)";
      }
  }
}
#endif

/*********************************** Wikipedia ***********************************/
#if defined(USE_WIKIPEDIA)
#include <ArduinoJson.h>
inline String MINIBOT::getWikipedia(String query, String lang)
{
  if (WiFi.status() != WL_CONNECTED)
    return "WiFi Error";

  WiFiClientSecure client;
  client.setInsecure(); 
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  client.setHandshakeTimeout(20000); 
#endif

  HTTPClient http;
  String url = "https://" + lang + ".wikipedia.org/api/rest_v1/page/summary/" + query;

  Serial.println("[Wikipedia]: Requesting URL: " + url);
  
  http.begin(client, url);
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  http.setConnectTimeout(20000); 
#endif
  http.setUserAgent("curl/7.68.0"); 

  int httpCode = http.GET();

  if (httpCode > 0)
  {
    String payload = http.getString();
    JsonDocument doc; 
    deserializeJson(doc, payload);
    
    if (doc.containsKey("extract")) {
        String extract = doc["extract"].as<String>();
        http.end();
        return extract;
    } else {
        http.end();
        return "No Summary Found";
    }
  }
  else
  {
    String errorStr = http.errorToString(httpCode);
    http.end();
    Serial.println("[Wikipedia]: Error: " + errorStr);
    return "Error: " + errorStr;
  }
}
#endif

/*********************************** Telegram ***********************************/
#if defined(USE_TELEGRAM)
inline void MINIBOT::sendTelegram(String token, String chatId, String message)
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi not connected!");
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  
  // URL Encode the message manually for basic characters or use a library if needed. 
  // For simplicity, we replace spaces with %20.
  message.replace(" ", "%20");
  
  String url = "https://api.telegram.org/bot" + token + "/sendMessage?chat_id=" + chatId + "&text=" + message;
  
  http.begin(client, url);
  int httpCode = http.GET();
  
  if (httpCode > 0) {
    Serial.println("Telegram Message Sent!");
  } else {
    Serial.println("Error sending Telegram: " + http.errorToString(httpCode));
  }
  http.end();
}
#endif

/*********************************** IFTTT ***********************************/
#if defined(USE_IFTTT)
inline bool MINIBOT::triggerIFTTTEvent(const String &eventName, const String &webhookKey, const String &jsonPayload)
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("[IFTTT]: WiFi not connected!");
    return false;
  }

  if (eventName.length() == 0 || webhookKey.length() == 0)
  {
    Serial.println("[IFTTT]: Event name or key missing.");
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  String url = "https://maker.ifttt.com/trigger/" + eventName + "/with/key/" + webhookKey;

  Serial.println("[IFTTT]: Triggering '" + eventName + "'...");

  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  int httpCode = http.POST(jsonPayload);

  if (httpCode > 0)
  {
    Serial.println("[IFTTT]: HTTP " + String(httpCode));
    String payload = http.getString();
    Serial.println("[IFTTT]: Response => " + payload);
  }
  else
  {
    Serial.println("[IFTTT]: Error => " + http.errorToString(httpCode));
  }

  http.end();
  return httpCode == HTTP_CODE_OK;
}
#endif

#else
#error "MINIBOT sadece ESP8266 için desteklenmektedir."
#endif

#if defined(USE_ESPNOW)
// Initialize static member
inline MINIBOT* MINIBOT::_instance = nullptr;
#endif

#endif
