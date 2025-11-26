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

#if defined(USE_WEATHER) || defined(USE_WIKIPEDIA) || defined(USE_TELEGRAM)
#ifndef USE_WIFI
#define USE_WIFI
#endif
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
  void eepromWriteInt(int address, int value);
  int eepromReadInt(int address);

  /*********************************** WiFi  ***********************************
   */
#if defined(USE_WIFI)
  void wifiStartAndConnect(const char *ssid, const char *pass);
  bool wifiConnectionControl();
  String wifiGetMACAddress();
  String wifiGetIPAddress();
#endif

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
inline void MINIBOT::eepromWriteInt(int address, int value) // EEPROM'a güvenli bir şekilde int türünde veri yazmak için fonksiyon
{
  byte highByte = highByte(value); // int'in yüksek baytını al
  byte lowByte = lowByte(value);   // int'in düşük baytını al

  EEPROM.write(address, highByte);    // İlk baytı EEPROM'a yaz
  EEPROM.write(address + 1, lowByte); // İkinci baytı EEPROM'a yaz
  EEPROM.commit();                    // Değişiklikleri kaydetmek için commit işlemi yapılmalıdır
}

inline int MINIBOT::eepromReadInt(int address) // EEPROM'dan int türünde veri okumak için fonksiyon
{
  byte highByte = EEPROM.read(address);    // İlk baytı oku
  byte lowByte = EEPROM.read(address + 1); // İkinci baytı oku
  return word(highByte, lowByte);          // Yüksek ve düşük baytları birleştirerek int değeri oluştur
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

                    if (len >= bufferSize)
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
      
      client.setHandshakeTimeout(20000); 

      url = "https://wttr.in/" + city + "?format=%t+%C";
      
      Serial.println("[Weather]: Requesting URL: " + url);
      
      http.begin(client, url);
      http.setConnectTimeout(20000); 
      
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
  client.setHandshakeTimeout(20000); 

  HTTPClient http;
  String url = "https://" + lang + ".wikipedia.org/api/rest_v1/page/summary/" + query;

  Serial.println("[Wikipedia]: Requesting URL: " + url);
  
  http.begin(client, url);
  http.setConnectTimeout(20000); 
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

#else
#error "MINIBOT sadece ESP8266 için desteklenmektedir."
#endif

#if defined(USE_ESPNOW)
// Initialize static member
inline MINIBOT* MINIBOT::_instance = nullptr;
#endif

#endif
