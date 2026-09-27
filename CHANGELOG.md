# Changelog

# CODLAI ERA (New Models)

## [Unreleased]

## [1.5.0] - 2026-09-27
### Added
- **Basit ESP-NOW mesajlasma** (cocuklar/blok kod icin): `espNowBegin(channel=1)`, `espNowSendText(text)`, `espNowSendNumber(name, value)`, `espNowAvailable()`, `espNowReadText()`, `espNowReadName()`, `espNowReadNumber()`. `CodlaiESPNowMessage` yapisina `char text[32]` ve `float value` alanlari eklendi (Kol/Arac kontrolunu bozmadan) - ayni surumdeki tum CODLAI kartlari arasinda uyumlu.
- **Melodi**: `buzzerPlayNote(note, durationMs)`, `buzzerPlayMelody(melodyId)` (1=Dogum Gunu, 2=Twinkle Twinkle, 3=Jingle Bells, 4=Baslangic Melodisi, 5=Daha Dun Annemizin [DOGRULANMAMIS, basitlestirilmis yer tutucu]), `buzzerSetTempo(bpm)`.
- **NeoPixel**: `moduleSmartLEDFill(r,g,b)`, `moduleSmartLEDClear()`, `moduleSmartLEDSetBrightness(0-255)`, `moduleSmartLEDBlink(r,g,b,times,ms)`, `moduleSmartLEDBreathe(r,g,b,ms)`.
- Yeni ornekler: `MINIBOT_ESPNOW_Simple_Messaging_Example.ino`, `MINIBOT_Buzzer_Melody_Example.ino`, `MINIBOT_NeoPixel_Effects_Example.ino`.

### Fixed
- Kok dizindeki `platformio.ini`'de `env:MINIBOT` icin `Adafruit NeoPixel` bagimliligi eksikti (sadece IOTBOT ortaminda tanimliydi) - `USE_NEOPIXEL` ile MINIBOT ortaminda derleme "Adafruit_NeoPixel.h: No such file or directory" hatasi veriyordu. lib_deps'e eklendi.

## [1.4.1] - 2026-09-27
### Fixed
- `otaBegin()` icinde parola if/else zincirinden sonra fazladan bir `else { ArduinoOTA.setPassword("1234"); }` bloğu vardi - bu "else without a previous if" derleme hatasina yol acip `USE_OTA` tanimlayan HER sketch'in derlenmesini engelliyordu. Fazla blok kaldirildi. (Editor ajaninin derleme servisi testinde bulundu.)

## [1.4.0] - 2026-09-27
### Added
- Yeni "3-Projects" ornek klasoru: kablosuz haberlesme gerektirmeyen, tek basina calisan, gercek hayattan basit proje ornekleri (onboard buzzer/LED + tek bir P-modulu kullanir).
- `MINIBOT_Parking_Assistant_Example.ino` - ultrasonik mesafe sensoru + onboard buzzer (mesafeye gore hizlanan bip) + mavi LED, araba park sensoru mantigi.
- `MINIBOT_PIR_Security_Alarm_Example.ino` - PIR hareket sensoru + onboard buzzer/LED alarmi + B1 butonuyla susturma.
- `MINIBOT_Magnetic_Door_Alarm_Example.ino` - manyetik kapi/pencere sensoru, B1 ile kurma/etkisizlestirme (arm/disarm), acilinca buzzer/LED alarmi.
- `MINIBOT_Vibration_Shock_Alarm_Example.ino` - titresim/darbe sensoru, B1 ile kurma/etkisizlestirme, darbe algilaninca kisa alarm patlamasi.

- Yeni ornek: `MINIBOT_ESPNOW_Fan_Control_Reactive_Example.ino` - bir IOTBOT'un yayinladigi DHT sicaklik verisine gore role modulunu (vantilator) otomatik acar/kapatir ("kablosuz otomatik vantilator").

### Fixed
- `initESPNow()` icinde kosulsuz `WiFi.mode(WIFI_STA)` cagrisi, ayni sketch'te onceden acilmis bir AP'yi (ornegin bir web sunucusu/OTA icin `softAP()`) sessizce dusuruyordu. Artik mevcut mod AP ya da AP_STA ise `WIFI_AP_STA`'ya geciliyor, AP kapatilmiyor.

## [1.3.0] - 2026-09-26
### Added
- `serverOnRequest(url, callback)`: `serverCreateLocalPage` SADECE sabit/statik bir HTML sayfasi render eder; bu yeni fonksiyon bir adrese istek geldiginde GERCEKTEN kod calistirmaniza (bir GPIO'yu tetiklemenize) izin verir.
- Yeni ornek: `MINIBOT_IoTBot_ESPNOW_Pair_Example.ino` - router/WiFi agi olmadan (ESP-NOW ile) bir IOTBOT ile dogrudan, iki yonlu haberlesme; gercek donanimda (iki kart, canli MAC adresleriyle) dogrulandi.
- Yeni baslangic seviyesi ornekler: `MINIBOT_WiFi_Simple_Status_Example.ino` (MAC/sunucu gerekmeyen en basit WiFi baglanma ornegi), `MINIBOT_ESPNOW_Broadcast_Simple_Example.ino` (MAC adresi bilmeden yayin/broadcast ile herhangi bir CODLAI kartina konusma) - egitim mufredati icin "once bunu dene" niteliginde.
- Yeni ornek: `MINIBOT_ESPNOW_NightLight_Reactive_Example.ino` - bir IOTBOT'un yayinladigi isik sensoru verisine gore kendi LED'ini otomatik acar/kapatir ("kablosuz gece lambasi").
- Yeni ornek: `MINIBOT_IoTBot_SmartLED_Remote_Example.ino` - B1 butonuyla uzaktaki bir IOTBOT'un akilli LED efektini degistirir.

### Fixed
- **ESP-NOW gonderme hatasi**: `initESPNow()` icinde `esp_now_set_self_role()` hic cagrilmiyordu; ESP8266'nin klasik `espnow.h` API'si bu olmadan `esp_now_send()`'i sessizce basarisiz kiliyordu ("Error sending the data" - gercek donanimda IoTBot ile ESP-NOW eslesme testi sirasinda tespit edildi). `ESP_NOW_ROLE_COMBO` ile duzeltildi.
- `USE_ESPNOW` (ve tek basina digger bazi bayraklar) tanimlandiginda `WiFi.h`'in hic include edilmedigi bir sira sorunu duzeltildi (bkz. CODLAI_IOTBOT v1.5.0'daki ayni duzeltme).

## [1.2.0] - 2026-09-25
### Added
- NTP time helpers: `ntpSync`, `ntpIsTimeValid`, `ntpGetEpoch`, `ntpGetDateTimeString`.
- CRC-protected EEPROM record helpers: `eepromCrc32`, `eepromWriteRecord`, `eepromReadRecord`.
- New advanced example: `MINIBOT_NTP_Time_Advanced_Example.ino` (TR/EN).
- `examples/MINIBOT_Musteri_Karsilama.cpp`'e Turkce/Ingilizce dil destegi eklendi: acilista B1 1.5sn basili tutulursa Ingilizce, birakilirsa (varsayilan) Turkce; secim LED yanip-sonmesiyle de teyit edilir.

### Fixed
- `library.json`'daki `dependencies` alani artik gercekte kullanilan kutuphaneleri gosteriyor (eski `ESPAsyncWebServer ^1.2.3` / `ESPAsyncTCP` / `AsyncTCP` uclusu yerine `mathieucarbou/ESPAsyncWebServer ^3.6.0` ve `bblanchon/ArduinoJson ^7.1.0`).

## [1.1.5] - 2026-02-04
### Added
- OTA helpers: `otaBegin`, `otaHandle` (requires `USE_OTA`).
- New advanced example: `MINIBOT_OTA_Update_Example.ino` (TR/EN).

## [1.1.4] - 2025-12-20
### Added
- Extended EEPROM helpers: `eepromBegin/Commit/End`, byte/int32/uint32/float/string/bytes read-write and region clear.
- New advanced example: `MINIBOT_EEPROM_Advanced_Example.ino` (TR/EN).

### Changed
- EEPROM int (legacy) helpers now lazy-initialize EEPROM to reduce common runtime issues.

## [1.1.3] - 2025-12-18
### Added
- Refreshed the ESP-NOW, email, Telegram, weather and Wikipedia advanced examples with bilingual commentary so the new helpers and connection tips are easy to follow.

## [1.1.2] - 2025-03-09
### Fixed
- Guarded HTTP client handshake/connect timeout helpers so ESP8266 builds compile with the stock BearSSL/HTTPClient interfaces.

## [1.1.0] - 2025-03-09
### Added
- `triggerIFTTTEvent` helper for Maker Webhook automations.
- Advanced example: `MINIBOT_IFTTT_Webhook_Example.ino`.

### Updated
- Documentation and metadata to highlight the IFTTT workflow.

## [1.0.0] - 2025-03-04
### Added
- **Rebranding**: Transitioned from CODROB to CODLAI.
- Standardized library structure.
- Added `serialStart` and `serialWrite` wrappers.
- Updated examples to use library wrappers.
- Initial Release for PlatformIO and Arduino IDE.

---

# CODROB ERA (Legacy Models)

## [1.6.4] - 2025-02-28
### Added
- Tüm modüller için config dosyası kaldırıldı. Ortak kütüpahaneler devrede. 

## [1.6.2] - 2025-02-21
### Added
- Config dosyası eklendi. 

## [1.5.6] - 2025-02-20
### Added
- Trafik iışıkları için tekli modül eklendi. 
- Arduino uyumluluğu için library.properties eklendi.
- esphome/ESPAsyncWebServer-esphome yerine mathieucarbou/ESPAsyncWebServer eklendi. 
- Keywords listesi güncellendi. 
- Gerekli uygulamalara define eklendi. uygulamaya gore kütüphane aktifleşecek hale getirildi.

### Fixed
- CPP ve H dosyası arduıno ile uyumlu hale getirildi. 

## [1.5.1] - 2025-02-13
### Fixed
- Firebase ve Wifi örnek uygulamalrındaki eksiklikler düzeltildi.  

## [1.4.5] - 2025-02-11
### Fixed
- Firebase ve Wifi örnek uygulamalrındaki eksiklikler düzeltildi.  

## [1.4.0] - 2025-02-08
### Added
- Örnek uygulamalar düzeltildi.  

## [1.3.0] - 2025-02-04
### Added
- Firebase kütüphaneleri eklendi. 

### Fixed
- EEPROM fonksiyonları düzeltildi. 

## [1.2.3] - 2025-01-31
### Added
- DHT için Fahreneght kodlaarı eklendi. 
- Wifi Kütüphaneleri ve fonskiyonları ekleni 
- Local server fonksiyonaları eklendi. 
- EEPROM fonksiyonları ekledi.

### Fixed
- Servo motor ayarları optimize edildi. 
- Açıklamalar düzeltildi. 

## [1.2.2] - 2025-01-31
### Fixed
- DHT modülü için ifonksiyon isim düzeltmeleri yapıldı. 
- Seriport fonksiyornları düzeltildi. 

## [1.2.1] - 2025-01-31
### Added
- DHT için Fahreneght kodlaarı eklendi. 

### Fixed
- Servo motor ayarları optimize edildi. 

## [1.2.0] - 2025-01-30
### Added
- Eksik olan tüm kütüphaneler eklendi, örnek uygulamalar güncellendi. 

### Fixed
- Servo ve IR okuyucu modüllerindeki buglar düzeltildi. 
## [1.0.1] - 2024-12-29
### Added
- MINIBOT sınıfı ve temel sensör işlevleri eklendi.

---

## [1.0.0] - 2024-12-28
### Added
- İlk sürüm yayımlandı.

