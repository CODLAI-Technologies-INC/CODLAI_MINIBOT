# Changelog

# CODLAI ERA (New Models)

## [Unreleased]
### Added
- NTP time helpers: `ntpSync`, `ntpIsTimeValid`, `ntpGetEpoch`, `ntpGetDateTimeString`.
- CRC-protected EEPROM record helpers: `eepromCrc32`, `eepromWriteRecord`, `eepromReadRecord`.
- New advanced example: `MINIBOT_NTP_Time_Advanced_Example.ino` (TR/EN).

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

