/*
  CODLAI MINIBOT - EEPROM Advanced Example

  TR:
  - Bu örnek, MINIBOT kütüphanesindeki EEPROM yardımcı fonksiyonlarının kullanımını gösterir.
  - EEPROM (Flash tabanlı) kalıcı hafızadır. Sık yazma işlemleri Flash ömrünü azaltabilir.
  - Adres planlamasını dikkatli yapın (aynı adres aralığını farklı veriler için kullanmayın).

  EN:
  - This example demonstrates the EEPROM helper functions in the MINIBOT library.
  - EEPROM (Flash-backed) is persistent storage. Excessive writes can reduce flash lifetime.
  - Plan your addresses carefully (do not overlap address ranges).
*/

#include <MINIBOT.h>

MINIBOT minibot;

// EEPROM address layout (example) / EEPROM adres planı (örnek)
// 0..1   : int16 (legacy eepromWriteInt)
// 10..13 : int32
// 20..23 : float
// 30..   : string (len+data)

void setup()
{
  minibot.serialStart(115200);
  delay(200);

  // TR: EEPROM'u başlatın. ESP8266 tarafında EEPROM.commit() için begin gereklidir.
  // EN: Initialize EEPROM. On ESP8266, EEPROM.begin() is required for commit().
  bool ok = minibot.eepromBegin(512);
  minibot.serialWrite(ok ? "[EEPROM] Ready" : "[EEPROM] Begin failed");

  // TR: 16-bit (2 byte) legacy int yaz/oku
  // EN: Write/read 16-bit (2 byte) legacy int
  minibot.eepromWriteInt(0, 1234);
  int v16 = minibot.eepromReadInt(0);
  minibot.serialWrite("v16 = ");
  minibot.serialWrite(v16);

  // TR/EN: int32 yaz/oku
  minibot.eepromWriteInt32(10, 123456789);
  int32_t v32 = minibot.eepromReadInt32(10, -1);
  minibot.serialWrite("v32 = ");
  minibot.serialWrite((long)v32);

  // TR/EN: float yaz/oku
  minibot.eepromWriteFloat(20, 36.5f);
  float vf = minibot.eepromReadFloat(20, -1.0f);
  minibot.serialWrite("vf = ");
  minibot.serialWrite(vf);

  // TR/EN: string yaz/oku (len + bytes)
  minibot.eepromWriteString(30, String("Merhaba MINIBOT / Hello MINIBOT"), 64);
  String s = minibot.eepromReadString(30, 64);
  minibot.serialWrite("str = ");
  minibot.serialWrite(s);

  // TR/EN: byte array yaz/oku
  uint8_t dataOut[4] = {1, 2, 3, 4};
  minibot.eepromWriteBytes(120, dataOut, sizeof(dataOut));

  uint8_t dataIn[4] = {0};
  minibot.eepromReadBytes(120, dataIn, sizeof(dataIn));
  minibot.serialWrite("bytes = ");
  for (size_t i = 0; i < sizeof(dataIn); i++)
  {
    Serial.print(dataIn[i]);
    Serial.print(i + 1 < sizeof(dataIn) ? "," : "\n");
  }

  // TR/EN: Recommended (CRC + versioned record)
  // Layout: [magic][version][len][crc32][bytes...]
  // This helps detect corrupted/invalid data and lets you version your stored struct.
  struct ExampleConfig
  {
    uint32_t bootCount;
    char name[16];
  };

  const int CONFIG_ADDR = 200;

  ExampleConfig cfgOut;
  cfgOut.bootCount = 1;
  snprintf(cfgOut.name, sizeof(cfgOut.name), "%s", "MINIBOT");

  bool wrec = minibot.eepromWriteRecord(CONFIG_ADDR, (const uint8_t *)&cfgOut, (uint16_t)sizeof(cfgOut), 1);
  minibot.serialWrite(wrec ? "record write = OK" : "record write = FAIL");

  ExampleConfig cfgIn;
  uint16_t outLen = 0;
  uint16_t outVer = 0;
  bool rrec = minibot.eepromReadRecord(CONFIG_ADDR, (uint8_t *)&cfgIn, (uint16_t)sizeof(cfgIn), &outLen, &outVer);

  if (rrec)
  {
    Serial.print("record ver=");
    Serial.print(outVer);
    Serial.print(" len=");
    Serial.print(outLen);
    Serial.print(" bootCount=");
    Serial.print((unsigned long)cfgIn.bootCount);
    Serial.print(" name=");
    Serial.println(cfgIn.name);
  }
  else
  {
    Serial.println("record read = FAIL (magic/len/crc mismatch)");
  }

  // TR: İsterseniz temizleme yapabilirsiniz (dikkat: tüm alanı sıfırlar)
  // EN: You can clear EEPROM if needed (warning: wipes region)
  // minibot.eepromClear(0, 512, 0xFF);
}

void loop()
{
  // TR/EN: No loop actions needed
}
