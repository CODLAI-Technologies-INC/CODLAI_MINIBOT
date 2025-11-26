/*
 * 📌 Firebase özelliklerini kullanabilmek için, MINIBOT_Config.h dosyasında 'USE_FIREBASE' tanımının başındaki
 * yorum satırlarını (//) kaldırın. Aksi takdirde, Firebase işlevleri devre dışı kalacaktır.
 *
 * 📌 To enable Firebase features, remove the comment (//) before the 'USE_FIREBASE' definition in
 * MINIBOT_Config.h. Otherwise, Firebase functions will be disabled.
 */

#define USE_FIREBASE
#include <MINIBOT.h>

MINIBOT minibot;

// 🔑 Firebase Configuration | 🔑 Firebase Yapılandırması
#define FIREBASE_PROJECT_URL "https://deneme-961c3-default-rtdb.firebaseio.com/" // Enter the Firebase server URL here. | Buraya Firebase sunucusunun linkini yazınız.
#define FIREBASE_API_KEY "AIzaSyA5wTuxzls6IxQct-G2_hf0i5LWF3ulsec"               // Enter the Firebase server API key here. | Buraya Firebase sunucusunun API anahtarını yazınız.

// 📧 Firebase User Authentication | 📧 Firebase Kullanıcı Kimlik Doğrulama
#define USER_EMAIL "deneme123@gmail.com" // Enter the email address used to create the Firebase user here. | Buraya Firebase sunucusunda oluşturduğunuz kullanıcı e-posta adresini yazınız.
#define USER_PASSWORD "dene123"          // Enter the password for the Firebase user here. | Buraya Firebase sunucusunda oluşturduğunuz kullanıcı şifresini yazınız.

// 📡 WiFi Settings | 📡 WiFi Ayarları
#define WIFI_SSID "WIFI_SSID" // Enter the name of the Wi-Fi network you want to connect to here. | Buraya bağlanmak istediğiniz Wi-Fi ağının adını yazınız.
#define WIFI_PASS "WiFi_PASS" // Enter the password of the Wi-Fi network you want to connect to here. | Buraya bağlanmak istediğiniz Wi-Fi ağının şifresini yazınız.

void setup()
{
    minibot.serialStart(115200); // Start Serial Communication
    Serial.println("🚀 MiniBot Firebase Example Starting...");

    // 🔗 Step 1: Connect to Wi-Fi
    minibot.wifiStartAndConnect(WIFI_SSID, WIFI_PASS);

    if (!minibot.wifiConnectionControl())
    {
        minibot.serialWrite("Device Stopped!");
        while (true) // Endless loop | Sonsuz döngü
        {
            if (minibot.wifiConnectionControl())
            {
                minibot.serialWrite("Connection Success! Reworking..");
                break; // Exit the loop, continue.| Döngüden çık, devam et.
            }
        }
    }

    // 🔥 Step 2: Initialize Firebase
    minibot.fbServerSetandStartWithUser(FIREBASE_PROJECT_URL, FIREBASE_API_KEY, USER_EMAIL, USER_PASSWORD);

    // ✍️ Step 3: Send Data to Firebase
    minibot.fbServerSetInt("/device/temperature", 25);
    minibot.fbServerSetString("/device/status", "Online");
    minibot.fbServerSetBool("/device/active", true);

    Serial.println("📤 Data sent to Firebase.");
}

void loop()
{
    // 🔄 Step 5: Read Data from Firebase
    int temp = minibot.fbServerGetInt("/device/temperature");
    String status = minibot.fbServerGetString("/device/status");
    bool active = minibot.fbServerGetBool("/device/active");

    // 🖨️ Display Data on Serial Monitor
    Serial.print("🌡️ Temperature: ");
    Serial.println(temp);

    Serial.print("💡 Status: ");
    Serial.println(status);

    Serial.print("🔋 Active: ");
    Serial.println(active ? "Yes" : "No");

    delay(60000); // Refresh every 5 seconds
}