#pragma once

#include <Arduino.h>

// ============================================================
//  BEYAZ ÇİZGİ TAKİP ROBOTU - AYAR DOSYASI
//  Arduino Uno / Nano + 2 kanal motor sürücü (L298N) + 5x TCRT5000
//
//  Sürüş: 4 motor, diferansiyel (tank) sürüş.
//  Sol taraftaki 2 motor bir kanala, sağdaki 2 motor diğer kanala
//  paralel bağlanır.
// ============================================================

// ------------------------------------------------------------
// 1) Sensör dizisi
// ------------------------------------------------------------
constexpr uint8_t SENSOR_COUNT = 5;

// Sensörlerin Arduino pinleri - ROBOTUN ÖNÜNDEN BAKINCA soldan sağa.
constexpr uint8_t SENSOR_PINS[SENSOR_COUNT] = {A0, A1, A2, A3, A4};

// Çizgi sensörün altındayken analog değer DÜŞÜYORSA true.
// Çoğu TCRT5000 modülünde AO çıkışı beyaz yüzeyde YÜKSELİR; bu durumda false.
// Otomatik algılama açıksa bu değer yalnızca yedek/başlangıç değeridir.
constexpr bool LINE_READS_LOW = false;
constexpr bool AUTO_DETECT_LINE_POLARITY = true;

// Konum hesabı için ağırlıklar (sol negatif, sağ pozitif).
// 3 sensör kullanacaksanız {-1, 0, 1}, 7 sensör için -3..+3 verin.
constexpr float SENSOR_WEIGHTS[SENSOR_COUNT] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};

// Kalibrasyon süresi. Bu süre boyunca sensör dizisini yavaşça
// çizginin üzerinde sağa-sola süpürün.
constexpr uint32_t CALIBRATION_MS = 6000;

// Kalibrasyonda bir sensörün okuyabildiği en büyük aralık bunun
// altındaysa sensör "bağlı değil / kontrast yok" kabul edilir.
constexpr int MIN_SENSOR_SPAN = 80;

// Normalize edilmiş 0..1 değeri bunun altındaysa sensör "boş" sayılır.
constexpr float SENSOR_FLOOR = 0.10f;

// Otomatik polarite algılamada, bir sensörün "çizgi gördü" sayılması için
// medyan değerden en az bu kadar sapması gerekir (ham ADC değeri).
constexpr int POLARITY_VOTE_MARGIN = 30;

// Çizgi var sayılması için aktif sensör değerlerinin toplamı.
constexpr float LINE_PRESENT_THRESHOLD = 0.35f;

// ------------------------------------------------------------
// 2) Motor sürücü pinleri (L298N)
// ------------------------------------------------------------
// Sol kanal: ENA + IN1 + IN2 -> 2 sol motor (paralel)
constexpr uint8_t LEFT_PWM_PIN  = 5;
constexpr uint8_t LEFT_IN1_PIN  = 7;
constexpr uint8_t LEFT_IN2_PIN  = 8;

// Sağ kanal: ENB + IN3 + IN4 -> 2 sağ motor (paralel)
constexpr uint8_t RIGHT_PWM_PIN = 6;
constexpr uint8_t RIGHT_IN3_PIN = 9;
constexpr uint8_t RIGHT_IN4_PIN = 10;

// Bir tarafın motorları ters dönüyorsa true yapın.
constexpr bool LEFT_MOTOR_INVERTED  = false;
constexpr bool RIGHT_MOTOR_INVERTED = false;

// ------------------------------------------------------------
// 3) Buton ve LED
// ------------------------------------------------------------
// Butonun bir ucu GND'ye, diğer ucu bu pine. Buton isteğe bağlıdır.
constexpr uint8_t CALIBRATION_BUTTON_PIN = 2;

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif
constexpr uint8_t STATUS_LED_PIN = LED_BUILTIN;

// Açılışta kalibrasyonun otomatik başlaması için beklenecek süre.
// 0 yaparsanız yalnızca buton / seri port komutu ile başlar.
constexpr uint32_t AUTO_START_DELAY_MS = 5000;

// Kalibrasyondan sonra motorların çalışması için beklenen süre.
constexpr uint32_t START_DELAY_MS = 2000;

// ------------------------------------------------------------
// 4) Hız ve PID
// ------------------------------------------------------------
constexpr int BASE_SPEED = 120;  // Düz giderken her kanalın PWM'i (0-255)
constexpr int MIN_PWM     = 35;  // Motorların dönmeye başladığı en düşük PWM
constexpr int MAX_PWM     = 235; // Güvenlik üst sınırı

// Hata -1..+1 arasındadır (sağ pozitif). Çıkış PWM cinsindendir.
// Ayar sırası: önce Kp, sonra Kd. Ki çoğu zaman 0 kalabilir.
constexpr float KP = 100.0f;
constexpr float KI = 0.0f;
constexpr float KD = 12.0f;

constexpr float INTEGRAL_LIMIT    = 60.0f; // I teriminin PWM sınırı
constexpr float DERIVATIVE_FILTER = 0.80f; // 0 = filtre yok, 0.9 = çok yumuşak

// ------------------------------------------------------------
// 5) Çizgi kaybolunca arama davranışı
// ------------------------------------------------------------
constexpr int SEARCH_SPEED = 100;           // Yerinde dönme PWM'i
constexpr uint32_t LINE_LOST_TIMEOUT_MS = 1500; // Bu süre sonunda dur

// ------------------------------------------------------------
// 6) Kontrol döngüsü
// ------------------------------------------------------------
constexpr uint32_t CONTROL_PERIOD_US = 5000; // 200 Hz
constexpr uint32_t DEBUG_PERIOD_MS = 200;    // Seri port çıktı aralığı
