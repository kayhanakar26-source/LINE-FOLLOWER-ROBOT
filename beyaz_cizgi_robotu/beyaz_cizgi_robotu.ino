// ============================================================
//  BEYAZ ÇİZGİ TAKİP ROBOTU
//  4 motor (2 kanal diferansiyel sürüş), 5 kanallı TCRT5000 dizisi
//
//  Arduino IDE:
//    1) Board: Arduino Uno (veya Nano)
//    2) Bu .ino dosyasını açıp yükleyin.
//
//  Seri port: 115200 baud
//    c / C : yeniden kalibrasyon
//    d / D : hata ayıklama çıktısını aç / kapat
//
//  Çalışma mantığı:
//    - Açılışta 5 saniye sonra kalibrasyon başlar (veya butona basın).
//    - Sensörler çizgiyi gördüğü sürece PID ile çizgi ortalanır.
//    - Çizgi kaybolursa son görülen tarafa doğru yerinde döner.
//    - 1,5 saniye içinde bulamazsa güvenlik için durur.
// ============================================================

#include "config.h"
#include "line_sensor.h"
#include "motor_driver.h"
#include "pid.h"

// ------------------------------------------------------------
// Donanım nesneleri
// ------------------------------------------------------------
LineSensorArray sensors;

MotorChannel leftMotor(LEFT_PWM_PIN, LEFT_IN1_PIN, LEFT_IN2_PIN, LEFT_MOTOR_INVERTED);
MotorChannel rightMotor(RIGHT_PWM_PIN, RIGHT_IN3_PIN, RIGHT_IN4_PIN, RIGHT_MOTOR_INVERTED);

PidController pid(KP, KI, KD, INTEGRAL_LIMIT, DERIVATIVE_FILTER);

// ------------------------------------------------------------
// Robot durumu
// ------------------------------------------------------------
enum class RobotState : uint8_t {
  WAIT,         // Kalibrasyon bekleniyor
  CALIBRATING,  // Sensörler ölçülüyor
  SETTLE,       // Kalkış öncesi kısa bekleme
  RUNNING       // Çizgi takibi
};

RobotState state = RobotState::WAIT;

uint32_t stateStartedMs = 0;
uint32_t lastControlUs = 0;
uint32_t lastDebugMs = 0;
uint32_t lastCalibrationPrintMs = 0;
uint32_t lineLostAtMs = 0;

bool onLine = false;
float lastValidError = 0.0f;
bool debugEnabled = true;

// ------------------------------------------------------------
// Yardımcı fonksiyonlar
// ------------------------------------------------------------

// Motorun ölü bölgede kalmasını engeller, PWM'i sınırlar.
int applyMotorDeadband(int speed) {
  speed = constrain(speed, -MAX_PWM, MAX_PWM);

  if (speed > 0 && speed < MIN_PWM) {
    return MIN_PWM;
  }
  if (speed < 0 && speed > -MIN_PWM) {
    return -MIN_PWM;
  }

  return speed;
}

int roundToPwm(float value) {
  return (value >= 0.0f) ? static_cast<int>(value + 0.5f)
                         : static_cast<int>(value - 0.5f);
}

void stopMotors() {
  leftMotor.stop();
  rightMotor.stop();
}

// Buton basıldığında bir kez true döndürür (25 ms debounce).
bool readButtonEvent() {
  static bool lastReading = HIGH;
  static bool stableReading = HIGH;
  static uint32_t lastChangeMs = 0;

  const bool reading = digitalRead(CALIBRATION_BUTTON_PIN);
  const uint32_t now = millis();

  if (reading != lastReading) {
    lastReading = reading;
    lastChangeMs = now;
  }

  if ((now - lastChangeMs) > 25 && reading != stableReading) {
    stableReading = reading;

    if (stableReading == LOW) {
      return true;
    }
  }

  return false;
}

void printBanner() {
  Serial.println();
  Serial.println(F("=============================================="));
  Serial.println(F(" BEYAZ CIZGI TAKIP ROBOTU - 4 MOTOR"));
  Serial.println(F("=============================================="));
  Serial.println(F("Kontrol: 200 Hz, PID + agirlikli sensor ortalamasi"));
  Serial.println(F("Buton yoksa kalibrasyon otomatik baslar."));
  Serial.println();

  if (AUTO_START_DELAY_MS > 0) {
    Serial.print(F("Kalibrasyon "));
    Serial.print(AUTO_START_DELAY_MS / 1000);
    Serial.println(F(" saniye icinde baslayacak."));
  }

  Serial.println(F("HAZIRLIK: Sensor dizisini cizginin uzerine yerlestirin."));
  Serial.println(F("Kalibrasyonda cizgi ve zemini gormesi icin saga-sola"));
  Serial.println(F("yavasca supurun. (6 saniye)"));
  Serial.println(F("Buton veya 'c' komutu ile hemen baslatabilirsiniz."));
  Serial.println();
}

void startCalibration() {
  stopMotors();
  digitalWrite(STATUS_LED_PIN, HIGH);

  sensors.startCalibration(millis());

  state = RobotState::CALIBRATING;
  stateStartedMs = millis();
  lastCalibrationPrintMs = stateStartedMs;

  Serial.println();
  Serial.println(F("=== KALIBRASYON BASLADI ==="));
  Serial.println(F("Sensorleri cizgi ve zemin uzerinde saga-sola supurun!"));
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());

    if (command == 'c' || command == 'C') {
      if (state != RobotState::CALIBRATING) {
        startCalibration();
      }
    } else if (command == 'd' || command == 'D') {
      debugEnabled = !debugEnabled;
      Serial.print(F("Debug: "));
      Serial.println(debugEnabled ? F("acik") : F("kapali"));
    }
  }
}

void waitForCalibration(uint32_t now) {
  stopMotors();

  // Yavaş yanıp sönme
  digitalWrite(STATUS_LED_PIN, ((now / 500) % 2) ? HIGH : LOW);

  if (AUTO_START_DELAY_MS > 0 && (now - stateStartedMs) >= AUTO_START_DELAY_MS) {
    startCalibration();
  }
}

void updateCalibration(uint32_t now) {
  stopMotors();

  if (sensors.updateCalibration(now)) {
    sensors.finishCalibration();
    sensors.printCalibration(Serial);

    if (sensors.validCount() == 0) {
      Serial.println();
      Serial.println(F("HATA: Hicbir sensor kalibre edilemedi!"));
      Serial.println(F("Kablolari, sensor mesafesini (3-8 mm) ve"));
      Serial.println(F("cizgi/zemin kontrastini kontrol edin."));
      Serial.println(F("5 saniye sonra yeniden denenecek..."));
      stopMotors();
      state = RobotState::WAIT;
      stateStartedMs = now;
      return;
    }

    if (sensors.validCount() < SENSOR_COUNT) {
      Serial.println(F("UYARI: Bazi sensorler gecersiz, robot eksik veriyle calisacak."));
    }

    stopMotors();
    digitalWrite(STATUS_LED_PIN, HIGH);
    Serial.println();
    Serial.print(F("Kalibrasyon tamam. Motorlar "));
    Serial.print(START_DELAY_MS / 1000);
    Serial.println(F(" saniye sonra calisacak."));

    state = RobotState::SETTLE;
    stateStartedMs = now;
    return;
  }

  // Hızlı yanıp sönme
  digitalWrite(STATUS_LED_PIN, ((now / 100) % 2) ? HIGH : LOW);

  if ((now - lastCalibrationPrintMs) >= 1000) {
    lastCalibrationPrintMs = now;
    Serial.print(F("Kalibrasyon: "));
    Serial.print((now - stateStartedMs) / 1000);
    Serial.print('/');
    Serial.print(CALIBRATION_MS / 1000);
    Serial.println(F(" s"));
  }
}

void updateSettle(uint32_t now) {
  stopMotors();
  digitalWrite(STATUS_LED_PIN, HIGH);

  if ((now - stateStartedMs) < START_DELAY_MS) {
    return;
  }

  pid.reset();
  onLine = false;
  lastValidError = 0.0f;
  lineLostAtMs = now;
  lastControlUs = micros();
  lastDebugMs = now;

  state = RobotState::RUNNING;

  Serial.println(F("ROBOT CALISIYOR. (Buton = yeniden kalibrasyon)"));
}

void updateRunning(uint32_t now) {
  const uint32_t nowUs = micros();
  const uint32_t elapsedUs = nowUs - lastControlUs;

  if (elapsedUs < CONTROL_PERIOD_US) {
    return;
  }

  lastControlUs = nowUs;

  float dt = static_cast<float>(elapsedUs) / 1000000.0f;
  if (dt > 0.1f) {
    dt = 0.1f;
  }

  bool detected = false;
  const float error = sensors.readError(detected);

  float correction = 0.0f;
  int leftOut = 0;
  int rightOut = 0;

  if (detected) {
    if (!onLine) {
      // Çizgiyi yeniden bulduk; eski I/D değerlerini sıfırla.
      pid.reset();
      onLine = true;
    }

    lineLostAtMs = now;
    lastValidError = error;

    correction = pid.update(error, dt);

    const int leftRaw = BASE_SPEED + roundToPwm(correction);
    const int rightRaw = BASE_SPEED - roundToPwm(correction);

    leftOut = applyMotorDeadband(leftRaw);
    rightOut = applyMotorDeadband(rightRaw);

    leftMotor.setSpeed(leftOut);
    rightMotor.setSpeed(rightOut);

    digitalWrite(STATUS_LED_PIN, HIGH);
  } else {
    // Çizgi görünmüyor: son görülen tarafa doğru yerinde dön.
    if (onLine) {
      onLine = false;
      lineLostAtMs = now;
      pid.reset();
    }

    if ((now - lineLostAtMs) < LINE_LOST_TIMEOUT_MS) {
      const int direction = (lastValidError >= 0.0f) ? 1 : -1;
      const int turnPwm = SEARCH_SPEED * direction;

      leftOut = applyMotorDeadband(turnPwm);
      rightOut = applyMotorDeadband(-turnPwm);

      leftMotor.setSpeed(leftOut);
      rightMotor.setSpeed(rightOut);

      digitalWrite(STATUS_LED_PIN, ((now / 100) % 2) ? HIGH : LOW);
    } else {
      stopMotors();
      digitalWrite(STATUS_LED_PIN, LOW);
    }

    correction = 0.0f;
  }

  if (debugEnabled && (now - lastDebugMs) >= DEBUG_PERIOD_MS) {
    lastDebugMs = now;

    Serial.print(F("e="));
    Serial.print(error, 2);
    Serial.print(F(" corr="));
    Serial.print(correction, 1);
    Serial.print(F(" L="));
    Serial.print(leftOut);
    Serial.print(F(" R="));
    Serial.print(rightOut);
    Serial.print(F(" line="));
    Serial.println(detected ? 1 : 0);
  }
}

// ------------------------------------------------------------
// Arduino giriş noktaları
// ------------------------------------------------------------
void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  pinMode(CALIBRATION_BUTTON_PIN, INPUT_PULLUP);

  leftMotor.begin();
  rightMotor.begin();
  stopMotors();

  sensors.begin();

  Serial.begin(115200);

  state = RobotState::WAIT;
  stateStartedMs = millis();

  printBanner();
}

void loop() {
  handleSerialCommands();

  // Buton: kalibrasyonu hemen başlat / yeniden başlat.
  // Kalibrasyon sırasındaki basışlar yok sayılır.
  if (readButtonEvent() && state != RobotState::CALIBRATING) {
    startCalibration();
    return;
  }

  // Komutlar kalibrasyonu başlatmış olabilir; zamanı yeniden oku.
  const uint32_t now = millis();

  switch (state) {
    case RobotState::WAIT:
      waitForCalibration(now);
      break;

    case RobotState::CALIBRATING:
      updateCalibration(now);
      break;

    case RobotState::SETTLE:
      updateSettle(now);
      break;

    case RobotState::RUNNING:
      updateRunning(now);
      break;
  }
}
