#pragma once

#include <Arduino.h>
#include <math.h>

#include "config.h"

// ============================================================
//  Sensör dizisi
//  - Kalibrasyon sırasında her sensörün min/max değerini kaydeder.
//  - Çizgi konumunu ağırlıklı ortalama ile -1..+1 aralığında döndürür.
//    (0 = tam orta, +1 = en sağdaki sensör)
// ============================================================
class LineSensorArray {
 public:
  void begin();

  // Kalibrasyonu başlat / sürdür / bitir.
  void startCalibration(uint32_t nowMs);
  bool updateCalibration(uint32_t nowMs);
  void finishCalibration();

  // Sensörleri okur. lineDetected, çizgi görülüyorsa true olur.
  // Dönen değer son geçerli hatadır (-1..+1).
  float readError(bool &lineDetected);

  // Kalibrasyonu geçerli (kontrastı yeterli) sensör sayısı.
  uint8_t validCount() const;

  void printCalibration(Stream &out) const;

 private:
  uint16_t min_[SENSOR_COUNT];
  uint16_t max_[SENSOR_COUNT];
  uint32_t lowVotes_[SENSOR_COUNT];
  uint32_t highVotes_[SENSOR_COUNT];
  bool valid_[SENSOR_COUNT];

  bool calibrated_ = false;
  bool lineReadsLow_ = LINE_READS_LOW;
  uint32_t calibrationStartMs_ = 0;
  float lastError_ = 0.0f;
  float maxWeight_ = 1.0f;

  float activation(uint8_t index, int rawValue) const;
  void resetCalibrationData();
};

inline void LineSensorArray::begin() {
  maxWeight_ = 1.0f;

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    pinMode(SENSOR_PINS[i], INPUT);
    valid_[i] = false;

    const float weight = fabsf(SENSOR_WEIGHTS[i]);
    if (weight > maxWeight_) {
      maxWeight_ = weight;
    }
  }

  resetCalibrationData();
}

inline void LineSensorArray::resetCalibrationData() {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    min_[i] = 65535;
    max_[i] = 0;
    lowVotes_[i] = 0;
    highVotes_[i] = 0;
  }
}

inline void LineSensorArray::startCalibration(uint32_t nowMs) {
  resetCalibrationData();

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    valid_[i] = false;
  }

  calibrated_ = false;
  lineReadsLow_ = LINE_READS_LOW;
  calibrationStartMs_ = nowMs;
  lastError_ = 0.0f;
}

inline bool LineSensorArray::updateCalibration(uint32_t nowMs) {
  int raw[SENSOR_COUNT];
  int sorted[SENSOR_COUNT];

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    raw[i] = analogRead(SENSOR_PINS[i]);
    sorted[i] = raw[i];

    if (raw[i] < static_cast<int>(min_[i])) {
      min_[i] = static_cast<uint16_t>(raw[i]);
    }
    if (raw[i] > static_cast<int>(max_[i])) {
      max_[i] = static_cast<uint16_t>(raw[i]);
    }
  }

  // Medyan: sensörlerin çoğunluğu zemini gördüğü için medyan zemin seviyesidir.
  // Çizginin altındaki 1-2 sensör bu değerden sapar; sapma yönü polariteyi verir.
  for (uint8_t i = 1; i < SENSOR_COUNT; ++i) {
    const int key = sorted[i];
    int j = static_cast<int>(i) - 1;

    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      --j;
    }

    sorted[j + 1] = key;
  }

  const int median = sorted[SENSOR_COUNT / 2];

  // Aynı anda 3+ sensör medyandan sapıyorsa çizgi geniş/sensör dizisi darsa
  // polarite güvenilir belirlenemez; bu örnekleri oylamaya katma.
  uint8_t deviationCount = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (raw[i] < (median - POLARITY_VOTE_MARGIN) ||
        raw[i] > (median + POLARITY_VOTE_MARGIN)) {
      ++deviationCount;
    }
  }

  if (deviationCount >= 1 && deviationCount <= 2) {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
      if (raw[i] < (median - POLARITY_VOTE_MARGIN)) {
        ++lowVotes_[i];
      } else if (raw[i] > (median + POLARITY_VOTE_MARGIN)) {
        ++highVotes_[i];
      }
    }
  }

  return (nowMs - calibrationStartMs_) >= CALIBRATION_MS;
}

inline void LineSensorArray::finishCalibration() {
  calibrated_ = true;

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    valid_[i] = (static_cast<int>(max_[i]) - static_cast<int>(min_[i])) >= MIN_SENSOR_SPAN;
  }

  // Polarite: yalnızca kalibrasyonu geçerli sensörlerin oylarını say.
  // Çizgi, sensör dizisinden ince olduğu için medyandan sapan azınlık çizgidir.
  // Aynı anda 3+ sensör sapan örnekler updateCalibration'da oylanmaz.
  if (AUTO_DETECT_LINE_POLARITY) {
    uint32_t lowTotal = 0;
    uint32_t highTotal = 0;

    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
      if (!valid_[i]) {
        continue;
      }

      lowTotal += lowVotes_[i];
      highTotal += highVotes_[i];
    }

    if (lowTotal > highTotal) {
      lineReadsLow_ = true;
    } else if (highTotal > lowTotal) {
      lineReadsLow_ = false;
    }
  }
}

inline uint8_t LineSensorArray::validCount() const {
  uint8_t count = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (valid_[i]) {
      ++count;
    }
  }

  return count;
}

// Ham değeri 0..1 "çizgi yoğunluğuna" çevirir.
inline float LineSensorArray::activation(uint8_t index, int rawValue) const {
  if (!valid_[index]) {
    return 0.0f;
  }

  const float low = static_cast<float>(min_[index]);
  const float high = static_cast<float>(max_[index]);
  const float middle = (low + high) * 0.5f;

  float value;

  if (lineReadsLow_) {
    value = (middle - static_cast<float>(rawValue)) / (middle - low);
  } else {
    value = (static_cast<float>(rawValue) - middle) / (high - middle);
  }

  return constrain(value, 0.0f, 1.0f);
}

inline float LineSensorArray::readError(bool &lineDetected) {
  float sum = 0.0f;
  float weightedSum = 0.0f;

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    float value = activation(i, analogRead(SENSOR_PINS[i]));

    // Küçük gürültüleri yok say.
    if (value < SENSOR_FLOOR) {
      value = 0.0f;
    }

    sum += value;
    weightedSum += value * SENSOR_WEIGHTS[i];
  }

  lineDetected = calibrated_ && (sum >= LINE_PRESENT_THRESHOLD);

  if (lineDetected && sum > 0.0f) {
    // Ağırlıklı ortalama -maxWeight..+maxWeight aralığındadır.
    const float position = weightedSum / sum;
    lastError_ = constrain(position / maxWeight_, -1.0f, 1.0f);
  }

  return lastError_;
}

inline void LineSensorArray::printCalibration(Stream &out) const {
  out.println(F("--- Kalibrasyon sonucu ---"));

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    const int span = static_cast<int>(max_[i]) - static_cast<int>(min_[i]);

    out.print(F("S"));
    out.print(i);
    out.print(F(": min="));
    out.print(min_[i]);
    out.print(F(" max="));
    out.print(max_[i]);
    out.print(F(" span="));
    out.print(span);
    out.print(F("  "));
    out.println(valid_[i] ? F("OK") : F("HATA (baglanti/kontrast)"));
  }

  out.print(F("Gecerli sensor: "));
  out.print(validCount());
  out.print('/');
  out.println(SENSOR_COUNT);

  out.print(F("Cizgi polaritesi: "));
  out.println(lineReadsLow_ ? F("analog DUSUK") : F("analog YUKSEK"));
}
