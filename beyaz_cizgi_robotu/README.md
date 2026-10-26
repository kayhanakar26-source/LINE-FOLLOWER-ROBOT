# Beyaz Çizgi Takip Robotu (4 Motor)

Arduino (C++) ile yazılmış, **4 motorlu** ve **5 sensörlü** beyaz çizgi takip robotu.

- 4 DC motor, diferansiyel (tank) sürüş: her taraftaki 2 motor paralel bağlı
- 5 × TCRT5000 analog sensör
- Ağırlıklı ortalama ile çizgi konumu
- 200 Hz PID kontrolü
- Açılışta otomatik kalibrasyon + otomatik sensör polaritesi algılama
- Çizgi kaybolunca son görülen tarafa doğru arama, 1,5 sn sonra güvenli duruş
- Motor ölü bölgesi (deadband) telafisi
- Seri porttan canlı hata ve PWM takibi
- Buton veya `c` komutu ile yeniden kalibrasyon

> Varsayılan hedef: **Arduino Uno / Nano**. Pinler `config.h` içinden değiştirilebilir.

## Hızlı başlangıç

1. README'deki bağlantı tablosuna göre kablolayın (ortak GND ve L298N jumper'larına dikkat).
2. `beyaz_cizgi_robotu.ino` dosyasını Arduino IDE ile yükleyin.
3. Robotu zemine koyup güç verin; 5 saniye sonra başlayan 6 saniyelik kalibrasyon boyunca sensör dizisini çizgi ve zemin üzerinde gezdirin.
4. Seri portta tüm sensörler `OK` ise robot 2 saniye sonra çalışmaya başlar.

---

## 1. Klasör yapısı

```
beyaz_cizgi_robotu/              ← Arduino sketch klasörü
├── platformio.ini
├── README.md
├── beyaz_cizgi_robotu.ino       ← ana program
├── config.h                     ← pinler ve ayarlar
├── line_sensor.h                ← sensör kalibrasyonu / konum hesabı
├── motor_driver.h               ← motor kanalı
└── pid.h                        ← PID denetleyici
```

---

## 2. Donanım listesi

| Parça | Adet | Not |
|---|---:|---|
| Arduino Uno veya Nano | 1 | 5 V mantık |
| L298N motor sürücü | 1 | 2 kanal, kanal başına ~2 A |
| DC redüktörlü motor + teker | 4 | TT motor gibi 3–6 V motorlar |
| TCRT5000 sensör modülü | 5 | **AO (analog) çıkışı olan** model |
| Pil | 1 | 2S Li-ion (7,4 V) veya motor voltajına uygun pil |
| Anahtar, kablo, şasi | — | — |
| Buton | 0–1 | İsteğe bağlı, D2 → GND |
| 5 V regülatör / buck | 0–1 | Arduino'yu pilden beslemek için önerilir |

> L298N yaklaşık 1,5–2 V gerilim düşürür. 6 V TT motorlar için 2S Li-ion (7,4 V) iyi bir seçimdir; 9 V pil motorları gereksiz yere ısıtabilir.

> L298N tek kanalda 2 A sürekli akım verir. 4 motoru 2'şerli paralel bağlarken motorların toplam akımının bu sınırı aşmamasına dikkat edin. Yüksek akımlı motorlarda iki L298N veya daha güçlü bir sürücü kullanın.

---

## 3. Bağlantı şeması

### Sensörler (robotun önünden bakınca soldan sağa)

| Sensör | Arduino |
|---|---|
| S0 (en sol) | A0 |
| S1 | A1 |
| S2 (orta) | A2 |
| S3 | A3 |
| S4 (en sağ) | A4 |
| VCC | 5 V |
| GND | GND |
| OUT / AO | İlgili analog pin |

> Modüldeki **AO** (analog out) çıkışını kullanın. Sadece dijital `DO` çıkışı olan modüller bu koddaki kalibrasyonla çalışmaz.

### Motor sürücü

| L298N | Arduino | Bağlantı |
|---|---|---|
| ENA | D5 | Sol kanal PWM |
| IN1 | D7 | Sol yön 1 |
| IN2 | D8 | Sol yön 2 |
| ENB | D6 | Sağ kanal PWM |
| IN3 | D9 | Sağ yön 1 |
| IN4 | D10 | Sağ yön 2 |
| GND | GND | **Ortak GND şart** |
| +12V | Pil + | Motor beslemesi |
| GND | Pil − | Motor beslemesi |
| 5V | Arduino 5V* | Mantık beslemesi |
| OUT1 / OUT2 | Sol 2 motor | Paralel |
| OUT3 / OUT4 | Sağ 2 motor | Paralel |

\* L298N üzerindeki 5V-EN jumper'ı takılıysa ve motor beslemesi 7 V'un üzerindeyse, L298N'in 5V çıkışı Arduino'yu besleyebilir. Daha güvenli ve stabil çözüm ayrı bir 5 V buck regülatördür. **Arduino'nun 5V pinine hem USB hem L298N 5V bağlamayın.**

### Diğer

| Parça | Bağlantı |
|---|---|
| Kalibrasyon butonu | D2 ↔ GND (dahili pull-up kullanılır) |
| Durum LED'i | D13 (Uno/Nano üzerindeki LED) |

### Motorların yönü

- Sol iki motor **kendi aralarında aynı yöne** bakacak şekilde bağlanır, L298N'in OUT1/OUT2 çıkışına paralel girilir.
- Sağ iki motor da OUT3/OUT4'e paralel bağlanır.
- Bir taraf ileri giderken diğer taraf geri gidiyorsa, o motorun iki kablosunu yer değiştirin. Alternatif olarak `config.h` içindeki `LEFT_MOTOR_INVERTED` / `RIGHT_MOTOR_INVERTED` değerini `true` yapın.

### Mekanik yerleşim

- Sensör dizisi, ön tekerlek/aks merkezinin **5–8 cm önüne** monte edilmelidir.
- Sensörler zeminden **3–8 mm** yüksekte olmalıdır.
- Sensörler arası mesafe **12–15 mm**, çizgi genişliği **15–18 mm** olmalıdır.
- Otomatik polarite algılaması için çizgi, aynı anda **en fazla 2 sensörün** altına girecek kadar ince olmalıdır (5 sensörlü dizide 12–15 mm aralık + 15–18 mm çizgi bunu sağlar). Daha geniş çizgi / dar aralıkta 3 sensör birden çizgiyi görebilir; bu durumda `AUTO_DETECT_LINE_POLARITY` kapatılıp polarite elle seçilmelidir.
- Beyaz çizgi ile zemin arasında net bir kontrast olmalıdır (koyu zemin + beyaz çizgi önerilir).

---

## 4. Yazılımı yükleme

### Arduino IDE

1. `beyaz_cizgi_robotu.ino` dosyasını açın.
2. Kart olarak **Arduino Uno** (veya Nano), doğru portu seçin.
3. Yükleyin.
4. Seri portu **115200 baud** ile açın.

### PlatformIO

```bash
cd beyaz_cizgi_robotu
pio run -t upload
pio device monitor
```

---

## 5. Kalibrasyon (önemli)

Kalibrasyon her açılışta gerekir. Amaç, her sensörün "çizgi" ve "zemin" değerlerini öğrenmesidir.

1. Robotu zemine koyun; sensörlerin altında çizgi olmasın.
2. Güç verin. **5 saniye** sonra kalibrasyon otomatik başlar (veya butona / seri porttan `c` komutuna basın).
3. 6 saniye boyunca sensör dizisini çizgi ve zemin üzerinde gezdirin:
   - her sensörün hem çizgiyi hem de zemini görmesini sağlayın,
   - çizginin üzerinden yavaşça geçirin,
   - sonunda robotu zeminde bırakın.
4. Seri portta şuna benzer bir çıktı görürsünüz:

```
--- Kalibrasyon sonucu ---
S0: min=180 max=910 span=730  OK
S1: min=190 max=905 span=715  OK
S2: min=175 max=915 span=740  OK
S3: min=185 max=900 span=715  OK
S4: min=195 max=920 span=725  OK
Gecerli sensor: 5/5
Cizgi polaritesi: analog YUKSEK
```

5. Tüm sensörler `OK` olmalı. `HATA` varsa kablo, 5 V besleme, sensör yüksekliği ve kontrastı kontrol edin.
6. Robot 2 saniye sonra çalışmaya başlar.

Yeniden kalibrasyon: **butona basın** veya seri porttan `c` gönderin.

### Polarite

TCRT5000 modülleri beyaz yüzeyde bazen düşük, bazen yüksek analog değer verebilir. Çoğu modülde daha fazla IR yansıması **daha yüksek AO değeri** demektir; yani beyaz çizgi `analog YUKSEK` görünür. Bu yüzden varsayılan `LINE_READS_LOW` değeri `false`'tur.

Kod, polariteyi kalibrasyon sırasında otomatik algılar. Çalışma prensibi: sensör dizisindeki çoğunluk zemini görür; medyan değerden aşağı veya yukarı sapan azınlık sensörler çizgiyi gösterir. Oyların çoğu hangi taraftaysa çizgi o taraftadır.

Otomatik algılama yanlış sonuç verirse:

```cpp
constexpr bool AUTO_DETECT_LINE_POLARITY = false;
constexpr bool LINE_READS_LOW = true;  // veya false
```

Seri çıktıdaki polarite satırına bakarak doğru değeri seçebilirsiniz. Çizgi 3 veya daha fazla sensörün altına giriyorsa otomatik algılama güvenilir değildir; polariteyi elle ayarlayın.

---

## 6. Nasıl çalışıyor?

### Çizgi konumu

Her sensörün ham değeri, kalibrasyonda öğrenilen min/max aralığına göre 0–1 arasına normalize edilir. Çizginin altındaki sensör 1'e yakın, zemin üzerindeki sensör 0'a yakın değer verir.

Sensör ağırlıkları:

```
S0    S1    S2    S3    S4
-2    -1     0    +1    +2
```

Ağırlıklı ortalama:

```
hata = Σ(ağırlık × sensör) / Σ(sensör)
```

Bu değer -1 (çizgi en solda) ile +1 (çizgi en sağda) arasındadır ve PID'e verilir.

### Motor karışımı

```
sol PWM  = BASE_SPEED + PID çıkışı
sağ PWM  = BASE_SPEED − PID çıkışı
```

Böylece hata pozitifken (çizgi sağda) sol tekerlek hızlanır, robot sağa döner.

### PID

- `Kp`: hataya anlık tepki
- `Ki`: kalıcı sapmayı giderir (varsayılan 0)
- `Kd`: salınımı söndürür / virajda öngörü sağlar
- Kontrol döngüsü: 5 ms (200 Hz)
- I terimi sınırlı, D terimi alçak geçiren filtreli

### Çizgi kaybı

Çizgi görünmezse robot son görülen tarafa doğru yerinde döner. 1,5 saniye içinde bulamazsa motorları durdurur (güvenlik).

---

## 7. Ayar (tuning)

Tüm ayarlar `config.h` içinde.

| Ayar | Ne yapar? |
|---|---|
| `BASE_SPEED` | Düz giderken hız (PWM). İlk denemede 100–120 ile başlayın. |
| `KP` | Ana takip gücü. Varsayılan 100. |
| `KD` | Salınım sönümleme. Varsayılan 12. |
| `KI` | Kalıcı sapma. Genelde 0 bırakın; gerekirse 5–20 deneyin. |
| `MIN_PWM` | Motorların dönmeye başladığı en düşük PWM. |
| `MAX_PWM` | Güvenlik üst sınırı (235). |
| `SEARCH_SPEED` | Çizgi kaybolunca dönme hızı. |
| `LINE_LOST_TIMEOUT_MS` | Arama süresi. |

### Adım adım PID ayarı

1. `KI = 0`, `KD = 0` yapın.
2. `BASE_SPEED` düşükken (ör. 100) `KP`'yi artırın; robot çizgiyi takip edene kadar.
3. Robot sağa-sola zikzak yapıyorsa `KP`'yi %10–20 azaltın.
4. Salınım devam ederse `KD`'yi 2–5'er artırın.
5. Virajlarda dışa taşıyorsa `KP`'yi biraz artırın veya `BASE_SPEED`'i düşürün.
6. Düz giderken sürekli bir tarafa kayıyorsa önce sensör dizisinin mekanik hizasını, sonra motor hız farkını kontrol edin. Gerekirse `KI`'yi küçük değerlerle deneyin.

> Aynı anda tek ayarı değiştirin. Seri porttan `e=`, `corr=`, `L=`, `R=` değerlerini izleyebilirsiniz.

---

## 8. Seri port komutları ve çıktı

| Komut | İşlev |
|---|---|
| `c` / `C` | Yeniden kalibrasyon başlat |
| `d` / `D` | Hata ayıklama çıktısını aç/kapat |

Çalışma sırasındaki örnek çıktı:

```
e=0.12 corr=14.3 L=134 R=106 line=1
e=-0.45 corr=-48.0 L=72 R=168 line=1
```

- `e`: hata (-1..+1)
- `corr`: PID çıkışı
- `L`, `R`: uygulanan PWM değerleri
- `line`: 1 = çizgi görülüyor, 0 = görülmüyor

---

## 9. Sorun giderme

| Belirti | Çözüm |
|---|---|
| Robot hiç hareket etmiyor | Seri çıktıda `line=0` mı? Robotu çizgiye koyun. `L298N ENA/ENB jumper'ları` takılı mı? Ortak GND var mı? Pil dolu mu? |
| Kalibrasyonda tüm sensörler `HATA` | Sensörlerin AO çıkışı Arduino'ya bağlı mı? 5 V ve GND doğru mu? Sensör–zemin mesafesi 3–8 mm mi? Çizgi/zemin kontrastı var mı? |
| Tek sensör `HATA` | O sensörün kablosu veya bağlantısı gevşek olabilir. |
| Robot çizgiden hemen çıkıyor | Polarite yanlış olabilir. Seri çıktıdaki `Cizgi polaritesi` satırına bakın; gerekirse otomatik algılamayı kapatıp `LINE_READS_LOW` değerini ters çevirin. |
| Robot ters yöne dönüyor | Bir motorun kablolarını ters çevirin veya `LEFT/RIGHT_MOTOR_INVERTED` ayarını değiştirin. |
| Sağa-sola zikzak | `KP` azaltın, `KD` artırın, `BASE_SPEED` düşürün. |
| Virajı dönemiyor | `KP` artırın, `BASE_SPEED` düşürün. |
| Motorlar düşük hızda dönmüyor | `MIN_PWM` değerini 5–10'ar artırın. |
| L298N ısınıyor | Motor akımı yüksek. Daha güçlü sürücü kullanın, soğutucu ekleyin, motorları zorlamayın. |
| Seri portta anlamsız karakterler | Baud hızını **115200** yapın. |

---

## 10. Güvenlik

- Motorları **Arduino'nun 5 V çıkışından beslemeyin**; ayrı pil kullanın.
- L298N GND, pil eksi ve Arduino GND **mutlaka ortak** olmalı.
- İlk testte robotu havaya kaldırın / tekerlekleri boşa çevirin.
- USB bağlıyken L298N'in 5 V çıkışını Arduino'nun 5 V pinine vermeyin.
- Uzun süreli stall (motor kilitlenmesi) sürücüyü ve motorları ısıtır; `LINE_LOST_TIMEOUT_MS` güvenlik için vardır.
- Pil kablolarına sigorta eklemeniz önerilir.

---

## 11. Farklı sensör sayısı

`config.h` içinde:

```cpp
constexpr uint8_t SENSOR_COUNT = 5;
constexpr uint8_t SENSOR_PINS[SENSOR_COUNT] = {A0, A1, A2, A3, A4};
constexpr float SENSOR_WEIGHTS[SENSOR_COUNT] = {-2, -1, 0, 1, 2};
```

- 3 sensör: `{-1, 0, 1}`, pinler `{A0, A1, A2}`
- 7 sensör: `{-3, -2, -1, 0, 1, 2, 3}`, uygun analog pinler

Ağırlıklar simetrik olduğu sürece hata otomatik olarak -1..+1 aralığına normalize edilir. Pin ve ağırlık dizilerinin uzunluğu `SENSOR_COUNT` ile aynı olmalıdır.

> 3 sensörde otomatik polarite daha az güvenilirdir. Çizgi aynı anda en fazla 1 sensörün altına girmiyorsa `AUTO_DETECT_LINE_POLARITY` kapatıp polariteyi elle ayarlayın.

---

## 12. Kısa kontrol listesi

- [ ] L298N ENA/ENB jumper'ları çıkarıldı
- [ ] 4 motor doğru taraflara bağlandı
- [ ] Ortak GND yapıldı
- [ ] TCRT5000 AO çıkışları A0–A4'e bağlı
- [ ] Sensörler 3–8 mm yükseklikte
- [ ] Beyaz çizgi ile zemin arasında kontrast var
- [ ] Kalibrasyonda 5/5 sensör `OK`
- [ ] İlk testte tekerlekler boşta denendi
