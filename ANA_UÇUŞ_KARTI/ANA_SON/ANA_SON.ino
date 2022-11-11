#include <SPI.h>
#include <SD.h>
#include <LoRaLib.h>
#include <EEPROM.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <SimpleKalmanFilter.h> 
#include <Adafruit_BMP280.h>
#include <SparkFunMPU9250-DMP.h>
#define Buzzer 42 // buzzer baglantisi

#define test_durumu 1

// Kullanilan modullerin kutuphanelerini kullanmak icin olusturulan objeler:
Adafruit_BMP280 bmp; // use I2C interface
MPU9250_DMP imu;

Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
sensors_event_t temp_event, pressure_event;
SX1278 lora = new LoRa; 
File dosya; 
TinyGPSPlus gps; 
SimpleKalmanFilter kalmanFiltresi(1, 1, 0.1);

// Kullanilan pinler:
const byte csBaglantisi = 49; // SD Kart Chip Select Pin
const byte RFMBaglantisi = A2; // RFM98W NSS Pin
const byte DragP = 62, MainP = 30; // Suruklenme ve Ana parasut aktivasyon pinleri

// GPS modulunun fonksiyonlari icin tanimlar:
static const uint32_t GPSBaud = 9600; 
const long SERIAL_REFRESH_TIME = 10; 
long mpuZaman = 0, mpuEskiZaman = 0, dt = 0;
long refresh_time, lastTransmission = 0;

// Kodda kullanilan degiskenler:
float ilk_basinc = 0, basinc = 0, sicaklik = 0;
float pitch = 0, roll = 0, yaw = 0;
float ivmeX = 0, ivmeY = 0, ivmeZ = 0;
float maksIrtifa = -2000, anlikIrtifa = 0,filtreliIrtifa=0;
unsigned int DragPrDiff = 15;
unsigned int MainPrDiff = 600; 
float veritipi;
bool roketDususte = false;

void setup() 
{
  Serial.begin(9600); 
  Serial3.begin(GPSBaud); // GPS modulunun seri haberlesmesinin baslamasi.

  // Cikti pinlerin tanimlanmasi:
  pinMode(DragP, OUTPUT);
  pinMode(MainP, OUTPUT);
  pinMode(Buzzer, OUTPUT); 
  pinMode(RFMBaglantisi, OUTPUT);
  pinMode(csBaglantisi, OUTPUT);
  pinMode(A10, INPUT);
  pinMode(A6, INPUT);
  digitalWrite(DragP, LOW);
  digitalWrite(MainP, LOW);

  unsigned status;
  status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  delay(250);
  bmp_temp->printSensorDetails();
  sicaklik = bmp.readTemperature();
  ilk_basinc = bmp.readPressure();

if (imu.begin() != INV_SUCCESS)
  {
    Serial.println("MPU9250 Baslatma Hatasi!");
  }

  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  imu.setGyroFSR(2000); // Set gyro to 2000 dps
  imu.setAccelFSR(2); // Set accel to +/-2g
  imu.setLPF(5); // Set LPF corner frequency to 5Hz
  imu.setSampleRate(10); // Set sample rate to 10Hz
  imu.setCompassSampleRate(10); // Set mag rate to 10Hz
  delay(250);

  // Slave Select olarak sd kart modulunu sec (SPI haberlesmesi):
  digitalWrite(csBaglantisi, LOW); digitalWrite(RFMBaglantisi, HIGH); 

  Serial.println("Sd kart baslatiliyor...");
  if (!SD.begin(csBaglantisi)) 
  {
    digitalWrite(Buzzer,HIGH);
    delay(2000);
    digitalWrite(Buzzer,LOW);
    delay(1000);
    digitalWrite(Buzzer,HIGH);
    delay(2000);
    digitalWrite(Buzzer,LOW);
    delay(2000);
    Serial.println("Sd kart baglantisi basarisiz!");
  }
  else {
    // Mikro SD Kart baglantisi basarisiz ise:
    digitalWrite(Buzzer,HIGH);
    delay(500);
    digitalWrite(Buzzer,LOW);
    delay(2000);
  }

  // Slave Select olarak rfm98w modulunu sec (SPI haberlesmesi):
  digitalWrite(csBaglantisi, HIGH); digitalWrite(RFMBaglantisi, LOW);

  int state = lora.begin(); // LoRa'yi baslat ve begin methodunun dondurdugunu kaydet.
  lora.setFrequency(434.69); 

  if (state == ERR_NONE) // Eger durum "HATASIZ" ise: 
  {
    Serial.println(F("LoRa Baglantisi Basarili!")); 
    digitalWrite(Buzzer,HIGH);
    delay(500);
    digitalWrite(Buzzer,LOW);
    delay(2000);
  } 

  else // Degilse: 
  {
    Serial.print(F("LoRa Baglantisi Basarisiz! Hata Kodu: ")); 
    Serial.println(state); // hata kodu
    digitalWrite(Buzzer,HIGH);
    delay(2000);
    digitalWrite(Buzzer,LOW);
    delay(1000);
    digitalWrite(Buzzer,HIGH);
    delay(2000);
    digitalWrite(Buzzer,LOW);
    delay(2000);
    while (true); // sonsuza kadar durdur
  }
}

void loop() 
{
  sicaklik = bmp.readTemperature();
  float irtifa = bmp.readAltitude(ilk_basinc / 100);

  float filtreliIrtifa = kalmanFiltresi.updateEstimate(irtifa); // okunan irtifayi Kalman Filtresinden gecir

  if (filtreliIrtifa > maksIrtifa) // Eger okunan irtifa maks irtifadan buyuk ise:
  {
    maksIrtifa = filtreliIrtifa; // Okunan irtifayi maks irtifa yap.
  }

  // Eger okunan irtifa ile maks irtifa arasi fark 15'den fazla ise:
  if (maksIrtifa - filtreliIrtifa > DragPrDiff && !roketDususte) 
  {
      Serial.println("Suruklenme Parasutleri Acildi!"); roketDususte = true; 
      digitalWrite(DragP,HIGH); // Suruklenme Parasutunu aktif et.
  }

  // Eger suruklenme parasutu acilmis ve okunan irtifa 600'un altinda ise:
  if (roketDususte == true && filtreliIrtifa < MainPrDiff)
  {
      Serial.println("Ana Parasut Acildi!");
      digitalWrite(MainP,HIGH); // Ana parasutu aktif et.
  }  
    
  // Slave Select olarak sd kart modulunu sec (SPI haberlesmesi):
  digitalWrite(csBaglantisi, LOW); digitalWrite(RFMBaglantisi, HIGH);
  
  dosya = SD.open("anaVeri.txt", FILE_WRITE); // Sd kart dosyasini ac.

  if (dosya) // Eger dosya duzgun acildiysa:
  {
    // Verileri dosyaya yazdir:
    dosya.print("Irtifa: "); dosya.print(filtreliIrtifa);
    dosya.print("|| Uydu Sayisi: "); dosya.print(gps.satellites.value(), 7); 
    dosya.print(" Irtifa(GPS): "); dosya.print(gps.altitude.meters(), 7);
    dosya.print(" Enlem: "); dosya.print(gps.location.lat(), 7);
    dosya.print(" Boylam: "); dosya.print(gps.location.lng(), 7);
    dosya.println(""); 

    dosya.close(); // dosyayi kapat.
  } 

  else 
  {
    
    dosya.close(); // dosyayi kapat.
  }

  // Slave Select olarak rfm98w modulunu sec (SPI haberlesmesi):
  digitalWrite(csBaglantisi, HIGH); digitalWrite(RFMBaglantisi, LOW);

  if ( imu.dataReady() )
  {
      mpuZaman = millis();
      dt = (float) (abs(mpuZaman - mpuEskiZaman) / 1000.0);
      imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
      ivmeX = imu.calcAccel(imu.ax) * 9.815;
    ivmeY = imu.calcAccel(imu.ay) * 9.815;
    ivmeZ = imu.calcAccel(imu.az) * 9.815;
    roll = roll + imu.calcGyro(imu.gx) * dt;
    pitch += imu.calcGyro(imu.gy) * dt;
    yaw += imu.calcGyro(imu.gz) * dt;
    mpuEskiZaman = millis();
  }


/*  mpu.update_accel_gyro();
  ivmeX = mpu.getAccX() * 9.81; ivmeY = mpu.getAccY()  * 9.81; ivmeZ = mpu.getAccZ() * 9.81;
  pitch = mpu.getPitch(); yaw = mpu.getYaw(); roll = mpu.getRoll();*/

   
  // Veri olarak gonderilecek string'in olusturulmasi:
  
  
  String bmpVeri= String(filtreliIrtifa), gpsEn = String(gps.location.lat(),6),gpsBoy = String(gps.location.lng(),6), spd = String(gps.speed.kmph(),6);
  String toplamVeri = bmpVeri+" //"+maksIrtifa;
  Serial.println(toplamVeri);

  if (lora.transmit(toplamVeri) != ERR_NONE)
    Serial.println("Veri Gonderilemedi!");

  if (test_durumu == 1) // Basinc Testi 
  {
    if (!roketDususte && abs(irtifa - maksIrtifa) > 15)
    {
      digitalWrite(DragP, HIGH);
      Serial.println("Roket Dususte! Suruklenme Parsutu Acildi!");
      roketDususte = true;
      
    }

    else if (roketDususte && irtifa < 600)
    {
      digitalWrite(MainP, HIGH);
      Serial.println("Ana Parsut Acildi!");
      
    }
  }

  else if (test_durumu == 2) // Yonelim Testi
  {
    if (abs(pitch) > 90 || abs(yaw) > 90)
    {
     digitalWrite(Buzzer, HIGH);
      delay(250);
      digitalWrite(Buzzer, LOW);
      delay(25);
    }
    
   
  }

  delay(150);  
  smartDelay(50); // 50ms'lik delay
}


//GPS ve Delay fonksiyonlari

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial3.available())
      gps.encode(Serial3.read());

  } while (millis() - start < ms);
}
