#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <SD.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  
#define csBaglantisi A2
bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;     
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 

Quaternion q;           
VectorInt16 aa;        
VectorInt16 aaReal;     
VectorInt16 aaWorld;   
VectorFloat gravity;    
float ypr[3];          


float roll, pitch, yaw, pitch_i;
float AccX, AccY, AccZ;
const unsigned int egilmeAcisi = 85;

unsigned long surZaman = 0;

bool anaBGuc = false, surPar = false, anaPar = false, motorlarAcik = false;
bool suruklenmeIlkGiris = true, suruklenmeIlkGiris_guc = true, suruklenmeIlkGiris_par = true;
bool anaIlkGiris = true, motorIlkGiris = true;
bool sdKartCalisiyor = true;

File yedekSD;
const byte  anaMOSFET = 15, suruklenmeMOSFET = 9;
const byte buzzerPin = 8;
const byte b_guc = 5, b_suruklenme = 6, b_ana = 7; // 34 -> 5, 36 -> 6, 38 -> 7   
// 46 ve 9 -> suruklenme, 8 ve 44 -> ana parasut

volatile bool mpuInterrupt = false;    
void dmpDataReady() 
{
    mpuInterrupt = true;
}

void setup() 
 {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); 
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(9600);
    mpu.initialize();

    pinMode(INTERRUPT_PIN, INPUT);
    pinMode(csBaglantisi, OUTPUT);
    pinMode(buzzerPin, OUTPUT);
    pinMode(suruklenmeMOSFET, OUTPUT);
    pinMode(anaMOSFET, OUTPUT);
    mpuBaslat();
    
    pinMode(b_guc, INPUT); // Ana bilgisayar guc durumu
    pinMode(b_suruklenme, INPUT); // Ana bilgisayar suruklenme p. durumu
    pinMode(b_ana, INPUT); // Ana bilgisayar ana p. durumu
    sdKartBaslat(csBaglantisi);

   for (int i = 0; i < 5; i++) 
    {
      digitalWrite(buzzerPin, HIGH);
      delay(250);
      digitalWrite(buzzerPin, LOW);
      delay(25);
    }
    Serial.println("Ölcüm");
    mpuOlc();
    Serial.println("OK");
    pitch_i = pitch;
}

void loop() 
{
  if (sdKartCalisiyor)
    yedekSD = SD.open("ySRT.txt", FILE_WRITE);

  mpuOlc();
  
  if (AccX > 2 || AccY > 2 || AccZ > 2)
  {
    if (motorIlkGiris)
    {
      writeToSDSafe(F("Ucus Tespit Edildi! -- "));

      motorIlkGiris = false;
    }
    motorlarAcik = true;
  }

  else 
    motorlarAcik = false;
if (abs(pitch_i - pitch) > egilmeAcisi)
       {
        digitalWrite(buzzerPin, HIGH);
      delay(150);
      digitalWrite(buzzerPin, LOW);
      delay(25);
       }
       if (abs(pitch_i - pitch) < egilmeAcisi)
       {
        digitalWrite(buzzerPin, LOW);
        delay(25);
       }
       
  if(!motorlarAcik && !surPar && (abs(pitch_i - pitch) > egilmeAcisi || !suruklenmeIlkGiris_par))
  {
    if (digitalRead(b_guc) == LOW)
    {
      surPar = true;
      Serial.println(F(" SURUKLENME PARASUTU ACILDI!"));
  
      writeToSDSafe(F(" SURUKLENME PARASUTU ACILDI!"));
      
      surZaman = millis();
      digitalWrite(suruklenmeMOSFET, HIGH);
    }
   
    else if (digitalRead(b_suruklenme) == LOW)
    {
      if (suruklenmeIlkGiris_par)
      {
        surZaman = millis(); 
        suruklenmeIlkGiris_par = false;
      }

      Serial.println(F("Ana Sistem Tespit Yapmadi, ana sistem bekleniyor..."));
          
      if (millis() - surZaman > 5000 && surZaman != 0)
      {
       
        Serial.print(F("Ana Sistem Tespit Yapmadi! SURUKLENME PARASUTU ACILDI!"));
        writeToSDSafe(F("Ana Sistem Tespit Yapmadi! SURUKLENME PARASUTU ACILDI!"));
  
        surPar = true;
        digitalWrite(suruklenmeMOSFET, HIGH);
      }
    }
  }

  if(!motorlarAcik && surPar && !anaPar && millis() - surZaman > 60000 && surZaman != 0)
  {
    if (digitalRead(b_guc) == LOW || (digitalRead(b_ana) == LOW && millis() - surZaman > 65000))
    {
      Serial.println(F("Ana Sistemin Gucu Yok! ANA PARASUT ACILDI!"));
      writeToSDSafe(F("Ana Sistemin Gucu Yok! ANA PARASUT ACILDI!"));
      anaPar = true;
      digitalWrite(anaMOSFET,HIGH);
    }
  }
  
  Serial.print(surZaman); Serial.print(F(" || ")); Serial.print(millis()); Serial.print(F(" || "));
  Serial.print(F("Roll: ")); Serial.print(roll); Serial.print(F(" Pitch: ")); 
  Serial.print(pitch); Serial.print(F(" Yaw: ")); Serial.print(yaw); 

  Serial.print(F("|| Ax: ")); Serial.print(AccX); Serial.print(F("  Ay: ")); Serial.print(AccY); 
  Serial.print(F(" Az: ")); Serial.println(AccZ);

  if (sdKartCalisiyor)
  {
    yedekSD.print("Zaman: " + String(millis() / 1000.0) + " --> Acilar (r,p,y): ");
    yedekSD.print(String(roll) + "/" + String(pitch)) + "/" + String(yaw) + "-- Ivmeler: (x,y,z))";
    yedekSD.println(String(AccX) + " " + String(AccY) + " " + String(AccZ)); 
    yedekSD.close();
  }

  delay(150);
}
