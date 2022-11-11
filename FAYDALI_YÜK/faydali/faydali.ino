#include <LoRaUnoLib.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

SoftwareSerial ss(4, 3);
TinyGPSPlus gps; 
SX1278 lora = new LoRa;

unsigned long loraSonGonderim = 0; float millisZaman = 0;
bool loraCalisiyor = false;
const byte buzzerPin = 8;

void setup() 
{
  Serial.begin(9600);
  ss.begin(9600);
  pinMode(buzzerPin, OUTPUT);

  int state = lora.begin(); // LoRa'yi baslat ve begin methodunun dondurdugunu kaydet.

  if (state == ERR_NONE) // EGer state "HATASIZ" ise: 
  {
    Serial.println(F("LoRa Baglantisi Basarili!")); // Basarili!
  } 

  else // Degilse: 
  {
    Serial.print(F("LoRa Baglantisi Basarisiz! Hata Kodu: ")); // Hata var:
    Serial.println(state); // hata kodu
    while (true)
    {
      digitalWrite(buzzerPin, HIGH);
      delay(750);
      digitalWrite(buzzerPin, LOW);
      delay(250);
    } 
  }

  digitalWrite(buzzerPin, HIGH);
  delay(750);
  digitalWrite(buzzerPin, LOW);

  lora.setFrequency(433.3893);
  loraGonder("Faydali Yuk Bilgisayari Baslatildi! Konum verileri gonderiliyor...");
}
void loop() 
{
  String Veri = "fSRT: ";
  Veri += String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
  loraGonder(Veri);

  Serial.println("Gonderilen Veri: " + Veri);

  smartDelay(150);
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());

  } while (millis() - start < ms);
}

int loraGonder(String str)
{
  int st = lora.scanChannel();

  if (st == CHANNEL_FREE && millis() - loraSonGonderim > 750)
  {
    lora.transmit(str);
    loraSonGonderim = millis();
  }
}
