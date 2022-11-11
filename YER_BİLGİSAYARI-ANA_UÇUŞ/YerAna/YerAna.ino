
#include <LoRaUnoLib.h>

SX1278 lora = new LoRa;
//const unsigned int frekans = 433.3893; 

void setup() 
{
  Serial.begin(9600);
 
  
  int state = lora.begin(); // LoRa'yi baslat ve begin methodunun dondurdugunu kaydet

  if (state == ERR_NONE) // Eger state "HATASIZ" ise: 
  {
    Serial.println("RFM98W LoRa Modulu Basarili Bir Sekilde Baslatildi!");
    lora.setFrequency(434.69);
  } 

  else // Degilse: 
    Serial.println("RFM98W LoRa Modulu Baslatilamadi!!! Hata Kodu: " + String(state));
}
void loop() 
{
  
  String str;

  int state = lora.receive(str);

  if (state == ERR_NONE)
  {
    Serial.print(str);
    Serial.print("\n");
    
  } 
    
  else if (state == ERR_RX_TIMEOUT) 
  {
    Serial.print(F("Zaman Asimi!\n"));
    
  } 

  else if (state == ERR_CRC_MISMATCH) 
    Serial.print(F("CRC hatasi!\n"));

  delay(750);
}
