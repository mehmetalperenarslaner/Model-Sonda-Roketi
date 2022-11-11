void sdKartBaslat(const byte CS)
{
  if (!SD.begin(CS)) 
  {
    Serial.println("HATA! Sd kart modulu baslatilamadi!");
   
    for (int i = 0; i < 15; i++)
    {
      digitalWrite(buzzerPin, HIGH);
      delay(750);
      digitalWrite(buzzerPin, LOW);
      delay(25);
    }
    sdKartCalisiyor = false; 
  }

  else
  {
    Serial.println("SD kart modulu basarili bir sekilde basladi!");
    sdKartCalisiyor = true;
  }
    
}

void writeToSDSafe(String string) 
{
  if(sdKartCalisiyor)
    yedekSD.println(string);
}

void mpuBaslat()
{
    Serial.println(F("MPU Baslatiliyor..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 baglantisi basarili!") : F("MPU6050 baglantisi basarisiz!!!!"));

    Serial.println(F("DMP Baslatiliyor..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 

    if (devStatus == 0) 
    {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        Serial.println(F("DMP Aciliyor..."));
        mpu.setDMPEnabled(true);

        Serial.print(F("Interrupt tespiti aciliyor..."));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP HAZIR! Ilk interrupt bekleniyor..."));
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    } 

    else 
    {
        Serial.print(F("DMP Basarisiz! (kod: "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        while(true)
        {
          digitalWrite(buzzerPin, HIGH);
          delay(750);
          digitalWrite(buzzerPin, LOW);
          delay(25);
        }
    }
}

void mpuOlc()
{
  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) 
  {
      if (mpuInterrupt && fifoCount < packetSize) 
      {
        fifoCount = mpu.getFIFOCount();
      }  
  }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

  if(fifoCount < packetSize)
  {
   while (fifoCount < packetSize)
   fifoCount = mpu.getFIFOCount();
  }
    else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) 
    {
       
        mpu.resetFIFO();     
        Serial.println(F("FIFO overflow!"));

    } 
    else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
     {

         while(fifoCount >= packetSize)
     { 
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
     }

            
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yaw = ypr[0] * 180/M_PI;
            pitch = ypr[1] * 180/M_PI;
            roll = ypr[2] * 180/M_PI;

            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            AccX = aaWorld.x / 16384.0;
            AccY = aaWorld.y / 16384.0;
            AccZ = aaWorld.z / 16384.0;
    }
}
