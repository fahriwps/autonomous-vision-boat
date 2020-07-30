void kalibrasi()
{
  while(Serial.available()>0){
    Serial.read();
  } // clear buffer :)
  Serial.println("This is begin compass calibration");
  Wire.beginTransmission(0x60);
  Wire.write(22);
  Wire.write(0xF0);
  Wire.endTransmission();  

  Serial.println("Place your compass to 0 deg (North) and hit a key");
  while(Serial.available()==0); // wait input from user !
  while(Serial.available()>0){
    Serial.read();
  } // clear buffer :)
  Wire.beginTransmission(0x60);
  Wire.write(22);
  Wire.write(0xF5);
  Wire.endTransmission();  

  Serial.println("Place your compass to 90 deg (East) and hit a key");
  while(Serial.available()==0); // wait input from user !
  while(Serial.available()>0){
    Serial.read();
  } // clear buffer :)
  Wire.beginTransmission(0x60);
  Wire.write(22);
  Wire.write(0xF5);
  Wire.endTransmission();

  Serial.println("Place your compass to 180 deg (South) and hit a key");
  while(Serial.available()==0); // wait input from user !
  while(Serial.available()>0){
    Serial.read();
  } // clear buffer :)
  Wire.beginTransmission(0x60);
  Wire.write(22);
  Wire.write(0xF5);
  Wire.endTransmission();  

  Serial.println("Place your compass to 270 deg (West) and hit a key");
  while(Serial.available()==0); // wait input from user !
  while(Serial.available()>0){
    Serial.read();
  } // clear buffer :)
  Wire.beginTransmission(0x60);
  Wire.write(22);
  Wire.write(0xF5);
  Wire.endTransmission();
  Serial.println("Calibration finished.");
}
