int kompas(){
  byte highByte, lowByte, fine;
  int bearing;
  Wire.beginTransmission(ADDRESS);
  Wire.write(2);
  Wire.endTransmission();

  Wire.requestFrom(ADDRESS, 2);
  while(Wire.available()){
    highByte = Wire.read();
    lowByte = Wire.read();
  }
    
  bearing = ((highByte<<8)+lowByte)/10;
  fine = ((highByte<<8)+lowByte)%10;
//  Serial.println(bearing);

  return bearing;
}
