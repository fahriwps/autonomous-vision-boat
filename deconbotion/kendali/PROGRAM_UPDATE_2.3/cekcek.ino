void cekRemote(){
  channel_1 = pulseIn(CHANNEL_1, HIGH, 25000);
  channel_2 = pulseIn(CHANNEL_2, HIGH, 25000);
  channel_3 = pulseIn(CHANNEL_3, HIGH, 25000);

  String out = (String)channel_1 + " " + (String)channel_2 + " " + (String)channel_3; 
  
//  Serial.println(out);
}

int a = 1300;
void cekMotor(){
  if(Serial.available()){
    a = Serial.parseInt();
  }
  kec_gerak(a, a);
}

void cekKompas(){
  int kanan, kiri;
  int nilai_kompas = kompas();
  error_kompas = 180 - (nilai_kompas + selisih_kompas);
  if (error_kompas>180){
    error_kompas = -180+(error_kompas%180);
  } else if (error_kompas<-180){
    error_kompas = 180+(error_kompas%180);
  }
  Serial.println(error_kompas);
}

