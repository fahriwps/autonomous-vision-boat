void mulai(){
  while(1){
    channel_3 = pulseIn(CHANNEL_3, HIGH, 25000);
    if (channel_3 < 1000){
//      PID_kompas(1380);
//      program_otomatis();
      kamera_m();
    } else {
      program_manual();
    }
    if (BACK) break;
  }
}

void program_manual(){
  channel_1 = pulseIn(CHANNEL_1, HIGH, 25000);
  channel_2 = pulseIn(CHANNEL_2, HIGH, 25000);

  if (channel_1>=1100 && channel_1<=1500){
    nilai_remote = map(channel_1, 1100, 1500, -10, -1);  
  }
  else if (channel_1>=1700 && channel_1<= 2010){
    nilai_remote = map(channel_1, 1700, 2000, 1, 10);  
  }
  else {
    nilai_remote = 0;
  }

/*
  if (channel_2>=1300 && channel_2<=2000){
    nilai_remote = map(channel_2, 1400, 2000, 1400, 2000);  
  }
  else if (channel_2>=700 && channel_2<= 1100){
    nilai_remote = map(channel_2, 700, 1200, 1, 10);  
  }
  else {
    channel_2 = 1300;
  }
*/

//  Serial.println(channel_2);
  
  PID_remote(channel_2, nilai_remote);  
}

void program_otomatis(){
  while (Serial.available())
  {  
    float koorX=Serial.parseInt();
    //float Radius=Serial.parseInt();
    int SetP_deteksi = 320;
    if(koorX=='y'){
      if (i>15) {kec_gerak(1380,1220);}
      else if(i<-15){kec_gerak(1220,1380);}   
    PID_kompas(1380);}
       
    else{PID_deteksi(SetP_deteksi,koorX,1380) ;
    }
    Serial.println(9999);
    
    Serial.println(koorX);
//    Serial.println(Radius);

    if (BACK){delay(100); lcd.clear(); break;}
  }
}

void kamera_m(){
  int koorX;
  bool set_m = 0;
  while(1){
    if(OK) {delay(100); set_m=1;}
    if (!set_m) {
      tampil(0,0,"CLICK OK . . . .");
      kec_gerak(1300, 1300);
    }
    else { tampil(0,0,"CAMERA START . .");
      if(Serial.available()>0)
      {  
        koorX =Serial.parseInt();
        tampil(0,1,"SIGNAL = %6d", koorX);
      }
      
      if(koorX==100){
        kec_gerak(1360,1360);
      } else {
        PID_kamera(1380, koorX);   
      }
      
    }
    if (BACK){kec_gerak(1300, 1300); delay(100);break;}
  }
}

void kompas_m(){
  bool set_m = 0;
  while(1){
    if(OK) {delay(100); set_m=1;}
    if (!set_m) {
      tampil(0,0,"CLICK OK . . . .");
      kec_gerak(1300, 1300);
    }
    if(set_m) {tampil(0,0,"COMPAS START . .");
      PID_kompas(1380);
    }
  
    if (BACK){kec_gerak(1300, 1300); delay(100);break;}
  }
}

