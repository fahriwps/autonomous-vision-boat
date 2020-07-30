void menu()
{
  int tab=1;
  while(1)
  {
//    lcd.clear();
    tampil(0,0,"   GAMANTARAY   ");
    if(tab==1){delay(100);tampil(0,1,"<     START    >");}
    if(tab==2){delay(100);tampil(0,1,"<  CAMERA ONLY >");}
    if(tab==3){delay(100);tampil(0,1,"<  COMPAS ONLY >");}
    if(tab==4){delay(100);tampil(0,1,"<    SETTING   >");}
    
    if(tab==1&&OK){delay(100); lcd.clear(); tampil(0,0,"====BRANGKAT===="); mulai();}
    if(tab==2&&OK){delay(100); lcd.clear(); kamera_m();}
    if(tab==3&&OK){delay(100); lcd.clear(); kompas_m();}
    if(tab==4&&OK){delay(100); lcd.clear(); setting_m();}
    
    if(NEXT){delay(100);tab++;}
    if(PREV){delay(100);tab--;}
    if(tab==5){tab=1;}
    if(tab==0){tab=4;}
  }
}

void setting_m()
{
  int tab1=1;
  while(1)
  {
    tampil(0,0,"     SETTING    ");
    if(tab1==1){delay(100);tampil(0,1,"<  PID REMOTE  >");}
    if(tab1==2){delay(100);tampil(0,1,"<  PID KAMERA  >");}
    if(tab1==3){delay(100);tampil(0,1,"<  PID KOMPAS  >");}
    if(tab1==4){delay(100);tampil(0,1,"<  LIMIT MOTOR >");}
    if(tab1==5){delay(100);tampil(0,1,"<  VAL MOTOR   >");}
    if(tab1==6){delay(100);tampil(0,1,"<  CEK REMOTE  >");}
    if(tab1==7){delay(100);tampil(0,1,"<  CEK KOMPAS  >");}

    if(tab1==1&&OK){delay(100); lcd.clear();pid_remote_m();}
    if(tab1==2&&OK){delay(100); lcd.clear();pid_kamera_m();}
    if(tab1==3&&OK){delay(100); lcd.clear();pid_kompas_m();}
    if(tab1==4&&OK){delay(100); lcd.clear();limit_m();}
    if(tab1==5&&OK){delay(100); lcd.clear();val_motor_m();}
    if(tab1==6&&OK){delay(100); lcd.clear();cek_remote_m();}
    if(tab1==7&&OK){delay(100); lcd.clear();cek_kompas_m();}
    
    if(NEXT){delay(100);tab1++;}
    if(PREV){delay(100);tab1--;}
    if(tab1==8){tab1=1;}
    if(tab1==0){tab1=7;} 
    if(BACK){lcd.clear(); break;}
  }
}

void pid_remote_m()
{
  int tab3=1;
  while(1)
  { 
    delay(100);lcd.clear();
    //Setting KP Remot
    if(tab3==1){
      tampil(0,0,">KP REMOTE =%4d", kp_remote);
      tampil(0,1," KD REMOTE =%4d", kd_remote);
      if(NEXT)      {delay(100); kp_remote++;}
      if(PREV)      {delay(100); kp_remote--;}
    }
    
    //Setting D Remot
    if(tab3==2){
      tampil(0,0," KP REMOTE =%4d", kp_remote);
      tampil(0,1,">KD REMOTE =%4d", kd_remote);
      if(NEXT)      {delay(100); kd_remote++;}
      if(PREV)      {delay(100); kd_remote--;}
    }    
    
    if(OK){delay(100);tab3++;}
    if(BACK){delay(100);tab3--;}
    if(tab3==3){
      delay(100);
      lcd.clear(); 
      EEPROM.update(0, kp_remote); 
      EEPROM.update(2, kd_remote); 
      break;
    }
    if(tab3==0){delay(100);lcd.clear();break;}
  }
}

void pid_kompas_m()
{
  int tab3=1;
  while(1)
  { 
    delay(100);lcd.clear();
    //Setting KP Remot
    if(tab3==1){
      tampil(0,0,">KP KOMPAS =%4d", kp_kompas);
      tampil(0,1," KD KOMPAS =%4d", kd_kompas);
      if(NEXT)      {delay(100); kp_kompas++;}
      if(PREV)      {delay(100); kp_kompas--;}
    }
    
    //Setting D Remot
    if(tab3==2){
      tampil(0,0," KP KOMPAS =%4d", kp_kompas);
      tampil(0,1,">KD KOMPAS =%4d", kd_kompas);
      if(NEXT)      {delay(100); kd_kompas++;}
      if(PREV)      {delay(100); kd_kompas--;}
    }    
    
    if(OK){delay(100);tab3++;}
    if(BACK){delay(100);tab3--;}
    if(tab3==3){
      delay(100);
      lcd.clear(); 
      EEPROM.update(4, kp_kompas); 
      EEPROM.update(6, kd_kompas); 
      break;
    }
    if(tab3==0){delay(100);lcd.clear();break;}
  }
}

void pid_kamera_m()
{
  int tab3=1;
  while(1)
  { 
    delay(100);lcd.clear();
    //Setting KP Kamera
    if(tab3==1){
      tampil(0,0,">KP KAMERA =%4d", kp_kamera);
      tampil(0,1," KD KAMERA =%4d", kd_kamera);
      if(NEXT)      {delay(100); kp_kamera++;}
      if(PREV)      {delay(100); kp_kamera--;}
    }
    
    //Setting D Kamera
    if(tab3==2){
      tampil(0,0," KP KAMERA =%4d", kp_kamera);
      tampil(0,1,">KD KAMERA =%4d", kd_kamera);
      if(NEXT)      {delay(100); kd_kamera++;}
      if(PREV)      {delay(100); kd_kamera--;}
    }    
    if(NEXT&&PREV){delay(100);kp_kamera = 0; kd_kamera = 0;}
    if(OK){delay(100);tab3++;}
    if(BACK){delay(100);tab3--;}
    if(tab3==3){
      delay(100);
      lcd.clear(); 
      EEPROM.update(18, kp_kamera); 
      EEPROM.update(20, kd_kamera); 
      break;
    }
    if(tab3==0){delay(100);lcd.clear();break;}
  }
}

void limit_m()
{
  int tab3=1;
  while(1)
  { 
    delay(100);lcd.clear();
    //Setting KP Remot
    if(tab3==1){
      tampil(0,0,">LIMIT ATAS=%4d", limit_atas);
      tampil(0,1," LIMIT BWH =%4d", limit_bawah);
      if(NEXT)      {delay(100); limit_atas++;}
      if(PREV)      {delay(100); limit_atas--;}
    }
    
    //Setting D Remot
    if(tab3==2){
      tampil(0,0," LIMIT ATAS=%4d", limit_atas);
      tampil(0,1,">LIMIT BWH =%4d", limit_bawah);
      if(NEXT)      {delay(100); limit_bawah++;}
      if(PREV)      {delay(100); limit_bawah--;}
    }    
    
    if(OK){delay(100);tab3++;}
    if(BACK){delay(100);tab3--;}
    if(tab3==3){
      delay(100);
      lcd.clear(); 
      EEPROM.put(10, limit_atas); 
      EEPROM.put(12, limit_bawah); 
      break;
    }
    if(tab3==0){delay(100);lcd.clear();break;}
  }
}

void val_motor_m()
{
  int tab3=1;
  bool cek_on=false;
  while(1)
  { 
    delay(100);lcd.clear();

    if(tab3==1){
      tampil(0,0,">ON=%d    SP=%4d",cek_on, cek_spd);
      tampil(0,1," KA=%4d KI=%4d", tambah_ka, tambah_ki);
      if(NEXT)      {delay(100); cek_on=true;}
      if(PREV)      {delay(100); cek_on=false;}
    }
    
    if(tab3==2){
      tampil(0,0," ON=%d   >SP=%4d",cek_on, cek_spd);
      tampil(0,1," KA=%4d KI=%4d", tambah_ka, tambah_ki);
      if(NEXT)      {delay(100); cek_spd++;}
      if(PREV)      {delay(100); cek_spd--;}
    }
    
    //Setting KP Remot
    if(tab3==3){
      tampil(0,0," ON=%d    SP=%4d",cek_on, cek_spd);
      tampil(0,1,">KA=%4d KI=%4d", tambah_ka, tambah_ki);
      if(NEXT)      {delay(100); tambah_ka++;}
      if(PREV)      {delay(100); tambah_ka--;}
    }
    
    //Setting D Remot
    if(tab3==4){
      tampil(0,0," ON=%d    SP=%4d",cek_on, cek_spd);
      tampil(0,1," KA=%4d>KI=%4d", tambah_ka, tambah_ki);
      if(NEXT)      {delay(100); tambah_ki++;}
      if(PREV)      {delay(100); tambah_ki--;}
    }    
    
    if(OK){delay(100);tab3++;}
    if(BACK){delay(100);tab3--;}
    if(tab3==5){
      delay(100);
      lcd.clear(); 
      
      EEPROM.put(22, cek_spd); 
      EEPROM.put(14, tambah_ka); 
      EEPROM.put(16, tambah_ki); 
      kec_gerak(1300, 1300);
      break;
    }
    if (!cek_on) kec_gerak(1300,1300);
    else kec_gerak(cek_spd, cek_spd);
    if(tab3==0){delay(100);lcd.clear();kec_gerak(1300, 1300);break;}
  }
}

void cek_remote_m(){
  lcd.clear();
  while(1){
    
    channel_1 = pulseIn(CHANNEL_1, HIGH, 25000);
    channel_2 = pulseIn(CHANNEL_2, HIGH, 25000);
    channel_3 = pulseIn(CHANNEL_3, HIGH, 25000);

    tampil(0,0,"C1=%4d  C2=%4d", channel_1, channel_2);
    tampil(0,1,"     C3=%4d    ", channel_3);
    if(OK){delay(100);break;}
    if(BACK){delay(100);break;}
    
    delay(100);
    
  }
}

void cek_kompas_m(){
  lcd.clear();
  while(1){
    
    int nilai_kompas = kompas();
    error_kompas = 180 - (nilai_kompas + selisih_kompas);
    if (error_kompas>180){
      error_kompas = -180+(error_kompas%180);
    } else if (error_kompas<-180){
      error_kompas = 180+(error_kompas%180);
    }
    
    tampil(0,0,"ERROR KOM = %4d", error_kompas);
    tampil(0,1,"VALUE KOM = %4d", nilai_kompas);
    if(OK){delay(100);break;}
    if(BACK){delay(100);break;}
    
    delay(100);
    
  }
}
