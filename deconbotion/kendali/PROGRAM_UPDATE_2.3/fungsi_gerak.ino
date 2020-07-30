
void kec_gerak(unsigned int kanan, unsigned int kiri){
  kanan +=  tambah_ka;
  kiri  +=  tambah_ki;
  
  if (kiri > limit_atas)        kiri  = limit_atas;
  else if (kiri < limit_bawah)  kiri  = limit_bawah;
  if (kanan > limit_atas)       kanan = limit_atas;
  else if (kanan < limit_bawah) kanan = limit_bawah;
  
  motor_ka.writeMicroseconds(kanan);
  motor_ki.writeMicroseconds(kiri);
 // Serial.print(kanan);
  //Serial.print(' ');
  //Serial.println(kiri);
}

void PID_remote(int kecepatan, int nilai){ // belok kanan
  int kanan, kiri;
  error_remote=set_point_remote-nilai;
  
  kanan=kecepatan+kp_remote*error_remote+kd_remote*(error_remote-last_error_remote);
  kiri=kecepatan-kp_remote*error_remote-kd_remote*(error_remote-last_error_remote);
  
  last_error_remote=error_remote;

  kec_gerak(kanan, kiri);
}

void PID_kompas(int kecepatan){
  int kanan, kiri;
  int nilai_kompas = kompas();
  error_kompas = 180 - (nilai_kompas + selisih_kompas);

  if (error_kompas>180){
    error_kompas = -180+(error_kompas%180);
  } else if (error_kompas<-180){
    error_kompas = 180+(error_kompas%180);
  }
  
  kanan=kecepatan-kp_kompas*error_kompas-kd_kompas*(error_kompas-last_error_kompas);
  kiri=kecepatan+kp_kompas*error_kompas+kd_kompas*(error_kompas-last_error_kompas);

  last_error_kompas=error_kompas;

  kec_gerak(kanan, kiri);
//  Serial.println(error_kompas);
}

void PID_deteksi(float SetpointX, float deteksiX, int kecepatan)
{
  int kanan,kiri;
  float errorX=SetpointX-deteksiX;
  
  kanan=kecepatan-kp_kamera*errorX-kd_kamera*(errorX-last_errorX);
  kiri=kecepatan+kp_kamera*errorX+kd_kamera*(errorX-last_errorX); 
  if(errorX<=0){i++;}
  else{i--;}
  last_errorX=errorX;
  kec_gerak(kanan, kiri);
  
}

void PID_kamera(int kecepatan, int nilai){ // belok kanan
  int kanan, kiri;
  error_kamera=10-nilai;
  
  kanan=kecepatan+kp_kamera*error_kamera+kd_kamera*(error_kamera-last_error_kamera);
  kiri=kecepatan-kp_kamera*error_kamera-kd_kamera*(error_kamera-last_error_kamera);
  
  last_error_kamera=error_kamera;

  kec_gerak(kanan, kiri);
}
