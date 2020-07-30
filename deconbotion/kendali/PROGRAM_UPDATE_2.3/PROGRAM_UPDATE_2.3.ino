#include <Servo.h>
#include <Wire.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>  

#define ADDRESS 0x60

//pin untuk motor
#define MOTOR_KA    6
#define MOTOR_KI    5

//pin untuk channel remote
#define CHANNEL_1   A1
#define CHANNEL_2   A2
#define CHANNEL_3   A3

//pin untuk button
#define PREV                  ( analogRead(A0)>850 && analogRead(A0)<950 )  // 235-- 236
#define NEXT                  ( analogRead(A0)>500 && analogRead(A0)<600 )  // 317
#define OK                    ( analogRead(A0)>700 && analogRead(A0)<800 )
#define BACK                  ( analogRead(A0)>600 && analogRead(A0)<700 )
/*
#define PREV                  ( !digitalRead(A5) )
#define NEXT                  ( !digitalRead(A4) )
#define UP                    ( !digitalRead(A3) )
#define DOWN                  ( !digitalRead(A2) )
*/

Servo motor_ka, motor_ki;
LiquidCrystal lcd (13, 12, 11, 10, 9, 8);

void tampil(uint8_t x,uint8_t y, const char* fmtstr, ...);
void cek_pid(), Range();
void menu();
void setting();
void Kec();

int channel_1, channel_2, channel_3;
int error_remote, last_error_remote,last_errorX;
int kp_remote = 0, kd_remote = 0;
int kec_awal = 1300, nilai_remote = 0;
int set_point_remote=0;

unsigned int e_kp, e_kd, e_MotorKa, e_MotorKi;
unsigned int kanan, kiri;
float Kp, Kd;
int tambah_ka, tambah_ki;

int kp_kamera, kd_kamera;

unsigned int e_limit_atas, e_limit_bawah, limit_atas, limit_bawah;
unsigned int cek_spd;


int kompas();
int set_point_kompas, selisih_kompas;
int error_kompas, last_error_kompas; 
int kp_kompas = 0, kd_kompas = 0;
int i;

int error_kamera, last_error_kamera;

void kalibrasi();
void kec_gerak(unsigned int,unsigned int);

void setup() {
  //initsialisasi
  //set motor pin sebagai output
  pinMode(MOTOR_KA, OUTPUT);
  pinMode(MOTOR_KI, OUTPUT);
  
  //set channel pin sebagai input
  pinMode(CHANNEL_1, INPUT);
  pinMode(CHANNEL_2, INPUT);
  pinMode(CHANNEL_3, INPUT);
  
  pinMode(A0, INPUT);
  
  //set pin motor 
  motor_ka.attach(MOTOR_KA);
  motor_ki.attach(MOTOR_KI);
    
  //serial monitor
  lcd.begin(16,2);
  Serial.begin(9600);
  Wire.begin();
  //Serial.setTimeout(1000);

//  EEPROM.update(4, 0);
//  EEPROM.update(6, 0);
//  limit_atas = 1400;
//  limit_bawah = 700; 
//
//  unsigned int cc = 1300;
//  EEPROM.put(22, cc);
//  EEPROM.write(18, 0);
//  EEPROM.write(20, 0);
  
  //eeprom
  kp_remote = EEPROM.read(0);
  kd_remote = EEPROM.read(2);
  kp_kompas = EEPROM.read(4);
  kd_kompas = EEPROM.read(6);
  kp_kamera = EEPROM.read(18);
  kd_kamera = EEPROM.read(20);
  EEPROM.get(10, limit_atas);
  EEPROM.get(12, limit_bawah);
  EEPROM.get(14, tambah_ka);
  EEPROM.get(16, tambah_ki);
  EEPROM.get(22, cek_spd);

  delay(100);
  //pengaturan awal
  Serial.println("===== MULAI PROGRAM =====");
  
  //pengaturan awal motor
  kec_gerak(1300, 1300);
  set_point_kompas = kompas();
  selisih_kompas = 180 - set_point_kompas;  
  
  tampil(0,0,"  GAMANTARAY  ");
  tampil(0,1,"    JUARA     ");
  delay(1500);
  lcd.clear();

  Serial.setTimeout(30);
  
  Serial.println("========= DONE ==========");
}

void loop() {

  menu();
//cekKompas();
//  kamera_m();
  delay(10);
}

