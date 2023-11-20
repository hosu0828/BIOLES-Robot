/* absolute encoder 아래 사이트 참고!!!!!!!!!!!!!!
  - https://create.arduino.cc/projecthub/zjor/dc-motor-position-control-c11c0e?ref=part&ref_id=10116&offset=2
  - https://playground.arduino.cc/Main/TimerPWMCheatsheet/
*/



#include "Servo.h"

// 서보모터 선언 -----------------------------------
Servo r_arm1, r_arm2, r_arm3, r_waist;
// -----------------------------------------------

// 이부분 다시보기**다시보*기*********다시보*기***************다시보*기******************************
char buffer0[9];   // []는 byte 수를 말함. 문자 1개당 1바이트이고 마지막에 \o가 포함되므로 총 데이터 개수 + 1 개로 놔야함.
const int buffer0_size = 22;     //@포함 자릿수!!
char buf[buffer0_size];      // 우분투에서 받는 버퍼.
// 이부분 다시보기**다시보*기*********다시보*기***************다시보*기******************************



// wheel encoder ----------------------------------
const int rwheel_encoderPinA = 0;
const int rwheel_encoderPinB = 1;
int rwheel_pos = 0;
// -------------------------------------------------



// cam encoder --------------------------------------
const int rcam_encoder = 19;
unsigned long rcam_angle, r_duration;
// --------------------------------------------------



// wheel motor --------------------------------------
const int rwheel_int1 = 4;
const int rwheel_int2 = 7;
const int rwheel_pwm = 11;
int rwheel;
// ---------------------------------------------------



// cam motor -----------------------------------------
const int rcam_int1 = 12;
const int rcam_int2 = 18;
const int rcam_pwm = 9;
int rcam;
// ----------------------------------------------------



// servo motor --------------------------------------
const int rarm_1 = 5;
const int rarm_2 = 6;
const int rarm_3 = 10;
const int rwaist = 13;
int rarm_1_ang, rarm_2_ang, rarm_3_ang, rwaist_ang;
// ---------------------------------------------------



// wheel encoder counter function --------------------
void do_rw_encA(){  rwheel_pos += (digitalRead(rwheel_encoderPinA)==digitalRead(rwheel_encoderPinB))?1:-1;}
void do_rw_encB(){  rwheel_pos += (digitalRead(rwheel_encoderPinA)==digitalRead(rwheel_encoderPinB))?-1:1;}
// ----------------------------------------------------



// do wheel motor 
void do_wheel(){

    if(rwheel > 0){
      digitalWrite(rwheel_int1,HIGH);
      digitalWrite(rwheel_int2,LOW);
      analogWrite(rwheel_pwm,rwheel);
    }
    else if(rwheel < 0){
      digitalWrite(rwheel_int1,LOW);
      digitalWrite(rwheel_int2,HIGH);
      analogWrite(rwheel_pwm,-1*(rwheel));
    }
    else{
      digitalWrite(rwheel_int1,HIGH);
      digitalWrite(rwheel_int2,HIGH);
      analogWrite(rwheel_pwm,0);
    }
    
}



// do cam motor 
void do_cam(){

    
    if(rcam > 0){
      digitalWrite(rcam_int1,HIGH);
      digitalWrite(rcam_int2,LOW);
      analogWrite(rcam_pwm,rcam);
    }
    else if(rcam < 0){
      digitalWrite(rcam_int1,LOW);
      digitalWrite(rcam_int2,HIGH);
      analogWrite(rcam_pwm,-1*(rcam));
    }
    else{
      digitalWrite(rcam_int1,HIGH);
      digitalWrite(rcam_int2,HIGH);
      analogWrite(rcam_pwm,0);
    }
}



// absolute cam encoder calculate function
void abenc(){  

  // calculate cam encoder angle ----------------------------------
  r_duration = pulseIn(rcam_encoder, HIGH);
  rcam_angle = ((r_duration-250)*360/(8530-250));
  //rcam_angle = ((r_duration-240)*360)/(8192-240);
}



void setup() {
  
  Serial.begin(500000);
  //TCCR2B = TCCR2B & 0b11111000 | 0x01;
  //TCCR3B = TCCR3B & 0b11111000 | 0x01;
  //Serial.setTimeout(10);



  // wheel encoder pin setting ------------------------------------
  pinMode (rwheel_encoderPinA,INPUT_PULLUP);
  pinMode (rwheel_encoderPinB,INPUT_PULLUP);
  // ---------------------------------------------------------------



  // wheel motor pin setting ---------------------------------------
  pinMode (rwheel_int1, OUTPUT);
  pinMode (rwheel_int2, OUTPUT);
  pinMode (rwheel_pwm, OUTPUT);
  // ---------------------------------------------------------------



  // wheel motor pin setting ---------------------------------------
  pinMode (rcam_int1, OUTPUT);
  pinMode (rcam_int2, OUTPUT);
  pinMode (rcam_pwm, OUTPUT);
  // ---------------------------------------------------------------



  // servo motor pin setting-------------------------------------
  r_arm1.attach(rarm_1);
  r_arm2.attach(rarm_2);
  r_arm3.attach(rarm_3);
  r_waist.attach(rwaist);

  // initial setting
  r_arm1.write(105);
  r_arm2.write(90);    // standing : 35, wide : 90
  r_arm3.write(97); 
  r_waist.write(95); 
  // ------------------------------------------------------------


  // wheel encoder interrupt setting ----------------------------
  attachInterrupt(2,do_rw_encA,CHANGE);
  attachInterrupt(3,do_rw_encB,CHANGE);
  // -------------------------------------------------------------  
}





void loop(){

  // calculate encoder value to degree ----------------------------
  // 계산하는 부분 다시 확인하기 확인하기 확인하기 확인하기
  float right_wheel_degree = float(rwheel_pos)*6/52;
  // --------------------------------------------------------------

  // calculate absolute encoder value to degree -------------------
  abenc();
  // --------------------------------------------------------------

  // **************** Serial Communication ************************

  // start serial (identify)
  Serial.print("!");

  // right wheel degree data
  dtostrf(right_wheel_degree,8,2,buffer0);
  Serial.print(buffer0);

  // right cam angle data
  dtostrf(rcam_angle,8,2,buffer0);
  Serial.print(buffer0);

  // End of Serial buffer.
  Serial.println("@");


  // Read ros2 --> data ********* 임시 데이터 이므로 값이 늘어나면 바꿔주기!!!!!!!!
  if(Serial.available() > 0) 
   {
     int rlen = Serial.readBytesUntil('$',buf, buffer0_size);  //data값이 정확히 들어왔는지 확인하고 실행.

     if(rlen == 21){
       char data1[5] = {buf[1],buf[2],buf[3],buf[4]};
       char data2[5] = {buf[5],buf[6],buf[7],buf[8]};
       char data3[4] = {buf[9],buf[10],buf[11]};
       char data4[4] = {buf[12],buf[13],buf[14]};
       char data5[4] = {buf[15],buf[16],buf[17]};
       char data6[4] = {buf[18],buf[19],buf[20]};
  
       rwheel = atoi(data1);
       rcam = atoi(data2);
       rwaist_ang = atoi(data3);
       rarm_1_ang = atoi(data4);
       rarm_2_ang = atoi(data5);
       rarm_3_ang = atoi(data6);

       r_arm1.write(rarm_1_ang);
       r_arm2.write(rarm_2_ang);
       r_arm3.write(rarm_3_ang);
       r_waist.write(rwaist_ang);
    
       do_wheel();
       do_cam();
     }

     for(int i = 0; i<buffer0_size+1; i++){
      buf[i] = '\0';
      }
   } 
}
