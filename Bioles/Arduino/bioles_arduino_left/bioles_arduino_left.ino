/* absolute encoder 아래 사이트 참고!!!!!!!!!!!!!!
  - https://create.arduino.cc/projecthub/zjor/dc-motor-position-control-c11c0e?ref=part&ref_id=10116&offset=2
  - https://playground.arduino.cc/Main/TimerPWMCheatsheet/
*/



#include "MPU9250.h"
#include "Servo.h"

// 서보모터 선언 -----------------------------------
Servo l_arm1, l_arm2, l_arm3, l_waist;
// -----------------------------------------------

// 이부분 다시보기**다시보*기*********다시보*기***************다시보*기******************************
char buffer0[9];   // []는 byte 수를 말함. 문자 1개당 1바이트이고 마지막에 \o가 포함되므로 총 데이터 개수 + 1 개로 놔야함.
const int buffer0_size = 22;     //@포함 자릿수!!
char buf[buffer0_size];      // 우분투에서 받는 버퍼.
// 이부분 다시보기**다시보*기*********다시보*기***************다시보*기******************************



// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;
// ----------------------------------------------------------



// wheel encoder ----------------------------------
const int lwheel_encoderPinA = 0;
const int lwheel_encoderPinB = 1;
int lwheel_pos = 0;
// -------------------------------------------------



// cam encoder --------------------------------------
const int lcam_encoder = 19;
unsigned long lcam_angle, l_duration;
// --------------------------------------------------



// wheel motor --------------------------------------
const int lwheel_int1 = 4;
const int lwheel_int2 = 7;
const int lwheel_pwm = 11;
int lwheel;
// ---------------------------------------------------



// cam motor -----------------------------------------
const int lcam_int1 = 12;
const int lcam_int2 = 18;
const int lcam_pwm = 9;
int lcam;
// ----------------------------------------------------



// servo motor --------------------------------------
const int larm_1 = 5;
const int larm_2 = 6;
const int larm_3 = 10;
const int lwaist = 13;
int larm_1_ang, larm_2_ang, larm_3_ang, lwaist_ang;
// ---------------------------------------------------



// wheel encoder counter function --------------------
void do_lw_encA(){  lwheel_pos += (digitalRead(lwheel_encoderPinA)==digitalRead(lwheel_encoderPinB))?1:-1;}
void do_lw_encB(){  lwheel_pos += (digitalRead(lwheel_encoderPinA)==digitalRead(lwheel_encoderPinB))?-1:1;}
// ----------------------------------------------------



// do wheel motor 
void do_wheel(){

    
    if(lwheel > 0){
      digitalWrite(lwheel_int1,HIGH);
      digitalWrite(lwheel_int2,LOW);
      analogWrite(lwheel_pwm,lwheel);
    }
    else if(lwheel < 0){
      digitalWrite(lwheel_int1,LOW);
      digitalWrite(lwheel_int2,HIGH);
      analogWrite(lwheel_pwm,-1*(lwheel));
    }
    else{
      digitalWrite(lwheel_int1,HIGH);
      digitalWrite(lwheel_int2,HIGH);
      analogWrite(lwheel_pwm,0);
    }
}



// do cam motor 
void do_cam(){

    
    if(lcam > 0){
      digitalWrite(lcam_int1,HIGH);
      digitalWrite(lcam_int2,LOW);
      analogWrite(lcam_pwm,lcam);
    }
    else if(lcam < 0){
      digitalWrite(lcam_int1,LOW);
      digitalWrite(lcam_int2,HIGH);
      analogWrite(lcam_pwm,-1*(lcam));
    }
    else{
      digitalWrite(lcam_int1,HIGH);
      digitalWrite(lcam_int2,HIGH);
      analogWrite(lcam_pwm,0);
    }
}



// absolute cam encoder calculate function
void abenc(){  

  // calculate cam encoder angle ----------------------------------
  l_duration = pulseIn(lcam_encoder, HIGH);
  lcam_angle = ((l_duration-250)*360/(8530-250));
  //lcam_angle = ((l_duration-240)*360)/(8192-240);
}



void setup() {
  
  Serial.begin(500000);
  //TCCR2B = TCCR2B & 0b11111000 | 0x01;
  //TCCR3B = TCCR3B & 0b11111000 | 0x01;
  //Serial.setTimeout(10);

  // IMU통신을 위한 I2C 통신규격 맞추는 부분 ------------------------
  Wire.begin();
  Wire.setClock(400000);
  // ------------------------------------------------------------

  // start communication with IMU  ------------------------------
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
  }
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(0);
  // -------------------------------------------------------------



  // wheel encoder pin setting ------------------------------------
  pinMode (lwheel_encoderPinA,INPUT_PULLUP);
  pinMode (lwheel_encoderPinB,INPUT_PULLUP);
  // ---------------------------------------------------------------



  // wheel motor pin setting ---------------------------------------
  pinMode (lwheel_int1, OUTPUT);
  pinMode (lwheel_int2, OUTPUT);
  pinMode (lwheel_pwm, OUTPUT);
  // ---------------------------------------------------------------



  // cam motor pin setting ---------------------------------------
  pinMode (lcam_int1, OUTPUT);
  pinMode (lcam_int2, OUTPUT);
  pinMode (lcam_pwm, OUTPUT);
  // ---------------------------------------------------------------



  // servo motor pin setting-------------------------------------
  l_arm1.attach(larm_1);
  l_arm2.attach(larm_2);
  l_arm3.attach(larm_3);
  l_waist.attach(lwaist);


  // initial setting 
  l_arm1.write(100);   
  l_arm2.write(92);  // standing : 180, wide : 92
  l_arm3.write(90);  
  l_waist.write(95); 
  // ------------------------------------------------------------


  // wheel encoder interrupt setting ----------------------------
  attachInterrupt(2,do_lw_encA,CHANGE);
  attachInterrupt(3,do_lw_encB,CHANGE);
  // -------------------------------------------------------------  
}





void loop(){

  // read imu sensor ----------------------------------------------
  IMU.readSensor();
  // --------------------------------------------------------------

  // calculate encoder value to degree ----------------------------
  // 계산하는 부분 다시 확인하기 확인하기 확인하기 확인하기
  float left_wheel_degree = float(lwheel_pos)*6/52;
  // --------------------------------------------------------------

  // calculate absolute encoder value to degree -------------------
  abenc();
  // --------------------------------------------------------------

  // **************** Serial Communication ************************

  // start serial (identify)
  Serial.print("!");
  
  // Accl data
  dtostrf(IMU.getAccelX_mss(),8,3,buffer0);
  Serial.print(buffer0);
  dtostrf(IMU.getAccelY_mss(),8,3,buffer0);
  Serial.print(buffer0);
  dtostrf(IMU.getAccelZ_mss(),8,3,buffer0);
  Serial.print(buffer0);

  // Gyro data
  dtostrf(IMU.getGyroX_rads(),8,3,buffer0);
  Serial.print(buffer0);
  dtostrf(IMU.getGyroY_rads(),8,3,buffer0);
  Serial.print(buffer0);
  dtostrf(IMU.getGyroZ_rads(),8,3,buffer0);
  Serial.print(buffer0);

  // Compass data
  dtostrf(IMU.getMagX_uT(),8,3,buffer0);
  Serial.print(buffer0);
  dtostrf(IMU.getMagY_uT(),8,3,buffer0);
  Serial.print(buffer0);
  dtostrf(IMU.getMagZ_uT(),8,3,buffer0);
  Serial.print(buffer0);

  // left wheel degree data
  dtostrf(left_wheel_degree,8,2,buffer0);
  Serial.print(buffer0);

  // left cam angle data
  dtostrf(lcam_angle,8,2,buffer0);
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
  
       lwheel = atoi(data1);
       lcam = atoi(data2);
       lwaist_ang = atoi(data3);
       larm_1_ang = atoi(data4);
       larm_2_ang = atoi(data5);
       larm_3_ang = atoi(data6);

       l_arm1.write(larm_1_ang);
       l_arm2.write(larm_2_ang);
       l_arm3.write(larm_3_ang);
       l_waist.write(lwaist_ang);
    
       do_wheel();
       do_cam();
     }

     for(int i = 0; i<buffer0_size+1; i++){
      buf[i] = '\0';
      }
   } 
}
