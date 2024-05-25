#include <NewPing.h>

#define SONAR_NUM 3      
#define MAX_DISTANCE 100

#define Front 0
#define Left  1 
#define Right 2

#define TRIG1 31 // 초음파 센서 1번 Trig 핀 번호
#define ECHO1 30 // 초음파 센서 1번 Echo 핀 번호

#define TRIG2 27 // 초음파 센서 2번 Trig 핀 번호
#define ECHO2 26 // 초음파 센서 2번 Echo 핀 번호

#define TRIG3 34 // 초음파 센서 3번 Trig 핀 번호
#define ECHO3 35 // 초음파 센서 3번 Echo 핀 번호

NewPing sonar[SONAR_NUM] = {   
NewPing(TRIG1, ECHO1, MAX_DISTANCE), 
NewPing(TRIG2, ECHO2, MAX_DISTANCE),
NewPing(TRIG3, ECHO3, MAX_DISTANCE)
};

float front_sonar = 0.0;
float left_sonar  = 0.0;
float right_sonar = 0.0;


float alpha = 0.1;
float front_filtered = 0.0;
float left_filtered = 0.0;
float right_filtered = 0.0;

void setup() 
{
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);

  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);
  
  Serial.begin(115200); 
}

void loop() 
{

  front_sonar = sonar[Front].ping_cm() * 10; 
  left_sonar  = sonar[Left].ping_cm() * 10;  
  right_sonar = sonar[Right].ping_cm() * 10; 

  if(front_sonar == 0.0) front_sonar = MAX_DISTANCE * 10; 
  if(left_sonar  == 0.0) left_sonar  = MAX_DISTANCE * 10;
  if(right_sonar == 0.0) right_sonar = MAX_DISTANCE * 10;


  front_filtered = alpha * front_sonar + (1 - alpha) * front_filtered;
  left_filtered  = alpha * left_sonar  + (1 - alpha) * left_filtered;
  right_filtered = alpha * right_sonar + (1 - alpha) * right_filtered;

  Serial.print("정면(재귀평균필터): "); Serial.print(front_filtered); Serial.print(" cm, ");
  Serial.print("정면(필터없음): "); Serial.print(front_sonar); Serial.print(" cm | ");

  Serial.print("왼쪽(재귀평균필터): "); Serial.print(left_filtered); Serial.print(" cm, ");
  Serial.print("왼쪽(필터없음): "); Serial.print(left_sonar); Serial.print(" cm | ");

  Serial.print("오른쪽(재귀평균필터): "); Serial.print(right_filtered); Serial.print(" cm, ");
  Serial.print("오른쪽(필터없음): "); Serial.print(right_sonar); Serial.println(" cm");

  delay(100);

}
