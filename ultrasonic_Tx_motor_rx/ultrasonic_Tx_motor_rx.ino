//front
#include <SPI.h> 
#include <mcp2515.h>
#include "Nunchuk.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//***********************************macro*********************************//
#define PIN_MOTOR_M1_IN1 8      //Define the positive pole of M1 
#define PIN_MOTOR_M1_IN2 9      //Define the negative pole of M1
#define PIN_MOTOR_M2_IN1 6      //Define the positive pole of M2
#define PIN_MOTOR_M2_IN2 7      //Define the negative pole of M2

#define slow_mode 5
#define average_mode 12
#define fast_mode 25

//***********************************Variable*********************************//
Adafruit_PWMServoDriver pwm_motor = Adafruit_PWMServoDriver(0x5F);
int mappedValue = 0;
int trigPin = 7;                  
int echoPin = 8;                
int distance; 
int currentSpeed;

unsigned long previousCANMillis = 0;  // CAN 메시지 전송 시간 기록
unsigned long CANInterval = 10;       // 10ms 주기
const int step = 2;


struct can_frame canMsg1;
struct can_frame canMsg2;
MCP2515 mcp2515(4);

//***********************************typedef*********************************//
int joyStickFlag;

//***********************************function*********************************//
float getDistance(); //초음파 거리 측정
void motorPWM(int channel, int motor_speed); //모터 속도
void Motor(int Motor_ID, int dir, int Motor_speed); // 현재 속도 반환

//***********************************setup*********************************//
void setup() {
  Serial.begin(115200);
  pwm_motor.begin();
  pwm_motor.setPWMFreq(1600);
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);  
  
  Wire.begin();   
  nunchuk_init();
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  Serial.println("Start");
}

//***********************************loop*********************************//
void loop(){
  //초음파
  unsigned long currentMillis = millis();
  if (currentMillis - previousCANMillis >= CANInterval) {
    previousCANMillis = currentMillis;
    distance = getDistance();  // ultrasonic
    
    canMsg1.can_id  = 0xF6;
    canMsg1.can_dlc = 8;
    canMsg1.data[0] = distance; //canData[0]에 초음파의 거리값이 들어가게 된다. 
  } 
    mcp2515.sendMessage(&canMsg1);


  // if (nunchuk_read()) { // wii data
  //   int joyStickFlag = nunchuk_joystickY();
    
  //   if(joyStickFlag){
  //      while(currentSpeed!=30){ currentSpeed += 5; }
  //   }
  // }

  // stm32로 부터 데이터를 받을 때 처리 
  if(canMsg2.can_id == 0xAAA){
      Serial.println("CanSTM32Data");
    //Motor(채널, 방향, 조절하고 싶은 모터의 속도)
      if(canMsg2.data[0] == fast_mode){ // 초음파가 15미만이 아니고 wii 값이 30초과이고 50이 아니면 모터 속도 15
          Motor(1,1,15);          // 50이 아닌 조건을 넣은 이유는 모터 가속도를 값을 추출하기위해 추가하였음  
         // Motor(2,1,15, 1);
      }
      else if(canMsg2.data[0] == average_mode){  // 가속도를 위해 wii 값이 50이면 모터 속도 30
         Motor(1,1,30);
          //Motor(2,1,30, 1);
    }
      else if(canMsg2.data[0] == slow_mode){  // 초음파 값이 15미만이면 메시지 값으로 0을 받고 0이면 모터 정지
          Motor(1,0,0);
          //Motor(2,0,0, 1);
      }
      else{ // 초기 상태 값, 즉 아무 값이 들어오지 않은 상태 - > 모터 정지 상태임
          Motor(1,0,0);
          //Motor(2,0,0);
      }
  }

}

//***********************************function define*********************************//

//-------------------------- Ultrasonic distance------------------------

float getDistance() {
  unsigned long duration; 
  float distance;        
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);  
  
  duration = pulseIn(echoPin, HIGH); 
  distance = duration * 0.0344 / 2;

  return distance;
}

//--------------------------Control motor rotation(PWM velocity)------------------------//
//motorPWM 함수: 모터의 속도를 제어하기 위해 PWM 신호를 생성합니다.
 void motorPWM(int channel, int motor_speed){
  
  if (currentSpeed < motor_speed) {
        currentSpeed += step;  // 점진적으로 증가
    } else if (currentSpeed > motor_speed) {
        currentSpeed -= step;  // 점진적으로 감소  
  }


  motor_speed = constrain(motor_speed, 0, 50); //속도 제한
  int motor_pwm = map(motor_speed, 0, 50, 0, 4095); // PWM값 변환  
  if (motor_pwm == 4095){ 
    pwm_motor.setPWM(channel, 4096, 0);  //max_velocity
  }
  else if (motor_pwm == 0){
    pwm_motor.setPWM(channel, 0, 4096);  //stop
  }
  else{
    pwm_motor.setPWM(channel, 0, motor_pwm);  //average speed
    // pwm_motor.setPWM(channel, 0, 4095 - motor_pwm);
  }
  Serial.print("motor_pwm : ");
  Serial.println(motor_pwm);
}


//------------------------Control motor rotation(direction)--------------------------
//Motor 함수: 모터의 방향과 속도를 설정하여 정방향, 역방향, 또는 정지를 제어합니다.

void Motor(int Motor_ID, int dir, int Motor_speed){
  if (Motor_ID < 1 || Motor_ID > 2) return; // ID 범위 확인

  if(dir > 0){dir = 1;}
  else if (dir < 0) {dir = -1;}
  else {dir = 0;}


  if (Motor_ID == 1){
    int targetSpeed = Motor_speed;  // 목표 속도 설정
    while (currentSpeed != targetSpeed) {
      if (currentSpeed < targetSpeed) {
        currentSpeed += step;  // 점진적으로 증가
        if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;  // 과도하게 증가하지 않도록
      }
      else if (currentSpeed > targetSpeed) {
        currentSpeed -= step;  // 점진적으로 감소
        if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;  // 과도하게 감소하지 않도록
      }

      // 모터의 속도 업데이트
      motorPWM(PIN_MOTOR_M1_IN1, currentSpeed);
      motorPWM(PIN_MOTOR_M1_IN2, currentSpeed);

      // 잠시 대기 (속도 변화를 부드럽게 하기 위해)
      delay(20);  // 속도 변화가 너무 빠르지 않도록 적절한 시간 간격 설정
    }
  }
  else if (Motor_ID == 2) {
    // 모터 2에 대한 처리 (동일한 방식으로 처리 가능)
  }
  return Motor_speed;
  Serial.print("Motor_speed : ");
  Serial.println(Motor_speed);
}