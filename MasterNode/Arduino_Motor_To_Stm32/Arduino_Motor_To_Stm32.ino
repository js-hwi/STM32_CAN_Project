#include <SPI.h>
#include <mcp2515.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//***********************************macro*********************************//
#define PIN_MOTOR_M1_IN1 8      //Define the positive pole of M1 
#define PIN_MOTOR_M1_IN2 9      //Define the negative pole of M1
#define PIN_MOTOR_M2_IN1 6      //Define the positive pole of M2
#define PIN_MOTOR_M2_IN2 7      //Define the negative pole of M2

//***********************************Variable*********************************//
Adafruit_PWMServoDriver pwm_motor = Adafruit_PWMServoDriver(0x5F);
int mappedValue = 0;
int trigPin = 7;                  
int echoPin = 8;                
uint8_t distance; 
uint8_t currentVelocity;

unsigned long previousCANMillis = 0;  // CAN 메시지 전송 시간 기록
unsigned long CANInterval = 10;       // 10ms 주기

struct can_frame canMsg1;
struct can_frame canMsg2;

MCP2515 mcp2515(4);
//int joyStickFlag;



//***********************************const variable*********************************//


//***********************************function*********************************//
float getDistance(); //초음파 거리 측정
void motorPWM(int channel, int motor_speed); //모터 속도
void Motor(int Motor_ID, int dir, int Motor_speed); // 현재 속도 반환
void Init_MotorVelocity();



//***********************************setup*********************************//

void setup() {
  Serial.begin(115200); // 시리얼 통신 시작
  
  pwm_motor.begin();
  pwm_motor.setPWMFreq(1600);
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);  

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  
  Init_MotorVelocity();
}


//***********************************loop*********************************//


void loop() {
  // CAN 메시지 구성
  unsigned long currentMillis = millis();
  if (currentMillis - previousCANMillis >= CANInterval) {
    previousCANMillis = currentMillis;
    distance = getDistance();  // ultrasonic
    
    canMsg1.can_id  = 0xF6;
    canMsg1.can_dlc = 8;
    canMsg1.data[0] = distance;  // 거리
    canMsg1.data[1] = currentVelocity; //모터 pwm값(속도)
  } 

  mcp2515.sendMessage(&canMsg1);


  // mcp2515.readMessage(&canMsg2); // 수신 요청
  // if(canMsg2.can_id == 0x77){
  //     Serial.println("canMsg를 수신했습니다.");
  //     Motor(1,1,canMsg2.data[1]);
  //     Motor(2,1,canMsg2.data[1]);
  // }
  delay(1000); // 1초 대기
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
  motor_speed = constrain(motor_speed, 0, 50);
  currentVelocity = motor_speed;
  //Serial.print("현재 속력값은:");
  //Serial.println(currentVelocity);
  int motor_pwm = map(motor_speed, 0, 50, 0, 4095);
  if (motor_pwm == 4095){
    pwm_motor.setPWM(channel, 4096, 0);
  }
  else if (motor_pwm == 0){
    pwm_motor.setPWM(channel, 0, 4096);
  }
  else{
    pwm_motor.setPWM(channel, 0, motor_pwm);
    // pwm_motor.setPWM(channel, 0, 4095 - motor_pwm);
  }
}


//------------------------Control motor rotation(direction)--------------------------
//Motor 함수: 모터의 방향과 속도를 설정하여 정방향, 역방향, 또는 정지를 제어합니다.
void Motor(int Motor_ID, int dir, int Motor_speed){
  //Motor(motor_ID, direction, speed)
  // motor_ID: Motor number, 1-2(M1~M2)
  // direction: Motor rotation direction. 1 or -1.
  // speed: Motor speed. 0-100.
  if(dir > 0){dir = 1;}
  else if (dir < 0) {dir = -1;}
  else {dir = 0;}

  if (Motor_ID == 1){
    if (dir == 1){
      motorPWM(PIN_MOTOR_M1_IN1, 0);
      motorPWM(PIN_MOTOR_M1_IN2, Motor_speed); 
      //Serial.print("Motor M1 1");
      
    }
    else if (dir == -1){
      motorPWM(PIN_MOTOR_M1_IN1, Motor_speed);
      motorPWM(PIN_MOTOR_M1_IN2, 0);
      //Serial.println("Motor M1 -1");
      }
    else {
      motorPWM(PIN_MOTOR_M1_IN1, 0);
      motorPWM(PIN_MOTOR_M1_IN2, 0);
      //Serial.println("Motor M1 STOP");
      }
  }
  else if (Motor_ID == 2){
    if (dir == 1){
      motorPWM(PIN_MOTOR_M2_IN1, Motor_speed);
      motorPWM(PIN_MOTOR_M2_IN2, 0);
      //Serial.println("Motor M2 1");
    }
    else if (dir == -1){
      motorPWM(PIN_MOTOR_M2_IN1, 0);
      motorPWM(PIN_MOTOR_M2_IN2, Motor_speed);
      //Serial.println("Motor M2 -1");
      }
    else {
      motorPWM(PIN_MOTOR_M2_IN1, 0);
      motorPWM(PIN_MOTOR_M2_IN2, 0);
      //Serial.println("Motor M2 STOP");
      }
  }
}

void Init_MotorVelocity(){
      Motor(1, 1, 0); // M1, forward rotation, fast rotation.
      Motor(2, 1, 0);  // M2, forward rotation,low rotation.
      currentVelocity = 0;
}