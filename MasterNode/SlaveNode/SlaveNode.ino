// ID  :  14번 
#include <Wire.h>

#define LED_PIN 12

const int buzzer = 5; // 부저가 연결된 핀 번호
int receivedValue = 0; // 마스터로부터 받은 데이터
int sendValue = 456;   // 마스터에게 보낼 데이터
bool buzzerFlag = false; // 부저를 울려야 하는지 확인하는 플래그

void setup() {

  pinMode(LED_BUILTIN, OUTPUT); 
  pinMode(LED_PIN, OUTPUT); 
  pinMode(buzzer, OUTPUT); // 부저 핀을 출력 모드로 설정

  Wire.begin(8); // 슬레이브 주소: 8
  Wire.onReceive(receiveEvent);  // 마스터가 데이터를 보낼 때 호출되는 함수
  Wire.onRequest(requestEvent); // 마스터가 데이터를 요청할 때 호출되는 함수
  Serial.begin(57600);
}

void loop() {
  
  delay(1000);
}


// 마스터로부터 데이터를 받을 때 호출
void receiveEvent(int howMany) {
    Serial.println("check");
    if (howMany >= sizeof(receivedValue)) { // 충분한 데이터가 전달된 경우
        for (int i = 0; i < sizeof(receivedValue); i++) {
            ((byte*)&receivedValue)[i] = Wire.read();
        }
        Serial.println("Data received:");
        Serial.println(receivedValue);

        if (receivedValue == 1) {
            digitalWrite(LED_BUILTIN, HIGH); // LED 켜기
        } else if (receivedValue == 2) {
            digitalWrite(LED_PIN, HIGH); // LED 켜기
        } else if (receivedValue == 3) {
            buzzerFlag = true;
        } else {
            Serial.println("Invalid data received!");
        }
    }
}

//마스터가 데이터를 요청할 때 호출
void requestEvent() {
  Wire.write((byte*)&sendValue, sizeof(sendValue)); // 정수 데이터 전송
}
