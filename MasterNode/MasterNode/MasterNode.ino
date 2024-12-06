#include <SPI.h>
#include <Wire.h>
#include <mcp2515.h>

int sendValue = 2; // 슬레이브로 보낼 데이터
struct can_frame canMsg2;
MCP2515 mcp2515(4);


void setup() {
    Wire.begin(); // I2C 마스터 시작
    Serial.begin(9600);

    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
}

void loop() {
    mcp2515.readMessage(&canMsg2); // 수신 요청
    if (canMsg2.can_id == 0x77) {
        Serial.println("CAN message received.");
        Wire.beginTransmission(8); // 슬레이브 주소: 8
        Wire.write((byte*)&sendValue, sizeof(sendValue)); // 정수 데이터 전송
        Wire.endTransmission();
        Serial.print("Sent to Slave: ");
        Serial.println(sendValue);
        delay(10); // 슬레이브 처리 대기
    }
    delay(1000); // 1초 간격
}
