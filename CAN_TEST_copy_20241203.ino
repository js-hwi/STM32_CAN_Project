#include <SPI.h>
#include <mcp2515.h>
#include "Nunchuk.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//----------------------------------------------------------------------------------
int mappedValue = 0;

struct can_frame canMsg1;
MCP2515 mcp2515(4);

void setup() {
  Serial.begin(115200); // 시리얼 통신 시작
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  
  Serial.println("MCP2515 Initialized");
}

void loop() {
  // CAN 메시지 구성
  canMsg1.can_id  = 0xF6;  // 메시지 ID
  canMsg1.can_dlc = 8;       // 데이터 길이
  canMsg1.data[0] = 0x8E;
  canMsg1.data[1] = 0x87;
  canMsg1.data[2] = 0x32;
  canMsg1.data[3] = 0xFA;
  canMsg1.data[4] = 0x26;
  canMsg1.data[5] = 0x8E;
  canMsg1.data[6] = 0xBE;
  canMsg1.data[7] = 0x86;

  // 메시지 전송
  if (mcp2515.sendMessage(&canMsg1) == MCP2515::ERROR_OK) {
    Serial.println("CAN message sent successfully!");
  } else {
    Serial.println("Error: Failed to send CAN message!");
  }

  if(canMsg1.can_id == 0xAAA){
     Serial.println("get CanMsg");
  }

  delay(1000); // 1초 대기
}
