#include <ModbusRTUSlave.h>

ModbusRTUSlave ModbusRTUSlave(&Serial, 1);  // dùng Serial với ID = 1

const int ledPin = 13;

void setup() {
  pinMode(ledPin, OUTPUT);
  ModbusRTUSlave.begin(9600);  // Baudrate RS485
  //ModbusRTUSlave.Write_4x(1, 123);  // Gán trước giá trị vào thanh ghi 4x[0]
}

void loop() {
  ModbusRTUSlave.poll();  // Lắng nghe và xử lý khung tin

  // Ghi 1 vào 0x[1] từ master để bật LED, ghi 0 để tắt
  if (ModbusRTUSlave.Read_bit_0x(1) == 1) {
    digitalWrite(ledPin, HIGH);
    ModbusRTUSlave.Set_ON_0x(2);
  } else {
    digitalWrite(ledPin, LOW);
    ModbusRTUSlave.Set_OFF_0x(2);
  }
  int a=ModbusRTUSlave.Read_4x(6);
  if(a == 10){
    digitalWrite(ledPin, HIGH);
    ModbusRTUSlave.Set_ON_0x(1);
  }

  // Cập nhật thời gian millis() vào thanh ghi 4x[1] và 4x[3] (dưới dạng float)
  float uptime_sec = millis() / 1000.0;
  ModbusRTUSlave.WriteFloat_4x(1, uptime_sec);
  ModbusRTUSlave.WriteFloat_4x(3, uptime_sec+2);
}
