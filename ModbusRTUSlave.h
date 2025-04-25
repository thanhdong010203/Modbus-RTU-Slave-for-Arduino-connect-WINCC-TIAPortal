#ifndef MODBUS_RTU_SLAVE_H
#define MODBUS_RTU_SLAVE_H

#include <Arduino.h>

/*
 *  This library for Modus RTU Slave TIAPortal WINCC RT Advance
 *  Created By: Thanh-Dong Ngo
 *  Created On: April 25, 2025
 *  My Channel: https://www.youtube.com/playlist?list=PLBXqXrr7xpVi0ICZPO7qnjRXk_-_tFEWn
 */
/*
Vùng | Ký hiệu | Function Code | Loại dữ liệu | Địa chỉ bắt đầu | Ghi chú
0x | Coils | FC 01 (Read)  FC 05, 15 (Write) | Digital Output (DO) | 00001 → 09999 | Có thể đọc/ghi. Mỗi coil = 1 bit (ON/OFF)
1x | Discrete Inputs | FC 02 (Read) | Digital Input (DI) | 10001 → 19999 | Chỉ đọc. Mỗi input = 1 bit
3x | Input Registers | FC 04 (Read) | Analog Input (AI) | 30001 → 39999 | Chỉ đọc. Mỗi input = 16-bit
4x | Holding Registers | FC 03 (Read)  FC 06, 16 (Write) | Analog Output / Memory | 40001 → 49999 | Có thể đọc/ghi. Mỗi register = 16-bit
*/
#ifndef BUFFER_SIZE
#define BUFFER_SIZE 64
#endif

#ifndef SLAVE_ID
#define SLAVE_ID 1
#endif

#define ILLEGAL_FUNCTION 0x01
#define ILLEGAL_DATA_ADDRESS 0x02
#define ILLEGAL_DATA_VALUE 0x03

#define float2uint32(...) *((uint32_t*)(&__VA_ARGS__))
#define uint322float(...) *((float *)(&__VA_ARGS__))

#define Set_ON_0x(Address)  Write_bit_0x(Address, 1)
#define Set_OFF_0x(Address) Write_bit_0x(Address, 0)
#define Set_ON_1x(Address)  Write_bit_1x(Address, 1)
#define Set_OFF_1x(Address) Write_bit_1x(Address, 0)

class ModbusRTUSlave {
public:
  
ModbusRTUSlave(HardwareSerial* serialPort, unsigned char id);
    void begin(long baud_rate);

    void poll();
    void delaysecond(uint8_t time_);

    void Write_bit_0x(uint16_t Address, bool Value);
    bool Read_bit_0x(uint16_t Address);
    void Write_bit_1x(uint16_t Address, bool Value);
    bool Read_bit_1x(uint16_t Address);

    void Write_4x(uint16_t add, uint16_t value);
    uint16_t Read_4x(uint16_t add);
    void WriteFloat_4x(uint16_t add, float value);
    float ReadFloat_4x(uint16_t add);

    void Write_3x(uint16_t add, uint16_t value);
    uint16_t Read_3x(uint16_t add);
    void WriteFloat_3x(uint16_t add, float value);
    float ReadFloat_3x(uint16_t add);


  byte TxData[BUFFER_SIZE];
    byte RxData[BUFFER_SIZE];
    uint8_t RxLength;

     uint8_t Coils_0x[BUFFER_SIZE];
     uint8_t Inputs_1x[BUFFER_SIZE];
     uint16_t Holding_Registers_4x[BUFFER_SIZE];
     uint16_t Input_Registers_3x[BUFFER_SIZE];

    uint8_t slaveID;

private:
    void Process_data();
    void modbusException(uint8_t exceptioncode);
    void sendData(uint8_t *DataSend, uint8_t DataLen);
    uint16_t CRC16(uint8_t *, uint8_t);

    uint8_t readHoldingRegs(void);
    uint8_t readInputRegs(void);
    uint8_t readCoils(void);
    uint8_t readInputs(void);
    uint8_t writeSingleReg(void);
    uint8_t writeHoldingRegs(void);
    uint8_t writeSingleCoil(void);
    uint8_t writeMultiCoils(void);


HardwareSerial* _serial;    // con trỏ đến Serial (Serial, Serial1, ...)
};

#endif // MODBUS_RTU_SLAVE_H
