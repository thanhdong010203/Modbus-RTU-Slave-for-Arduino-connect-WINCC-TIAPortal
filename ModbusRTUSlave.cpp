#include "HardwareSerial.h"
#include "ModbusRTUSlave.h"


ModbusRTUSlave::ModbusRTUSlave(HardwareSerial* serialPort, unsigned char id) {
    _serial = serialPort;
    slaveID = id;
}

void ModbusRTUSlave::begin(long baud_rate) {
    _serial->begin(baud_rate);
    _serial->setTimeout(100);
}




void ModbusRTUSlave::Write_bit_0x(uint16_t Address, bool Value) {
 Address --;
  int addByte = Address / 8;
  uint16_t bitPosition = Address % 8;
  if (Value == 1) {
    Coils_0x[addByte] |= 1 << bitPosition;  // Replace that bit with 1
  } else {
    Coils_0x[addByte] &= ~(1 << bitPosition);  // Replace that bit with 0
  }
}

bool ModbusRTUSlave::Read_bit_0x(uint16_t Address) {
  Address --;
  if (Address > (BUFFER_SIZE * 8 - 1)) return false;
  int addByte = Address / 8;
  uint16_t bitPosition = Address % 8;
  return ((Coils_0x[addByte] >> bitPosition) & 0x01);
}

void ModbusRTUSlave::Write_bit_1x(uint16_t Address, bool Value) {
  Address --;
  int addByte = Address / 8;
  uint16_t bitPosition = Address % 8;
  if (Value == 1) {
    Inputs_1x[addByte] |= 1 << bitPosition;
  } else {
    Inputs_1x[addByte] &= ~(1 << bitPosition);
  }
}

bool ModbusRTUSlave::Read_bit_1x(uint16_t Address) {
  Address --;
  if (Address > (BUFFER_SIZE * 8 - 1)) return false;
  int addByte = Address / 8;
  uint16_t bitPosition = Address % 8;
  return ((Inputs_1x[addByte] >> bitPosition) & 0x01);
}

void ModbusRTUSlave::Write_4x(uint16_t add, uint16_t value) {
  add--;
  Holding_Registers_4x[add] = value;
}

void ModbusRTUSlave::WriteFloat_4x(uint16_t add, float value) {
  add--;
  uint32_t temp = *((uint32_t*)(&value));
  Holding_Registers_4x[add] = temp & 0xFFFF;
  Holding_Registers_4x[add + 1] = (temp >> 16) & 0xFFFF;
}

uint16_t ModbusRTUSlave::Read_4x(uint16_t add) {
  add--;
  return Holding_Registers_4x[add];
}

float ModbusRTUSlave::ReadFloat_4x(uint16_t add) {
  add--;
  uint32_t temp = ((uint32_t)Holding_Registers_4x[add + 1] << 16) | Holding_Registers_4x[add];
  return *((float*)(&temp));
}

void ModbusRTUSlave::Write_3x(uint16_t add, uint16_t value) {
  add--;
  Input_Registers_3x[add] = value;
}

uint16_t ModbusRTUSlave::Read_3x(uint16_t add) {
  add--;
  return Input_Registers_3x[add];
}

void ModbusRTUSlave::WriteFloat_3x(uint16_t add, float value) {
  add--;
  uint32_t temp = *((uint32_t*)(&value));
  Input_Registers_3x[add] = temp & 0xFFFF;
  Input_Registers_3x[add + 1] = (temp >> 16) & 0xFFFF;
}

float ModbusRTUSlave::ReadFloat_3x(uint16_t add) {
  add--;
  uint32_t temp = ((uint32_t)Input_Registers_3x[add + 1] << 16) | Input_Registers_3x[add];
  return *((float*)(&temp));
}








void ModbusRTUSlave::poll() {
  RxLength = _serial->readBytes(RxData, BUFFER_SIZE);
  if (RxLength > 0) {
    uint16_t crc_check = CRC16(RxData, RxLength - 2);
    uint16_t crc_Receive = (RxData[RxLength - 2] << 8) | RxData[RxLength - 1];
    if ((RxData[0] == slaveID) && (crc_check == crc_Receive)) {
      Process_data();
    } else if ((RxData[0] == slaveID) && (crc_check != crc_Receive)) {
      modbusException(ILLEGAL_FUNCTION);
    }
  }
}

void ModbusRTUSlave::modbusException(uint8_t exceptioncode) {
  TxData[0] = slaveID;
  TxData[1] = RxData[1] | 0x80;  // MSB = 1
  TxData[2] = exceptioncode;
  sendData(TxData, 3);
}

void ModbusRTUSlave::Process_data() {
  switch (RxData[1]) {
    case 0x03: readHoldingRegs(); break;
    case 0x04: readInputRegs(); break;
    case 0x01: readCoils(); break;
    case 0x02: readInputs(); break;
    case 0x06: writeSingleReg(); break;
    case 0x10: writeHoldingRegs(); break;
    case 0x05: writeSingleCoil(); break;
    case 0x0F: writeMultiCoils(); break;
    default: modbusException(ILLEGAL_FUNCTION); break;
  }
}

void ModbusRTUSlave::sendData(uint8_t *DataSend, uint8_t DataLen) {
  uint16_t crc = CRC16(DataSend, DataLen);
  DataSend[DataLen++] = highByte(crc);
  DataSend[DataLen++] = lowByte(crc);
  for (int i = 0; i < DataLen; i++) {
    Serial.write(DataSend[i]);
  }
}





uint8_t ModbusRTUSlave::readHoldingRegs() {
  uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);
  uint16_t numRegs = ((RxData[4] << 8) | RxData[5]);

  if ((numRegs < 1) || (numRegs > 125)) {
    modbusException(ILLEGAL_DATA_VALUE);
    return 0;
  }

  if (startAddr + numRegs - 1 > BUFFER_SIZE - 1) {
    modbusException(ILLEGAL_DATA_ADDRESS);
    return 0;
  }

  TxData[0] = slaveID;
  TxData[1] = RxData[1];
  TxData[2] = numRegs * 2;
  int indx = 3;

  for (int i = 0; i < numRegs; i++) {
    TxData[indx++] = (Holding_Registers_4x[startAddr] >> 8) & 0xFF;
    TxData[indx++] = Holding_Registers_4x[startAddr] & 0xFF;
    startAddr++;
  }

  sendData(TxData, indx);
  return 1;
}

uint8_t ModbusRTUSlave::readInputRegs() {
  uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);
  uint16_t numRegs = ((RxData[4] << 8) | RxData[5]);

  if ((numRegs < 1) || (numRegs > 125)) {
    modbusException(ILLEGAL_DATA_VALUE);
    return 0;
  }

  if (startAddr + numRegs - 1 > BUFFER_SIZE - 1) {
    modbusException(ILLEGAL_DATA_ADDRESS);
    return 0;
  }

  TxData[0] = slaveID;
  TxData[1] = RxData[1];
  TxData[2] = numRegs * 2;
  int indx = 3;

  for (int i = 0; i < numRegs; i++) {
    TxData[indx++] = (Input_Registers_3x[startAddr] >> 8) & 0xFF;
    TxData[indx++] = Input_Registers_3x[startAddr] & 0xFF;
    startAddr++;
  }

  sendData(TxData, indx);
  return 1;
}

uint8_t ModbusRTUSlave::readCoils() {
  uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);
  uint16_t numCoils = ((RxData[4] << 8) | RxData[5]);

  if ((numCoils < 1) || (numCoils > 2000)) {
    modbusException(ILLEGAL_DATA_VALUE);
    return 0;
  }

  if (startAddr + numCoils - 1 > BUFFER_SIZE * 8 - 1) {
    modbusException(ILLEGAL_DATA_ADDRESS);
    return 0;
  }

  memset(TxData, 0, BUFFER_SIZE);
  TxData[0] = slaveID;
  TxData[1] = RxData[1];
  TxData[2] = (numCoils / 8) + ((numCoils % 8) > 0 ? 1 : 0);
  int indx = 3;
  int startByte = startAddr / 8;
  uint16_t bitPosition = startAddr % 8;
  int indxPosition = 0;

  for (int i = 0; i < numCoils; i++) {
    TxData[indx] |= ((Coils_0x[startByte] >> bitPosition) & 0x01) << indxPosition;
    indxPosition++;
    bitPosition++;
    if (indxPosition > 7) {
      indxPosition = 0;
      indx++;
    }
    if (bitPosition > 7) {
      bitPosition = 0;
      startByte++;
    }
  }

  if (numCoils % 8 != 0) indx++;
  sendData(TxData, indx);
  return 1;
}

uint8_t ModbusRTUSlave::readInputs() {
  uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);
  uint16_t numCoils = ((RxData[4] << 8) | RxData[5]);

  if ((numCoils < 1) || (numCoils > 2000)) {
    modbusException(ILLEGAL_DATA_VALUE);
    return 0;
  }

  if (startAddr + numCoils - 1 > BUFFER_SIZE * 8 - 1) {
    modbusException(ILLEGAL_DATA_ADDRESS);
    return 0;
  }

  memset(TxData, 0, BUFFER_SIZE);
  TxData[0] = slaveID;
  TxData[1] = RxData[1];
  TxData[2] = (numCoils / 8) + ((numCoils % 8) > 0 ? 1 : 0);
  int indx = 3;
  int startByte = startAddr / 8;
  uint16_t bitPosition = startAddr % 8;
  int indxPosition = 0;

  for (int i = 0; i < numCoils; i++) {
    TxData[indx] |= ((Inputs_1x[startByte] >> bitPosition) & 0x01) << indxPosition;
    indxPosition++;
    bitPosition++;
    if (indxPosition > 7) {
      indxPosition = 0;
      indx++;
    }
    if (bitPosition > 7) {
      bitPosition = 0;
      startByte++;
    }
  }

  if (numCoils % 8 != 0) indx++;
  sendData(TxData, indx);
  return 1;
}








uint8_t ModbusRTUSlave::writeHoldingRegs() {
  uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);
  uint16_t numRegs = ((RxData[4] << 8) | RxData[5]);

  if ((numRegs < 1) || (numRegs > 123)) {
    modbusException(ILLEGAL_DATA_VALUE);
    return 0;
  }

  if (startAddr + numRegs - 1 > BUFFER_SIZE - 1) {
    modbusException(ILLEGAL_DATA_ADDRESS);
    return 0;
  }

  uint8_t indx = 7;
  for (int i = 0; i < numRegs; i++) {
    Holding_Registers_4x[startAddr] = (RxData[indx] << 8) | RxData[indx + 1];
    startAddr++;
    indx += 2;
  }

  TxData[0] = slaveID;
  TxData[1] = RxData[1];
  TxData[2] = RxData[2];
  TxData[3] = RxData[3];
  TxData[4] = RxData[4];
  TxData[5] = RxData[5];

  sendData(TxData, 6);
  return 1;
}

uint8_t ModbusRTUSlave::writeSingleReg() {
  uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);

  if (startAddr > BUFFER_SIZE - 1) {
    modbusException(ILLEGAL_DATA_ADDRESS);
    return 0;
  }

  Holding_Registers_4x[startAddr] = (RxData[4] << 8) | RxData[5];

  TxData[0] = slaveID;
  TxData[1] = RxData[1];
  TxData[2] = RxData[2];
  TxData[3] = RxData[3];
  TxData[4] = RxData[4];
  TxData[5] = RxData[5];

  sendData(TxData, 6);
  return 1;
}

uint8_t ModbusRTUSlave::writeSingleCoil() {
  uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);

  if (startAddr > BUFFER_SIZE * 8 - 1) {
    modbusException(ILLEGAL_DATA_ADDRESS);
    return 0;
  }

  int startByte = startAddr / 8;
  uint16_t bitPosition = startAddr % 8;

  if ((RxData[4] == 0xFF) && (RxData[5] == 0x00)) {
    Coils_0x[startByte] |= 1 << bitPosition;
  } else if ((RxData[4] == 0x00) && (RxData[5] == 0x00)) {
    Coils_0x[startByte] &= ~(1 << bitPosition);
  }

  TxData[0] = slaveID;
  TxData[1] = RxData[1];
  TxData[2] = RxData[2];
  TxData[3] = RxData[3];
  TxData[4] = RxData[4];
  TxData[5] = RxData[5];

  sendData(TxData, 6);
  return 1;
}

uint8_t ModbusRTUSlave::writeMultiCoils() {
  uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);
  uint16_t numCoils = ((RxData[4] << 8) | RxData[5]);

  if ((numCoils < 1) || (numCoils > 1968)) {
    modbusException(ILLEGAL_DATA_VALUE);
    return 0;
  }

  if (startAddr + numCoils - 1 > BUFFER_SIZE * 8 - 1) {
    modbusException(ILLEGAL_DATA_ADDRESS);
    return 0;
  }

  int startByte = startAddr / 8;
  uint16_t bitPosition = startAddr % 8;
  int indxPosition = 0;
  int indx = 7;

  for (int i = 0; i < numCoils; i++) {
    if ((RxData[indx] >> indxPosition) & 0x01) {
      Coils_0x[startByte] |= 1 << bitPosition;
    } else {
      Coils_0x[startByte] &= ~(1 << bitPosition);
    }

    bitPosition++;
    indxPosition++;

    if (indxPosition > 7) {
      indxPosition = 0;
      indx++;
    }

    if (bitPosition > 7) {
      bitPosition = 0;
      startByte++;
    }
  }

  TxData[0] = slaveID;
  TxData[1] = RxData[1];
  TxData[2] = RxData[2];
  TxData[3] = RxData[3];
  TxData[4] = RxData[4];
  TxData[5] = RxData[5];

  sendData(TxData, 6);
  return 1;
}









/* Table of CRC values for high–order byte */
static unsigned char auchCRCHi[] = {
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
  0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
  0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
  0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
  0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
  0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
  0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
  0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
  0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
  0x40
};

//unsigned uIndex ; /* will index into CRC lookup table */
static char auchCRCLo[] = {
  0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
  0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
  0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
  0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
  0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
  0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
  0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
  0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
  0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
  0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
  0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
  0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
  0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
  0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
  0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
  0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
  0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
  0x40
};

uint16_t ModbusRTUSlave::CRC16(uint8_t *puchMsg, uint8_t usDataLen) {
  unsigned char uchCRCHi = 0xFF; /* high byte of CRC initialized */
  unsigned char uchCRCLo = 0xFF; /* low byte of CRC initialized */
  unsigned uIndex;               /* will index into CRC lookup table */
  //while (usDataLen––) /* pass through message buffer */
  while (usDataLen--) {
    uIndex = uchCRCHi ^ *puchMsg++; /* calculate the CRC */
    uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
    uchCRCLo = auchCRCLo[uIndex];
  }
  return (uchCRCHi << 8 | uchCRCLo);
}

void ModbusRTUSlave::delaysecond(uint8_t time_) {
  while (time_-- > 0) {
    delay(245);
    poll();
    delay(245);
    poll();
    delay(245);
    poll();
    delay(245);
    poll();
  }
}
