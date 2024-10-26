#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

typedef struct {
    uint8_t u8id;
    uint8_t u8txenpin;
    uint16_t u16timeOut;
    uint32_t u32overTime;
    uint8_t u8state;
    uint8_t u8lastError;
    uint16_t u16InCnt;
    uint16_t u16OutCnt;
    uint16_t u16errCnt;
    uint8_t u8lastRec;
    uint8_t u8BufferSize;
    uint8_t au8Buffer[256];
    uint16_t au16regs[128];
    uint8_t u8regsize;
    uint32_t u32time;
    uint32_t u32timeOut;
    FILE* port;
} Modbus;

void Modbus_init(Modbus* modbus, uint8_t u8id, FILE* port, uint8_t u8txenpin) {
    modbus->port = port;
    modbus->u8id = u8id;
    modbus->u8txenpin = u8txenpin;
    modbus->u16timeOut = 1000;
    modbus->u32overTime = 0;
}

void Modbus_init_serial(Modbus* modbus, uint8_t u8id, uint8_t u8serno, uint8_t u8txenpin) {
    modbus->u8id = u8id;
    modbus->u8txenpin = u8txenpin;
    modbus->u16timeOut = 1000;
    modbus->u32overTime = 0;

    switch(u8serno) {
        case 1:
            modbus->port = fopen("/dev/ttyS1", "r+");
            break;
        case 2:
            modbus->port = fopen("/dev/ttyS2", "r+");
            break;
        case 3:
            modbus->port = fopen("/dev/ttyS3", "r+");
            break;
        case 0:
        default:
            modbus->port = fopen("/dev/ttyS0", "r+");
            break;
    }
}

void Modbus_start(Modbus* modbus) {
    if (modbus->u8txenpin > 1) {
        // pinMode(modbus->u8txenpin, OUTPUT);
        // digitalWrite(modbus->u8txenpin, LOW);
    }

    while(fgetc(modbus->port) != EOF);
    modbus->u8lastRec = modbus->u8BufferSize = 0;
    modbus->u16InCnt = modbus->u16OutCnt = modbus->u16errCnt = 0;
}

void Modbus_begin(Modbus* modbus, FILE* install_port, long u32speed) {
    modbus->port = install_port;
    // install_port->begin(u32speed);
    Modbus_start(modbus);
}

void Modbus_begin_with_txenpin(Modbus* modbus, FILE* install_port, long u32speed, uint8_t u8txenpin) {
    modbus->u8txenpin = u8txenpin;
    modbus->port = install_port;
    // install_port->begin(u32speed);
    Modbus_start(modbus);
}

void Modbus_begin_speed(Modbus* modbus, long u32speed) {
    // static_cast<HardwareSerial*>(modbus->port)->begin(u32speed);
    Modbus_start(modbus);
}

void Modbus_setID(Modbus* modbus, uint8_t u8id) {
    if ((u8id != 0) && (u8id <= 247)) {
        modbus->u8id = u8id;
    }
}

void Modbus_setTxendPinOverTime(Modbus* modbus, uint32_t u32overTime) {
    modbus->u32overTime = u32overTime;
}

uint8_t Modbus_getID(Modbus* modbus) {
    return modbus->u8id;
}

void Modbus_setTimeOut(Modbus* modbus, uint16_t u16timeOut) {
    modbus->u16timeOut = u16timeOut;
}

int Modbus_getTimeOutState(Modbus* modbus) {
    return ((unsigned long)(time(NULL) - modbus->u32timeOut) > (unsigned long)modbus->u16timeOut);
}

uint16_t Modbus_getInCnt(Modbus* modbus) {
    return modbus->u16InCnt;
}

uint16_t Modbus_getOutCnt(Modbus* modbus) {
    return modbus->u16OutCnt;
}

uint16_t Modbus_getErrCnt(Modbus* modbus) {
    return modbus->u16errCnt;
}

uint8_t Modbus_getState(Modbus* modbus) {
    return modbus->u8state;
}

uint8_t Modbus_getLastError(Modbus* modbus) {
    return modbus->u8lastError;
}

int8_t Modbus_query(Modbus* modbus, struct modbus_t telegram) {
    uint8_t u8regsno, u8bytesno;
    if (modbus->u8id != 0) return -2;
    if (modbus->u8state != COM_IDLE) return -1;

    if ((telegram.u8id == 0) || (telegram.u8id > 247)) return -3;

    memcpy(modbus->au16regs, telegram.au16reg, sizeof(telegram.au16reg));

    modbus->au8Buffer[ID] = telegram.u8id;
    modbus->au8Buffer[FUNC] = telegram.u8fct;
    modbus->au8Buffer[ADD_HI] = (telegram.u16RegAdd >> 8) & 0xFF;
    modbus->au8Buffer[ADD_LO] = telegram.u16RegAdd & 0xFF;

    switch(telegram.u8fct) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUT:
        case MB_FC_READ_REGISTERS:
        case MB_FC_READ_INPUT_REGISTER:
            modbus->au8Buffer[NB_HI] = (telegram.u16CoilsNo >> 8) & 0xFF;
            modbus->au8Buffer[NB_LO] = telegram.u16CoilsNo & 0xFF;
            modbus->u8BufferSize = 6;
            break;
        case MB_FC_WRITE_COIL:
            modbus->au8Buffer[NB_HI] = (modbus->au16regs[0] > 0) ? 0xff : 0;
            modbus->au8Buffer[NB_LO] = 0;
            modbus->u8BufferSize = 6;
            break;
        case MB_FC_WRITE_REGISTER:
            modbus->au8Buffer[NB_HI] = (modbus->au16regs[0] >> 8) & 0xFF;
            modbus->au8Buffer[NB_LO] = modbus->au16regs[0] & 0xFF;
            modbus->u8BufferSize = 6;
            break;
        case MB_FC_WRITE_MULTIPLE_COILS:
            u8regsno = telegram.u16CoilsNo / 16;
            u8bytesno = u8regsno * 2;
            if ((telegram.u16CoilsNo % 16) != 0) {
                u8bytesno++;
                u8regsno++;
            }

            modbus->au8Buffer[NB_HI] = (telegram.u16CoilsNo >> 8) & 0xFF;
            modbus->au8Buffer[NB_LO] = telegram.u16CoilsNo & 0xFF;
            modbus->au8Buffer[BYTE_CNT] = u8bytesno;
            modbus->u8BufferSize = 7;

            for (uint16_t i = 0; i < u8bytesno; i++) {
                if(i % 2) {
                    modbus->au8Buffer[modbus->u8BufferSize] = modbus->au16regs[i / 2] & 0xFF;
                } else {
                    modbus->au8Buffer[modbus->u8BufferSize] = (modbus->au16regs[i / 2] >> 8) & 0xFF;
                }
                modbus->u8BufferSize++;
            }
            break;

        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            modbus->au8Buffer[NB_HI] = (telegram.u16CoilsNo >> 8) & 0xFF;
            modbus->au8Buffer[NB_LO] = telegram.u16CoilsNo & 0xFF;
            modbus->au8Buffer[BYTE_CNT] = (uint8_t)(telegram.u16CoilsNo * 2);
            modbus->u8BufferSize = 7;

            for (uint16_t i = 0; i < telegram.u16CoilsNo; i++) {
                modbus->au8Buffer[modbus->u8BufferSize] = (modbus->au16regs[i] >> 8) & 0xFF;
                modbus->u8BufferSize++;
                modbus->au8Buffer[modbus->u8BufferSize] = modbus->au16regs[i] & 0xFF;
                modbus->u8BufferSize++;
            }
            break;
    }

    Modbus_sendTxBuffer(modbus);
    modbus->u8state = COM_WAITING;
    modbus->u8lastError = 0;
    return 0;
}

int8_t Modbus_poll(Modbus* modbus) {
    uint8_t u8current;
    u8current = fgetc(modbus->port) != EOF;

    if ((unsigned long)(time(NULL) - modbus->u32timeOut) > (unsigned long)modbus->u16timeOut) {
        modbus->u8state = COM_IDLE;
        modbus->u8lastError = NO_REPLY;
        modbus->u16errCnt++;
        return 0;
    }

    if (u8current == 0) return 0;

    if (u8current != modbus->u8lastRec) {
        modbus->u8lastRec = u8current;
        modbus->u32time = time(NULL);
        return 0;
    }
    if ((unsigned long)(time(NULL) - modbus->u32time) < (unsigned long)T35) return 0;

    modbus->u8lastRec = 0;
    int8_t i8state = Modbus_getRxBuffer(modbus);
    if (i8state < 6) {
        modbus->u8state = COM_IDLE;
        modbus->u16errCnt++;
        return i8state;
    }

    uint8_t u8exception = Modbus_validateAnswer(modbus);
    if (u8exception != 0) {
        modbus->u8state = COM_IDLE;
        return u8exception;
    }

    switch(modbus->au8Buffer[FUNC]) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUT:
            Modbus_get_FC1(modbus);
            break;
        case MB_FC_READ_INPUT_REGISTER:
        case MB_FC_READ_REGISTERS:
            Modbus_get_FC3(modbus);
            break;
        case MB_FC_WRITE_COIL:
        case MB_FC_WRITE_REGISTER:
        case MB_FC_WRITE_MULTIPLE_COILS:
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            break;
        default:
            break;
    }
    modbus->u8state = COM_IDLE;
    return modbus->u8BufferSize;
}

int8_t Modbus_poll_regs(Modbus* modbus, uint16_t* regs, uint8_t u8size) {
    memcpy(modbus->au16regs, regs, sizeof(uint16_t) * u8size);
    modbus->u8regsize = u8size;
    uint8_t u8current;

    u8current = fgetc(modbus->port) != EOF;

    if (u8current == 0) return 0;

    if (u8current != modbus->u8lastRec) {
        modbus->u8lastRec = u8current;
        modbus->u32time = time(NULL);
        return 0;
    }
    if ((unsigned long)(time(NULL) - modbus->u32time) < (unsigned long)T35) return 0;

    modbus->u8lastRec = 0;
    int8_t i8state = Modbus_getRxBuffer(modbus);
    modbus->u8lastError = i8state;
    if (i8state < 7) return i8state;

    if (modbus->au8Buffer[ID] != modbus->u8id) return 0;

    uint8_t u8exception = Modbus_validateRequest(modbus);
    if (u8exception > 0) {
        if (u8exception != NO_REPLY) {
            Modbus_buildException(modbus, u8exception);
            Modbus_sendTxBuffer(modbus);
        }
        modbus->u8lastError = u8exception;
        return u8exception;
    }

    modbus->u32timeOut = time(NULL);
    modbus->u8lastError = 0;

    switch(modbus->au8Buffer[FUNC]) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUT:
            return Modbus_process_FC1(modbus, regs, u8size);
            break;
        case MB_FC_READ_INPUT_REGISTER:
        case MB_FC_READ_REGISTERS:
            return Modbus_process_FC3(modbus, regs, u8size);
            break;
        case MB_FC_WRITE_COIL:
            return Modbus_process_FC5(modbus, regs, u8size);
            break;
        case MB_FC_WRITE_REGISTER:
            return Modbus_process_FC6(modbus, regs, u8size);
            break;
        case MB_FC_WRITE_MULTIPLE_COILS:
            return Modbus_process_FC15(modbus, regs, u8size);
            break;
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            return Modbus_process_FC16(modbus, regs, u8size);
            break;
        default:
            break;
    }
    return i8state;
}

int8_t Modbus_getRxBuffer(Modbus* modbus) {
    int bBuffOverflow = 0;

    if (modbus->u8txenpin > 1) {
        // digitalWrite(modbus->u8txenpin, LOW);
    }

    modbus->u8BufferSize = 0;
    while (fgetc(modbus->port) != EOF) {
        modbus->au8Buffer[modbus->u8BufferSize] = fgetc(modbus->port);
        modbus->u8BufferSize++;

        if (modbus->u8BufferSize >= MAX_BUFFER) bBuffOverflow = 1;
    }
    modbus->u16InCnt++;

    if (bBuffOverflow) {
        modbus->u16errCnt++;
        return ERR_BUFF_OVERFLOW;
    }
    return modbus->u8BufferSize;
}

void Modbus_sendTxBuffer(Modbus* modbus) {
    uint16_t u16crc = Modbus_calcCRC(modbus, modbus->u8BufferSize);
    modbus->au8Buffer[modbus->u8BufferSize] = (u16crc >> 8) & 0xFF;
    modbus->u8BufferSize++;
    modbus->au8Buffer[modbus->u8BufferSize] = u16crc & 0xFF;
    modbus->u8BufferSize++;

    if (modbus->u8txenpin > 1) {
        // digitalWrite(modbus->u8txenpin, HIGH);
    }

    fwrite(modbus->au8Buffer, 1, modbus->u8BufferSize, modbus->port);

    if (modbus->u8txenpin > 1) {
        fflush(modbus->port);
        volatile uint32_t u32overTimeCountDown = modbus->u32overTime;
        while (u32overTimeCountDown-- > 0);
        // digitalWrite(modbus->u8txenpin, LOW);
    }
    while(fgetc(modbus->port) != EOF);

    modbus->u8BufferSize = 0;

    modbus->u32timeOut = time(NULL);

    modbus->u16OutCnt++;
}

uint16_t Modbus_calcCRC(Modbus* modbus, uint8_t u8length) {
    unsigned int temp, temp2, flag;
    temp = 0xFFFF;
    for (unsigned char i = 0; i < u8length; i++) {
        temp = temp ^ modbus->au8Buffer[i];
        for (unsigned char j = 1; j <= 8; j++) {
            flag = temp & 0x0001;
            temp >>= 1;
            if (flag)
                temp ^= 0xA001;
        }
    }
    temp2 = temp >> 8;
    temp = (temp << 8) | temp2;
    temp &= 0xFFFF;
    return temp;
}

uint8_t Modbus_validateRequest(Modbus* modbus) {
    uint16_t u16MsgCRC = ((modbus->au8Buffer[modbus->u8BufferSize - 2] << 8) | modbus->au8Buffer[modbus->u8BufferSize - 1]);
    if (Modbus_calcCRC(modbus, modbus->u8BufferSize - 2) != u16MsgCRC) {
        modbus->u16errCnt++;
        return NO_REPLY;
    }

    int isSupported = 0;
    for (uint8_t i = 0; i < sizeof(fctsupported); i++) {
        if (fctsupported[i] == modbus->au8Buffer[FUNC]) {
            isSupported = 1;
            break;
        }
    }
    if (!isSupported) {
        modbus->u16errCnt++;
        return EXC_FUNC_CODE;
    }

    uint16_t u16regs = 0;
    uint8_t u8regs;
    switch (modbus->au8Buffer[FUNC]) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUT:
        case MB_FC_WRITE_MULTIPLE_COILS:
            u16regs = (modbus->au8Buffer[ADD_HI] << 8 | modbus->au8Buffer[ADD_LO]) / 16;
            u16regs += (modbus->au8Buffer[NB_HI] << 8 | modbus->au8Buffer[NB_LO]) / 16;
            u8regs = (uint8_t)u16regs;
            if (u8regs > modbus->u8regsize) return EXC_ADDR_RANGE;
            break;
        case MB_FC_WRITE_COIL:
            u16regs = (modbus->au8Buffer[ADD_HI] << 8 | modbus->au8Buffer[ADD_LO]) / 16;
            u8regs = (uint8_t)u16regs;
            if (u8regs > modbus->u8regsize) return EXC_ADDR_RANGE;
            break;
        case MB_FC_WRITE_REGISTER:
            u16regs = (modbus->au8Buffer[ADD_HI] << 8 | modbus->au8Buffer[ADD_LO]);
            u8regs = (uint8_t)u16regs;
            if (u8regs > modbus->u8regsize) return EXC_ADDR_RANGE;
            break;
        case MB_FC_READ_REGISTERS:
        case MB_FC_READ_INPUT_REGISTER:
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            u16regs = (modbus->au8Buffer[ADD_HI] << 8 | modbus->au8Buffer[ADD_LO]);
            u16regs += (modbus->au8Buffer[NB_HI] << 8 | modbus->au8Buffer[NB_LO]);
            u8regs = (uint8_t)u16regs;
            if (u8regs > modbus->u8regsize) return EXC_ADDR_RANGE;
            break;
    }
    return 0;
}

uint8_t Modbus_validateAnswer(Modbus* modbus) {
    uint16_t u16MsgCRC = ((modbus->au8Buffer[modbus->u8BufferSize - 2] << 8) | modbus->au8Buffer[modbus->u8BufferSize - 1]);
    if (Modbus_calcCRC(modbus, modbus->u8BufferSize - 2) != u16MsgCRC) {
        modbus->u16errCnt++;
        return NO_REPLY;
    }

    if ((modbus->au8Buffer[FUNC] & 0x80) != 0) {
        modbus->u16errCnt++;
        return ERR_EXCEPTION;
    }

    int isSupported = 0;
    for (uint8_t i = 0; i < sizeof(fctsupported); i++) {
        if (fctsupported[i] == modbus->au8Buffer[FUNC]) {
            isSupported = 1;
            break;
        }
    }
    if (!isSupported) {
        modbus->u16errCnt++;
        return EXC_FUNC_CODE;
    }

    return 0;
}

void Modbus_buildException(Modbus* modbus, uint8_t u8exception) {
    uint8_t u8func = modbus->au8Buffer[FUNC];

    modbus->au8Buffer[ID] = modbus->u8id;
    modbus->au8Buffer[FUNC] = u8func + 0x80;
    modbus->au8Buffer[2] = u8exception;
    modbus->u8BufferSize = EXCEPTION_SIZE;
}

void Modbus_get_FC1(Modbus* modbus) {
    uint8_t u8byte, i;
    u8byte = 3;
    for (i = 0; i < modbus->au8Buffer[2]; i++) {
        if (i % 2) {
            modbus->au16regs[i / 2] = (modbus->au8Buffer[i + u8byte] << 8) | (modbus->au16regs[i / 2] & 0xFF);
        } else {
            modbus->au16regs[i / 2] = (modbus->au16regs[i / 2] & 0xFF00) | modbus->au8Buffer[i + u8byte];
        }
    }
}

void Modbus_get_FC3(Modbus* modbus) {
    uint8_t u8byte, i;
    u8byte = 3;

    for (i = 0; i < modbus->au8Buffer[2] / 2; i++) {
        modbus->au16regs[i] = (modbus->au8Buffer[u8byte] << 8) | modbus->au8Buffer[u8byte + 1];
        u8byte += 2;
    }
}

int8_t Modbus_process_FC1(Modbus* modbus, uint16_t* regs, uint8_t u8size) {
    uint8_t u8currentRegister, u8currentBit, u8bytesno, u8bitsno;
    uint8_t u8CopyBufferSize;
    uint16_t u16currentCoil, u16coil;

    uint16_t u16StartCoil = (modbus->au8Buffer[ADD_HI] << 8) | modbus->au8Buffer[ADD_LO];
    uint16_t u16Coilno = (modbus->au8Buffer[NB_HI] << 8) | modbus->au8Buffer[NB_LO];

    u8bytesno = (uint8_t)(u16Coilno / 8);
    if (u16Coilno % 8 != 0) u8bytesno++;
    modbus->au8Buffer[ADD_HI] = u8bytesno;
    modbus->u8BufferSize = ADD_LO;
    modbus->au8Buffer[modbus->u8BufferSize + u8bytesno - 1] = 0;

    u8bitsno = 0;

    for (u16currentCoil = 0; u16currentCoil < u16Coilno; u16currentCoil++) {
        u16coil = u16StartCoil + u16currentCoil;
        u8currentRegister = (uint8_t)(u16coil / 16);
        u8currentBit = (uint8_t)(u16coil % 16);

        modbus->au8Buffer[modbus->u8BufferSize] |= (bitRead(regs[u8currentRegister], u8currentBit) << u8bitsno);
        u8bitsno++;

        if (u8bitsno > 7) {
            u8bitsno = 0;
            modbus->u8BufferSize++;
        }
    }

    if (u16Coilno % 8 != 0) modbus->u8BufferSize++;
    u8CopyBufferSize = modbus->u8BufferSize + 2;
    Modbus_sendTxBuffer(modbus);
    return u8CopyBufferSize;
}

int8_t Modbus_process_FC3(Modbus* modbus, uint16_t* regs, uint8_t u8size) {
    uint8_t u8StartAdd = (modbus->au8Buffer[ADD_HI] << 8) | modbus->au8Buffer[ADD_LO];
    uint8_t u8regsno = (modbus->au8Buffer[NB_HI] << 8) | modbus->au8Buffer[NB_LO];
    uint8_t u8CopyBufferSize;
    uint8_t i;

    modbus->au8Buffer[2] = u8regsno * 2;
    modbus->u8BufferSize = 3;

    for (i = u8StartAdd; i < u8StartAdd + u8regsno; i++) {
        modbus->au8Buffer[modbus->u8BufferSize] = (regs[i] >> 8) & 0xFF;
        modbus->u8BufferSize++;
        modbus->au8Buffer[modbus->u8BufferSize] = regs[i] & 0xFF;
        modbus->u8BufferSize++;
    }
    u8CopyBufferSize = modbus->u8BufferSize + 2;
    Modbus_sendTxBuffer(modbus);

    return u8CopyBufferSize;
}

int8_t Modbus_process_FC5(Modbus* modbus, uint16_t* regs, uint8_t u8size) {
    uint8_t u8currentRegister, u8currentBit;
    uint8_t u8CopyBufferSize;
    uint16_t u16coil = (modbus->au8Buffer[ADD_HI] << 8) | modbus->au8Buffer[ADD_LO];
    u8currentRegister = (uint8_t)(u16coil / 16);
    u8currentBit = (uint8_t)(u16coil % 16);

    bitWrite(regs[u8currentRegister], u8currentBit, (modbus->au8Buffer[NB_HI] == 0xff));

    modbus->u8BufferSize = 6;
    u8CopyBufferSize = modbus->u8BufferSize + 2;
    Modbus_sendTxBuffer(modbus);

    return u8CopyBufferSize;
}

int8_t Modbus_process_FC6(Modbus* modbus, uint16_t* regs, uint8_t u8size) {
    uint8_t u8add = (modbus->au8Buffer[ADD_HI] << 8) | modbus->au8Buffer[ADD_LO];
    uint8_t u8CopyBufferSize;
    uint16_t u16val = (modbus->au8Buffer[NB_HI] << 8) | modbus->au8Buffer[NB_LO];

    regs[u8add] = u16val;

    modbus->u8BufferSize = RESPONSE_SIZE;

    u8CopyBufferSize = modbus->u8BufferSize + 2;
    Modbus_sendTxBuffer(modbus);

    return u8CopyBufferSize;
}

int8_t Modbus_process_FC15(Modbus* modbus, uint16_t* regs, uint8_t u8size) {
    uint8_t u8currentRegister, u8currentBit, u8frameByte, u8bitsno;
    uint8_t u8CopyBufferSize;
    uint16_t u16currentCoil, u16coil;
    uint8_t bTemp;

    uint16_t u16StartCoil = (modbus->au8Buffer[ADD_HI] << 8) | modbus->au8Buffer[ADD_LO];
    uint16_t u16Coilno = (modbus->au8Buffer[NB_HI] << 8) | modbus->au8Buffer[NB_LO];

    u8bitsno = 0;
    u8frameByte = 7;
    for (u16currentCoil = 0; u16currentCoil < u16Coilno; u16currentCoil++) {
        u16coil = u16StartCoil + u16currentCoil;
        u8currentRegister = (uint8_t)(u16coil / 16);
        u8currentBit = (uint8_t)(u16coil % 16);

        bTemp = bitRead(modbus->au8Buffer[u8frameByte], u8bitsno);

        bitWrite(regs[u8currentRegister], u8currentBit, bTemp);

        u8bitsno++;

        if (u8bitsno > 7) {
            u8bitsno = 0;
            u8frameByte++;
        }
    }

    modbus->u8BufferSize = 6;
    u8CopyBufferSize = modbus->u8BufferSize + 2;
    Modbus_sendTxBuffer(modbus);
    return u8CopyBufferSize;
}

int8_t Modbus_process_FC16(Modbus* modbus, uint16_t* regs, uint8_t u8size) {
    uint8_t u8StartAdd = (modbus->au8Buffer[ADD_HI] << 8) | modbus->au8Buffer[ADD_LO];
    uint8_t u8regsno = (modbus->au8Buffer[NB_HI] << 8) | modbus->au8Buffer[NB_LO];
    uint8_t u8CopyBufferSize;
    uint8_t i;
    uint16_t temp;

    modbus->au8Buffer[NB_HI] = 0;
    modbus->au8Buffer[NB_LO] = u8regsno;
    modbus->u8BufferSize = RESPONSE_SIZE;

    for (i = 0; i < u8regsno; i++) {
        temp = (modbus->au8Buffer[(BYTE_CNT + 1) + i * 2] << 8) | modbus->au8Buffer[(BYTE_CNT + 2) + i * 2];
        regs[u8StartAdd + i] = temp;
    }
    u8CopyBufferSize = modbus->u8BufferSize + 2;
    Modbus_sendTxBuffer(modbus);

    return u8CopyBufferSize;
}

