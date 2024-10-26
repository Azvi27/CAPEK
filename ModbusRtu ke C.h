#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

typedef struct {
    uint8_t u8id;
    uint8_t u8fct;
    uint16_t u16RegAdd;
    uint16_t u16CoilsNo;
    uint16_t *au16reg;
} modbus_t;

enum {
    RESPONSE_SIZE = 6,
    EXCEPTION_SIZE = 3,
    CHECKSUM_SIZE = 2
};

enum MESSAGE {
    ID = 0,
    FUNC,
    ADD_HI,
    ADD_LO,
    NB_HI,
    NB_LO,
    BYTE_CNT
};

enum MB_FC {
    MB_FC_NONE = 0,
    MB_FC_READ_COILS = 1,
    MB_FC_READ_DISCRETE_INPUT = 2,
    MB_FC_READ_REGISTERS = 3,
    MB_FC_READ_INPUT_REGISTER = 4,
    MB_FC_WRITE_COIL = 5,
    MB_FC_WRITE_REGISTER = 6,
    MB_FC_WRITE_MULTIPLE_COILS = 15,
    MB_FC_WRITE_MULTIPLE_REGISTERS = 16
};

enum COM_STATES {
    COM_IDLE = 0,
    COM_WAITING = 1
};

enum ERR_LIST {
    ERR_NOT_MASTER = -1,
    ERR_POLLING = -2,
    ERR_BUFF_OVERFLOW = -3,
    ERR_BAD_CRC = -4,
    ERR_EXCEPTION = -5
};

enum {
    NO_REPLY = 255,
    EXC_FUNC_CODE = 1,
    EXC_ADDR_RANGE = 2,
    EXC_REGS_QUANT = 3,
    EXC_EXECUTE = 4
};

static const unsigned char fctsupported[] = {
    MB_FC_READ_COILS,
    MB_FC_READ_DISCRETE_INPUT,
    MB_FC_READ_REGISTERS,
    MB_FC_READ_INPUT_REGISTER,
    MB_FC_WRITE_COIL,
    MB_FC_WRITE_REGISTER,
    MB_FC_WRITE_MULTIPLE_COILS,
    MB_FC_WRITE_MULTIPLE_REGISTERS
};

#define T35 5
#define MAX_BUFFER 64

typedef struct Modbus {
    FILE *port;
    uint8_t u8id;
    uint8_t u8txenpin;
    uint8_t u8state;
    uint8_t u8lastError;
    uint8_t au8Buffer[MAX_BUFFER];
    uint8_t u8BufferSize;
    uint8_t u8lastRec;
    uint16_t *au16regs;
    uint16_t u16InCnt, u16OutCnt, u16errCnt;
    uint16_t u16timeOut;
    uint32_t u32time, u32timeOut, u32overTime;
    uint8_t u8regsize;

    void (*sendTxBuffer)(struct Modbus*);
    int8_t (*getRxBuffer)(struct Modbus*);
    uint16_t (*calcCRC)(struct Modbus*, uint8_t u8length);
    uint8_t (*validateAnswer)(struct Modbus*);
    uint8_t (*validateRequest)(struct Modbus*);
    void (*get_FC1)(struct Modbus*);
    void (*get_FC3)(struct Modbus*);
    int8_t (*process_FC1)(struct Modbus*, uint16_t *regs, uint8_t u8size);
    int8_t (*process_FC3)(struct Modbus*, uint16_t *regs, uint8_t u8size);
    int8_t (*process_FC5)(struct Modbus*, uint16_t *regs, uint8_t u8size);
    int8_t (*process_FC6)(struct Modbus*, uint16_t *regs, uint8_t u8size);
    int8_t (*process_FC15)(struct Modbus*, uint16_t *regs, uint8_t u8size);
    int8_t (*process_FC16)(struct Modbus*, uint16_t *regs, uint8_t u8size);
    void (*buildException)(struct Modbus*, uint8_t u8exception);

    void (*start)(struct Modbus*);
    void (*setTimeOut)(struct Modbus*, uint16_t u16timeOut);
    uint16_t (*getTimeOut)(struct Modbus*);
    bool (*getTimeOutState)(struct Modbus*);
    int8_t (*query)(struct Modbus*, modbus_t telegram);
    int8_t (*poll)(struct Modbus*);
    int8_t (*pollWithRegs)(struct Modbus*, uint16_t *regs, uint8_t u8size);
    uint16_t (*getInCnt)(struct Modbus*);
    uint16_t (*getOutCnt)(struct Modbus*);
    uint16_t (*getErrCnt)(struct Modbus*);
    uint8_t (*getID)(struct Modbus*);
    uint8_t (*getState)(struct Modbus*);
    uint8_t (*getLastError)(struct Modbus*);
    void (*setID)(struct Modbus*, uint8_t u8id);
    void (*setTxendPinOverTime)(struct Modbus*, uint32_t u32overTime);
    void (*end)(struct Modbus*);
} Modbus;

Modbus* Modbus_new(uint8_t u8id, FILE* port, uint8_t u8txenpin);
void Modbus_delete(Modbus* modbus);

#endif // MODBUS_RTU_H

