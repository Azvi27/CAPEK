#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <inttypes.h>
#include "Arduino.h"

typedef struct
{
    uint8_t u8id;          /*!< Slave address between 1 and 247. 0 means broadcast */
    uint8_t u8fct;         /*!< Function code: 1, 2, 3, 4, 5, 6, 15 or 16 */
    uint16_t u16RegAdd;    /*!< Address of the first register to access at slave/s */
    uint16_t u16CoilsNo;   /*!< Number of coils or registers to access */
    uint16_t *au16reg;     /*!< Pointer to memory image in master */
} modbus_t;

enum
{
    RESPONSE_SIZE = 6,
    EXCEPTION_SIZE = 3,
    CHECKSUM_SIZE = 2
};

enum MESSAGE
{
    ID = 0, //!< ID field
    FUNC, //!< Function code position
    ADD_HI, //!< Address high byte
    ADD_LO, //!< Address low byte
    NB_HI, //!< Number of coils or registers high byte
    NB_LO, //!< Number of coils or registers low byte
    BYTE_CNT  //!< byte counter
};

enum MB_FC
{
    MB_FC_NONE = 0,   /*!< null operator */
    MB_FC_READ_COILS = 1, /*!< FCT=1 -> read coils or digital outputs */
    MB_FC_READ_DISCRETE_INPUT = 2, /*!< FCT=2 -> read digital inputs */
    MB_FC_READ_REGISTERS = 3, /*!< FCT=3 -> read registers or analog outputs */
    MB_FC_READ_INPUT_REGISTER = 4, /*!< FCT=4 -> read analog inputs */
    MB_FC_WRITE_COIL = 5, /*!< FCT=5 -> write single coil or output */
    MB_FC_WRITE_REGISTER = 6, /*!< FCT=6 -> write single register */
    MB_FC_WRITE_MULTIPLE_COILS = 15, /*!< FCT=15 -> write multiple coils or outputs */
    MB_FC_WRITE_MULTIPLE_REGISTERS = 16 /*!< FCT=16 -> write multiple registers */
};

enum COM_STATES
{
    COM_IDLE = 0,
    COM_WAITING = 1
};

enum ERR_LIST
{
    ERR_NOT_MASTER = -1,
    ERR_POLLING = -2,
    ERR_BUFF_OVERFLOW = -3,
    ERR_BAD_CRC = -4,
    ERR_EXCEPTION = -5
};

enum
{
    NO_REPLY = 255,
    EXC_FUNC_CODE = 1,
    EXC_ADDR_RANGE = 2,
    EXC_REGS_QUANT = 3,
    EXC_EXECUTE = 4
};

const unsigned char fctsupported[] =
{
    MB_FC_READ_COILS,
    MB_FC_READ_DISCRETE_INPUT,
    MB_FC_READ_REGISTERS,
    MB_FC_READ_INPUT_REGISTER,
    MB_FC_WRITE_COIL,
    MB_FC_WRITE_REGISTER,
    MB_FC_WRITE_MULTIPLE_COILS,
    MB_FC_WRITE_MULTIPLE_REGISTERS
};

#define T35  5
#define MAX_BUFFER  64 //!< maximum size for the communication buffer in bytes

typedef struct
{
    Stream *port; //!< Pointer to Stream class object (Either HardwareSerial or SoftwareSerial)
    uint8_t u8id; //!< 0=master, 1..247=slave number
    uint8_t u8txenpin; //!< flow control pin: 0=USB or RS-232 mode, >1=RS-485 mode
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
} Modbus;

void Modbus_init(Modbus *modbus, uint8_t u8id, Stream *port, uint8_t u8txenpin)
{
    modbus->u8id = u8id;
    modbus->port = port;
    modbus->u8txenpin = u8txenpin;
    modbus->u8state = COM_IDLE;
    modbus->u8lastError = 0;
    modbus->u8BufferSize = 0;
    modbus->u16InCnt = 0;
    modbus->u16OutCnt = 0;
    modbus->u16errCnt = 0;
    modbus->u16timeOut = 1000; // default timeout
}

void Modbus_start(Modbus *modbus)
{
    // Start the Modbus communication
    modbus->u8state = COM_WAITING;
}

void Modbus_setTimeOut(Modbus *modbus, uint16_t u16timeOut)
{
    modbus->u16timeOut = u16timeOut;
}

uint16_t Modbus_getTimeOut(Modbus *modbus)
{
    return modbus->u16timeOut;
}

uint8_t Modbus_getID(Modbus *modbus)
{
    return modbus->u8id;
}

void Modbus_setID(Modbus *modbus, uint8_t u8id)
{
    modbus->u8id = u8id;
}

uint16_t Modbus_getInCnt(Modbus *modbus)
{
    return modbus->u16InCnt;
}

uint16_t Modbus_getOutCnt(Modbus *modbus)
{
    return modbus->u16OutCnt;
}

uint16_t Modbus_getErrCnt(Modbus *modbus)
{
    return modbus->u16errCnt;
}

uint8_t Modbus_getLastError(Modbus *modbus)
{
    return modbus->u8lastError;
}

// Additional functions for processing Modbus requests, sending buffers, etc., need to be implemented

#endif // MODBUS_RTU_H
