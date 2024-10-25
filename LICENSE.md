#include <stdint.h>
#include "ModbusRtu.h"

/* _____PUBLIC FUNCTIONS_____________________________________________________ */

/**
 * @brief
 * Constructor for a Master/Slave in C.
 * 
 * In C, we use a struct to represent the Modbus object and pass it to the functions.
 * @param u8id   node address 0=master, 1..247=slave
 * @param port   serial port used (Stream type pointer)
 * @param u8txenpin pin for txen RS-485 (=0 means USB/RS232C mode)
 */
void Modbus_init(Modbus* modbus, uint8_t u8id, Stream* port, uint8_t u8txenpin) {
    modbus->port = port;
    modbus->u8id = u8id;
    modbus->u8txenpin = u8txenpin;
    modbus->u16timeOut = 1000;
    modbus->u32overTime = 0;
}

/**
 * @brief
 * DEPRECATED constructor for a Master/Slave in C.
 * THIS FUNCTION IS ONLY PROVIDED FOR BACKWARDS COMPATIBILITY.
 * USE Modbus_init() INSTEAD.
 * 
 * @param u8id   node address 0=master, 1..247=slave
 * @param u8serno  serial port used 0..3 (ignored for software serial)
 * @param u8txenpin pin for txen RS-485 (=0 means USB/RS232C mode)
 */
void Modbus_init_deprecated(Modbus* modbus, uint8_t u8id, uint8_t u8serno, uint8_t u8txenpin) {
    modbus->u8id = u8id;
    modbus->u8txenpin = u8txenpin;
    modbus->u16timeOut = 1000;
    modbus->u32overTime = 0;

    switch (u8serno) {
#if defined(UBRR1H)
    case 1:
        modbus->port = &Serial1;
        break;
#endif

#if defined(UBRR2H)
    case 2:
        modbus->port = &Serial2;
        break;
#endif

#if defined(UBRR3H)
    case 3:
        modbus->port = &Serial3;
        break;
#endif
    case 0:
    default:
        modbus->port = &Serial;
        break;
    }
}

/**
 * @brief
 * Start-up class object in C.
 * Call this AFTER calling begin() on the serial port, typically within setup().
 * 
 * @param modbus  Pointer to the Modbus object
 */
void Modbus_start(Modbus* modbus) {
    if (modbus->u8txenpin > 1) {   // pin 0 & pin 1 are reserved for RX/TX
        // return RS485 transceiver to transmit mode
        pinMode(modbus->u8txenpin, OUTPUT);
        digitalWrite(modbus->u8txenpin, LOW);
    }

    while (modbus->port->read() >= 0);
    modbus->u8lastRec = modbus->u8BufferSize = 0;
    modbus->u16InCnt = modbus->u16OutCnt = modbus->u16errCnt = 0;
}

/**
 * @brief
 * Install a serial port, begin() it, and start ModbusRtu in C.
 * 
 * @param install_port pointer to Stream type (e.g., SoftwareSerial or HardwareSerial)
 * @param u32speed     baud rate, in standard increments (300..115200)
 */
void Modbus_begin(Modbus* modbus, Stream* install_port, long u32speed) {
    modbus->port = install_port;
    install_port->begin(u32speed);
    Modbus_start(modbus);
}

/**
 * @brief
 * Install a serial port, begin() it, and start ModbusRtu in C.
 * 
 * @param install_port  pointer to Stream type (e.g., SoftwareSerial or HardwareSerial)
 * @param u32speed      baud rate, in standard increments (300..115200)
 * @param u8txenpin     pin for txen RS-485 (=0 means USB/RS232C mode)
 */
void Modbus_begin_with_txen(Modbus* modbus, Stream* install_port, long u32speed, uint8_t u8txenpin) {
    modbus->u8txenpin = u8txenpin;
    modbus->port = install_port;
    install_port->begin(u32speed);
    Modbus_start(modbus);
}

/**
 * @brief
 * Begin hardware serial port and start ModbusRtu in C.
 * 
 * @param modbus Pointer to the Modbus object
 * @param u32speed baud rate, in standard increments (300..115200). Default=19200
 */
void Modbus_begin_hw(Modbus* modbus, long u32speed) {
    // !!Can ONLY do this if port ACTUALLY IS a HardwareSerial object!!
    ((HardwareSerial*)modbus->port)->begin(u32speed);
    Modbus_start(modbus);
}

/**
 * @brief
 * Method to write a new slave ID address in C.
 * 
 * @param modbus Pointer to the Modbus object
 * @param u8id new slave address between 1 and 247
 */
void Modbus_setID(Modbus* modbus, uint8_t u8id) {
    if ((u8id != 0) && (u8id <= 247)) {
        modbus->u8id = u8id;
    }
}

/**
 * @brief
 * Method to write the overtime count for txend pin in C.
 * 
 * @param modbus Pointer to the Modbus object
 * @param u32overTime count for txend pin
 */
void Modbus_setTxendPinOverTime(Modbus* modbus, uint32_t u32overTime) {
    modbus->u32overTime = u32overTime;
}

/**
 * @brief
 * Method to read current slave ID address in C.
 * 
 * @param modbus Pointer to the Modbus object
 * @return u8id current slave address between 1 and 247
 */
uint8_t Modbus_getID(Modbus* modbus) {
    return modbus->u8id;
}

/**
 * @brief
 * Initialize time-out parameter in C.
 * 
 * @param modbus Pointer to the Modbus object
 * @param u16timeOut time-out value (ms)
 */
void Modbus_setTimeOut(Modbus* modbus, uint16_t u16timeOut) {
    modbus->u16timeOut = u16timeOut;
}

/**
 * @brief
 * Return communication Watchdog state in C.
 * 
 * @param modbus Pointer to the Modbus object
 * @return TRUE if millis() > u32timeOut
 */
uint8_t Modbus_getTimeOutState(Modbus* modbus) {
    return ((unsigned long)(millis() - modbus->u32timeOut) > (unsigned long)modbus->u16timeOut);
}

/**
 * @brief
 * Get input messages counter value in C.
 * 
 * @param modbus Pointer to the Modbus object
 * @return input messages counter
 */
uint16_t Modbus_getInCnt(Modbus* modbus) {
    return modbus->u16InCnt;
}

/**
 * @brief
 * Get transmitted messages counter value in C.
 * 
 * @param modbus Pointer to the Modbus object
 * @return transmitted messages counter
 */
uint16_t Modbus_getOutCnt(Modbus* modbus) {
    return modbus->u16OutCnt;
}

/**
 * @brief
 * Get errors counter value in C.
 * 
 * @param modbus Pointer to the Modbus object
 * @return errors counter
 */
uint16_t Modbus_getErrCnt(Modbus* modbus) {
    return modbus->u16errCnt;
}

/**
 * @brief
 * Get modbus master state in C.
 * 
 * @param modbus Pointer to the Modbus object
 * @return = 0 IDLE, = 1 WAITING FOR ANSWER
 */
uint8_t Modbus_getState(Modbus* modbus) {
    return modbus->u8state;
}

/**
 * @brief
 * Get the last error in the protocol processor in C.
 * 
 * @param modbus Pointer to the Modbus object
 * @return Error code (e.g., NO_REPLY, EXC_FUNC_CODE, etc.)
 */
uint8_t Modbus_getLastError(Modbus* modbus) {
    return modbus->u8lastError;
}

/**
 * @brief
 * Generate a query to a slave with a modbus_t telegram structure in C.
 * The Master must be in COM_IDLE mode. After this, its state would be COM_WAITING.
 * This function should only be called in the loop() section.
 *
 * @param modbus Pointer to the Modbus object
 * @param telegram modbus telegram structure (id, fct, ...)
 * @return Status code (-3 for invalid ID, -2 for invalid Master ID, -1 for busy state, 0 for success)
 */
int8_t Modbus_query(Modbus* modbus, modbus_t telegram) {
    uint8_t u8regsno, u8bytesno;
    if (modbus->u8id != 0) return -2;
    if (modbus->u8state != COM_IDLE) return -1;

    if ((telegram.u8id == 0) || (telegram.u8id > 247)) return -3;

    modbus->au16regs = telegram.au16reg;

    // telegram header
    modbus->au8Buffer[ID] = telegram.u8id;
    modbus->au8Buffer[FUNC] = telegram.u8fct;
    modbus->au8Buffer[ADD_HI] = highByte(telegram.u16RegAdd);
    modbus->au8Buffer[ADD_LO] = lowByte(telegram.u16RegAdd);

    switch (telegram.u8fct) {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUT:
    case MB_FC_READ_REGISTERS:
    case MB_FC_READ_INPUT_REGISTER:
        modbus->au8Buffer[NB_HI] = highByte(telegram.u16CoilsNo);
        modbus->au8Buffer[NB_LO] = lowByte(telegram.u16CoilsNo);
        modbus->u8BufferSize = 6;
        break;
    case MB_FC_WRITE_COIL:
        modbus->au8Buffer[NB_HI] = ((modbus->au16regs[0] > 0) ? 0xff : 0);
        modbus->au8Buffer[NB_LO] = 0;
        modbus->u8BufferSize = 6;
        break;
    case MB_FC_WRITE_REGISTER:
        modbus->au8Buffer[NB_HI] = highByte(modbus->au16regs[0]);
        modbus->au8Buffer[NB_LO] = lowByte(modbus->au16regs[0]);
        modbus->u8BufferSize = 6;
        break;
    case MB_FC_WRITE_MULTIPLE_COILS: // TODO: implement "sending coils"
        u8regsno = telegram.u16CoilsNo / 16;
        u8bytesno = u8regsno * 2;
        if ((telegram.u16CoilsNo % 16) != 0) {
            u8bytesno++;
            u8regsno++;
        }

        modbus->au8Buffer[NB_HI] = highByte(telegram.u16CoilsNo);
        modbus->au8Buffer[NB_LO] = lowByte(telegram.u16CoilsNo);
        modbus->au8Buffer[BYTE_CNT] = u8bytesno;
        modbus->u8BufferSize = 7;

        for (uint16_t i = 0; i < u8bytesno; i++) {
            if (i % 2) {
                modbus->au8Buffer[modbus->u8BufferSize] = lowByte(modbus->au16regs[i / 2]);
            } else {
                modbus->au8Buffer[modbus->u8BufferSize] = highByte(modbus->au16regs[i / 2]);
            }
            modbus->u8BufferSize++;
        }
        break;

    case MB_FC_WRITE_MULTIPLE_REGISTERS:
        modbus->au8Buffer[NB_HI] = highByte(telegram.u16CoilsNo);
        modbus->au8Buffer[NB_LO] = lowByte(telegram.u16CoilsNo);
        modbus->au8Buffer[BYTE_CNT] = (uint8_t)(telegram.u16CoilsNo * 2);
        modbus->u8BufferSize = 7;

        for (uint16_t i = 0; i < telegram.u16CoilsNo; i++) {
            modbus->au8Buffer[modbus->u8BufferSize] = highByte(modbus->au16regs[i]);
            modbus->u8BufferSize++;
            modbus->au8Buffer[modbus->u8BufferSize] = lowByte(modbus->au16regs[i]);
            modbus->u8BufferSize++;
        }
        break;
    }

    Modbus_sendTxBuffer(modbus);
    modbus->u8state = COM_WAITING;
    modbus->u8lastError = 0;
    return 0;
}

/**
 * @brief *** Only for Modbus Master ***
 * This function checks if there is any incoming answer if pending in C.
 * If there is no answer, it changes the Master state to COM_IDLE.
 * This function must be called only in the loop section.
 * Avoid any delay() function.
 *
 * Any incoming data is redirected to au16regs pointer,
 * as defined in its modbus_t query telegram.
 *
 * @param modbus Pointer to the Modbus object
 * @return errors counter
 */
int8_t Modbus_poll(Modbus* modbus) {
    // check if there is any incoming frame
    uint8_t u8current;
    u8current = modbus->port->available();

    if ((unsigned long)(millis() - modbus->u32timeOut) > (unsigned long)modbus->u16timeOut) {
        modbus->u8state = COM_IDLE;
        modbus->u8lastError = NO_REPLY;
        modbus->u16errCnt++;
        return 0;
    }

    if (u8current == 0) return 0;

    // check T35 after frame end or still no frame end
    if (u8current != modbus->u8lastRec) {
        modbus->u8lastRec = u8current;
        modbus->u32time = millis();
        return 0;
    }
    if ((unsigned long)(millis() - modbus->u32time) < (unsigned long)T35) return 0;

    // transfer Serial buffer frame to auBuffer
    modbus->u8lastRec = 0;
    int8_t i8state = Modbus_getRxBuffer(modbus);
    if (i8state < 6) {
        modbus->u8state = COM_IDLE;
        modbus->u16errCnt++;
        return i8state;
    }

    // validate message: id, CRC, FCT, exception
    uint8_t u8exception = Modbus_validateAnswer(modbus);
    if (u8exception != 0) {
        modbus->u8state = COM_IDLE;
        return u8exception;
    }

    // process answer
    switch (modbus->au8Buffer[FUNC]) {
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
        // Nothing to do
        break;
    default:
        break;
    }

    modbus->u8state = COM_IDLE;
    return 0;
}

/**
 * @brief *** Only for Modbus Slave ***
 * This function checks if there is any incoming query in C.
 * If there is no query, it returns 0.
 * This function must be called only in the loop section.
 * Avoid any delay() function.
 *
 * Any incoming data is redirected to au16regs pointer,
 * as defined in its modbus_t query telegram.
 *
 * @param modbus Pointer to the Modbus object
 * @param regs Register table for communication exchange
 * @param u8size Size of the register table
 * @return 0 if no query, 1..4 if communication error, >4 if correct query processed
 */
int8_t Modbus_poll_slave(Modbus* modbus, uint16_t *regs, uint8_t u8size) {
    modbus->au16regs = regs;
    modbus->u8regsize = u8size;
    uint8_t u8current;

    // check if there is any incoming frame
    u8current = modbus->port->available();

    if (u8current == 0) return 0;

    // check T35 after frame end or still no frame end
    if (u8current != modbus->u8lastRec) {
        modbus->u8lastRec = u8current;
        modbus->u32time = millis();
        return 0;
    }
    if ((unsigned long)(millis() - modbus->u32time) < (unsigned long)T35) return 0;

    modbus->u8lastRec = 0;
    int8_t i8state = Modbus_getRxBuffer(modbus);
    modbus->u8lastError = i8state;
    if (i8state < 7) return i8state;

    // check slave id
    if (modbus->au8Buffer[ID] != modbus->u8id) return 0;

    // validate message: CRC, FCT, address and size
    uint8_t u8exception = Modbus_validateRequest(modbus);
    if (u8exception > 0) {
        if (u8exception != NO_REPLY) {
            Modbus_buildException(modbus, u8exception);
            Modbus_sendTxBuffer(modbus);
        }
        modbus->u8lastError = u8exception;
        return u8exception;
    }

    modbus->u32timeOut = millis();
    modbus->u8lastError = 0;

    // process message
    switch (modbus->au8Buffer[FUNC]) {
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

/* _____PRIVATE FUNCTIONS_____________________________________________________ */

/**
 * @brief
 * This function moves Serial buffer data to the Modbus au8Buffer in C.
 *
 * @param modbus Pointer to the Modbus object
 * @return buffer size if OK, ERR_BUFF_OVERFLOW if u8BufferSize >= MAX_BUFFER
 */
int8_t Modbus_getRxBuffer(Modbus* modbus) {
    uint8_t bBuffOverflow = 0;

    if (modbus->u8txenpin > 1) digitalWrite(modbus->u8txenpin, LOW);

    modbus->u8BufferSize = 0;
    while (modbus->port->available()) {
        modbus->au8Buffer[modbus->u8BufferSize] = modbus->port->read();
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

/**
 * @brief
 * This function transmits au8Buffer to Serial line in C.
 * Only if u8txenpin != 0, there is a flow handling in order to keep
 * the RS485 transceiver in output state as long as the message is being sent.
 * The CRC is appended to the buffer before starting to send it.
 *
 * @param modbus Pointer to the Modbus object
 */
void Modbus_sendTxBuffer(Modbus* modbus) {
    // append CRC to message
    uint16_t u16crc = Modbus_calcCRC(modbus, modbus->u8BufferSize);
    modbus->au8Buffer[modbus->u8BufferSize] = u16crc >> 8;
    modbus->u8BufferSize++;
    modbus->au8Buffer[modbus->u8BufferSize] = u16crc & 0x00ff;
    modbus->u8BufferSize++;

    if (modbus->u8txenpin > 1) {
        // set RS485 transceiver to transmit mode
        digitalWrite(modbus->u8txenpin, HIGH);
    }

    // transfer buffer to serial line
    modbus->port->write(modbus->au8Buffer, modbus->u8BufferSize);

    if (modbus->u8txenpin > 1) {
        // must wait transmission end before changing pin state
        modbus->port->flush();
        // return RS485 transceiver to receive mode
        volatile uint32_t u32overTimeCountDown = modbus->u32overTime;
        while (u32overTimeCountDown-- > 0);
        digitalWrite(modbus->u8txenpin, LOW);
    }
    while (modbus->port->read() >= 0);

    modbus->u8BufferSize = 0;

    // set time-out for master
    modbus->u32timeOut = millis();

    // increase message counter
    modbus->u16OutCnt++;
}

/**
 * @brief
 * This function calculates CRC in C.
 *
 * @param modbus Pointer to the Modbus object
 * @param u8length Length of the message to calculate CRC for
 * @return uint16_t calculated CRC value for the message
 */
uint16_t Modbus_calcCRC(Modbus* modbus, uint8_t u8length) {
    unsigned int temp, temp2, flag;
    temp = 0xFFFF;
    for (unsigned char i = 0; i < u8length; i++) {
        temp = temp ^ modbus->au8Buffer[i];
        for (unsigned char j = 1; j <= 8; j++) {
            flag = temp & 0x0001;
            temp >>= 1;
            if (flag) temp ^= 0xA001;
        }
    }
    // Reverse byte order.
    temp2 = temp >> 8;
    temp = (temp << 8) | temp2;
    temp &= 0xFFFF;
    return temp;
}

/**
 * @brief
 * This function validates slave incoming messages in C.
 *
 * @param modbus Pointer to the Modbus object
 * @return 0 if OK, EXCEPTION if anything fails
 */
uint8_t Modbus_validateRequest(Modbus* modbus) {
    // check message crc vs calculated crc
    uint16_t u16MsgCRC = ((modbus->au8Buffer[modbus->u8BufferSize - 2] << 8) | modbus->au8Buffer[modbus->u8BufferSize - 1]);
    if (Modbus_calcCRC(modbus, modbus->u8BufferSize - 2) != u16MsgCRC) {
        modbus->u16errCnt++;
        return NO_REPLY;
    }
    return 0;
}
