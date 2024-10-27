#include "ModbusRtu.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h> // Added to provide millis() function
#include <stdint.h>
#include <stdbool.h>

// Deklarasi fungsi set_baud_rate
void set_baud_rate(FILE* port, long u32speed);

// Mock-up function for millis() using clock()
unsigned long millis() {
    unsigned long clocks_per_sec = CLOCKS_PER_SEC;
    if (clocks_per_sec == 0) {
        clocks_per_sec = 1000; // Default value if CLOCKS_PER_SEC is not properly defined
    }
    return (unsigned long)(clock() / (clocks_per_sec / 1000));
}

Modbus* Modbus_new(uint8_t u8id, FILE* port, uint8_t u8txenpin) {
    Modbus* modbus = (Modbus*)malloc(sizeof(Modbus));
    if (modbus == NULL) {
        return NULL; // Handle memory allocation failure
    }
    
    modbus->port = port;
    modbus->u8id = u8id;
    modbus->u8txenpin = u8txenpin;
    modbus->u16timeOut = 1000;
    modbus->u32overTime = 0;
    
    return modbus;
}

void Modbus_delete(Modbus* modbus) {
    if (modbus != NULL) {
        free(modbus);
    }
}

// Placeholder definitions for Serial, Serial1, Serial2, Serial3
#if !defined(Serial)
#define Serial ((FILE *)0) // Replace with appropriate file pointer for Serial port
#endif
#if !defined(Serial1)
#define Serial1 ((FILE *)1) // Replace with appropriate file pointer for Serial1
#endif
#if !defined(Serial2)
#define Serial2 ((FILE *)2) // Replace with appropriate file pointer for Serial2
#endif
#if !defined(Serial3)
#define Serial3 ((FILE *)3) // Replace with appropriate file pointer for Serial3
#endif

Modbus* Modbus_new_with_serial(uint8_t u8id, uint8_t u8serno, uint8_t u8txenpin) {
    Modbus* modbus = (Modbus*)malloc(sizeof(Modbus));
    if (modbus == NULL) {
        return NULL; // Handle memory allocation failure
    }
    
    modbus->u8id = u8id;
    modbus->u8txenpin = u8txenpin;
    modbus->u16timeOut = 1000;
    modbus->u32overTime = 0;

    switch (u8serno) {
#if defined(UBRR1H)
        case 1:
            modbus->port = Serial1;
            break;
#endif

#if defined(UBRR2H)
        case 2:
            modbus->port = Serial2;
            break;
#endif

#if defined(UBRR3H)
        case 3:
            modbus->port = Serial3;
            break;
#endif
        case 0:
        default:
            modbus->port = Serial;
            break;
    }
    
    return modbus;
}

void Modbus_free(Modbus* modbus) {
    if (modbus != NULL) {
        free(modbus);
    }
}

void Modbus_start(Modbus* modbus) {
    if (modbus->u8txenpin > 1) { // pin 0 & pin 1 are reserved for RX/TX
        // Assuming pinMode and digitalWrite are available in your environment
#if defined(ARDUINO)
        pinMode(modbus->u8txenpin, 1); // Assuming OUTPUT is defined as 1
        digitalWrite(modbus->u8txenpin, 0); // Assuming LOW is defined as 0
#else
        // Placeholder for platforms that do not support pinMode/digitalWrite
        // Replace this with appropriate GPIO handling for your environment
#endif
    }

    // Clear the input buffer
    while (fgetc(modbus->port) != EOF);

    modbus->u8lastRec = 0;
    modbus->u8BufferSize = 0;
    modbus->u16InCnt = 0;
    modbus->u16OutCnt = 0;
    modbus->u16errCnt = 0;
}

void Modbus_begin(Modbus* modbus, FILE* install_port, long u32speed) {
    modbus->port = install_port;
    set_baud_rate(install_port, u32speed); // Placeholder to set baud rate
    Modbus_start(modbus);
}

void Modbus_begin_with_txenpin(Modbus* modbus, FILE* install_port, long u32speed, uint8_t u8txenpin) {
    modbus->u8txenpin = u8txenpin;
    modbus->port = install_port;
    set_baud_rate(install_port, u32speed); // Placeholder to set baud rate
    Modbus_start(modbus);
}

void Modbus_begin_simple(Modbus* modbus, long u32speed) {
    set_baud_rate(modbus->port, u32speed); // Cast to appropriate port and set baud rate
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
    return ((unsigned long)(millis() - modbus->u32timeOut) > (unsigned long)modbus->u16timeOut);
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

// Placeholder for setting baud rate
void set_baud_rate(FILE* port, long u32speed) {
    // Implement the baud rate setting for the given port
}

// Functions to handle different types of streams
void Modbus_start_with_stream(Modbus* modbus, void* stream) {
    modbus->port = (FILE*)stream;
    Modbus_start(modbus);
}

int8_t Modbus_query(Modbus* modbus, modbus_t telegram) {
    uint8_t u8regsno, u8bytesno;
    if (modbus->u8id != 0) return -2;
    if (modbus->u8state != COM_IDLE) return -1;

    if ((telegram.u8id == 0) || (telegram.u8id > 247)) return -3;

    modbus->au16regs = telegram.au16reg;

    modbus->au8Buffer[ID] = telegram.u8id;
    modbus->au8Buffer[FUNC] = telegram.u8fct;
    modbus->au8Buffer[ADD_HI] = (uint8_t)(telegram.u16RegAdd >> 8);
    modbus->au8Buffer[ADD_LO] = (uint8_t)(telegram.u16RegAdd & 0xFF);

    switch (telegram.u8fct) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUT:
        case MB_FC_READ_REGISTERS:
        case MB_FC_READ_INPUT_REGISTER:
            modbus->au8Buffer[NB_HI] = (uint8_t)(telegram.u16CoilsNo >> 8);
            modbus->au8Buffer[NB_LO] = (uint8_t)(telegram.u16CoilsNo & 0xFF);
            modbus->u8BufferSize = 6;
            break;
        case MB_FC_WRITE_COIL:
            modbus->au8Buffer[NB_HI] = ((modbus->au16regs[0] > 0) ? 0xFF : 0);
            modbus->au8Buffer[NB_LO] = 0;
            modbus->u8BufferSize = 6;
            break;
        case MB_FC_WRITE_REGISTER:
            modbus->au8Buffer[NB_HI] = (uint8_t)(modbus->au16regs[0] >> 8);
            modbus->au8Buffer[NB_LO] = (uint8_t)(modbus->au16regs[0] & 0xFF);
            modbus->u8BufferSize = 6;
            break;
        case MB_FC_WRITE_MULTIPLE_COILS:
            u8regsno = telegram.u16CoilsNo / 16;
            u8bytesno = u8regsno * 2;
            if ((telegram.u16CoilsNo % 16) != 0) {
                u8bytesno++;
                u8regsno++;
            }

            modbus->au8Buffer[NB_HI] = (uint8_t)(telegram.u16CoilsNo >> 8);
            modbus->au8Buffer[NB_LO] = (uint8_t)(telegram.u16CoilsNo & 0xFF);
            modbus->au8Buffer[BYTE_CNT] = u8bytesno;
            modbus->u8BufferSize = 7;

            for (uint16_t i = 0; i < u8bytesno; i++) {
                if (i % 2) {
                    modbus->au8Buffer[modbus->u8BufferSize] = (uint8_t)(modbus->au16regs[i / 2] & 0xFF);
                } else {
                    modbus->au8Buffer[modbus->u8BufferSize] = (uint8_t)(modbus->au16regs[i / 2] >> 8);
                }
                modbus->u8BufferSize++;
            }
            break;
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            modbus->au8Buffer[NB_HI] = (uint8_t)(telegram.u16CoilsNo >> 8);
            modbus->au8Buffer[NB_LO] = (uint8_t)(telegram.u16CoilsNo & 0xFF);
            modbus->au8Buffer[BYTE_CNT] = (uint8_t)(telegram.u16CoilsNo * 2);
            modbus->u8BufferSize = 7;

            for (uint16_t i = 0; i < telegram.u16CoilsNo; i++) {
                modbus->au8Buffer[modbus->u8BufferSize] = (uint8_t)(modbus->au16regs[i] >> 8);
                modbus->u8BufferSize++;
                modbus->au8Buffer[modbus->u8BufferSize] = (uint8_t)(modbus->au16regs[i] & 0xFF);
                modbus->u8BufferSize++;
            }
            break;
    }

    void Modbus_sendTxBuffer(Modbus* modbus); // Forward declaration

    Modbus_sendTxBuffer(modbus);
    modbus->u8state = COM_WAITING;
    modbus->u8lastError = 0;
    return 0;
}

// Modbus_poll
// Forward declarations for undefined functions
uint8_t Modbus_available(Modbus* modbus);
int8_t Modbus_getRxBuffer(Modbus* modbus);
uint8_t Modbus_validateAnswer(Modbus* modbus);
void Modbus_get_FC1(Modbus* modbus);
void Modbus_get_FC3(Modbus* modbus);

int8_t Modbus_poll(Modbus* modbus) {
    uint8_t u8current;
    u8current = Modbus_available(modbus);

    if ((unsigned long)(millis() - modbus->u32timeOut) > (unsigned long)modbus->u16timeOut) {
        modbus->u8state = COM_IDLE;
        modbus->u8lastError = NO_REPLY;
        modbus->u16errCnt++;
        return 0;
    }

    if (u8current == 0) return 0;

    if (u8current != modbus->u8lastRec) {
        modbus->u8lastRec = u8current;
        modbus->u32time = millis();
        return 0;
    }
    if ((unsigned long)(millis() - modbus->u32time) < (unsigned long)T35) return 0;

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
            // nothing to do
            break;
        default:
            break;
    }
    modbus->u8state = COM_IDLE;
    return modbus->u8BufferSize;
}

// Implementasi fungsi poll yang sesuai dengan header
int8_t Modbus_pollWithRegs(Modbus* modbus, uint16_t *regs, uint8_t u8size) {
    modbus->au16regs = regs;
    modbus->u8regsize = u8size;
    uint8_t u8current;

    // Mengecek apakah data tersedia di serial port
    // NOTE: Fungsi fgetc hanya digunakan sebagai contoh, perlu diganti dengan API yang tepat untuk serial
    if (modbus->port != NULL) {
        u8current = fgetc(modbus->port);
    } else {
        u8current = 0;
    }
    
    if (u8current == EOF) {
        // Jika tidak ada data yang diterima
        return 0;
    }

    // Mengecek jika ada perubahan pada buffer terakhir
    if (u8current != modbus->u8lastRec) {
        modbus->u8lastRec = u8current;
        modbus->u32time = millis();
        return 0;
    }

    if ((unsigned long)(millis() - modbus->u32time) < (unsigned long)T35) {
        return 0;
    }

    // Reset last received
    modbus->u8lastRec = 0;

    // Mengambil data dari buffer
    int8_t i8state = modbus->getRxBuffer(modbus);
    modbus->u8lastError = i8state;
    if (i8state < 7) {
        return i8state;
    }

    // Memeriksa ID slave
    if (modbus->au8Buffer[ID] != modbus->u8id) {
        return 0;
    }

    // Memvalidasi pesan: CRC, FCT, alamat dan ukuran
    uint8_t u8exception = modbus->validateRequest(modbus);
    if (u8exception > 0) {
        if (u8exception != NO_REPLY) {
            modbus->buildException(modbus, u8exception);
            modbus->sendTxBuffer(modbus);
        }
        modbus->u8lastError = u8exception;
        return u8exception;
    }

    modbus->u32timeOut = millis();
    modbus->u8lastError = 0;

    // Memproses pesan berdasarkan fungsi kode (function code)
    switch (modbus->au8Buffer[FUNC]) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUT:
            return modbus->process_FC1(modbus, regs, u8size);
            break;
        case MB_FC_READ_INPUT_REGISTER:
        case MB_FC_READ_REGISTERS:
            return modbus->process_FC3(modbus, regs, u8size);
            break;
        case MB_FC_WRITE_COIL:
            return modbus->process_FC5(modbus, regs, u8size);
            break;
        case MB_FC_WRITE_REGISTER:
            return modbus->process_FC6(modbus, regs, u8size);
            break;
        case MB_FC_WRITE_MULTIPLE_COILS:
            return modbus->process_FC15(modbus, regs, u8size);
            break;
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            return modbus->process_FC16(modbus, regs, u8size);
            break;
        default:
            break;
    }

    return i8state;
}

// Implementasi getRxBuffer untuk Modbus dalam C
int8_t Modbus_getRxBuffer(Modbus* modbus) {
    bool bBuffOverflow = false;

    // Mengatur pin TX enable ke LOW jika diperlukan (pengganti digitalWrite)
    if (modbus->u8txenpin > 1) {
        // Placeholder untuk menggantikan fungsi digitalWrite pada platform yang relevan
        // Misalnya, gunakan GPIO handling yang sesuai untuk platform Anda
        printf("Setting TX enable pin %d to LOW\n", modbus->u8txenpin);
    }

    modbus->u8BufferSize = 0;

    // Membaca data dari port
    while (1) {
        int c = fgetc(modbus->port);
        if (c == EOF) {
            break;  // Jika tidak ada data lagi
        }

        // Menyimpan data ke buffer
        modbus->au8Buffer[modbus->u8BufferSize] = (uint8_t)c;
        modbus->u8BufferSize++;

        // Mengecek apakah buffer overflow terjadi
        if (modbus->u8BufferSize >= MAX_BUFFER) {
            bBuffOverflow = true;
            break;  // Keluar dari loop jika buffer penuh
        }
    }

    // Menambah jumlah hitungan data masuk
    modbus->u16InCnt++;

    // Jika terjadi buffer overflow
    if (bBuffOverflow) {
        modbus->u16errCnt++;
        return ERR_BUFF_OVERFLOW;
    }

    return modbus->u8BufferSize;
}

// Implementasi sendTxBuffer untuk Modbus dalam C
void Modbus_sendTxBuffer(Modbus* modbus) {
    // Menghitung CRC dan menambahkan ke buffer
    uint16_t u16crc = modbus->calcCRC(modbus, modbus->u8BufferSize);
    modbus->au8Buffer[modbus->u8BufferSize] = u16crc >> 8;  // CRC high byte
    modbus->u8BufferSize++;
    modbus->au8Buffer[modbus->u8BufferSize] = u16crc & 0x00ff;  // CRC low byte
    modbus->u8BufferSize++;

    // Mengatur TX enable pin ke HIGH jika diperlukan (menggantikan digitalWrite)
    if (modbus->u8txenpin > 1) {
        // Placeholder untuk mengatur GPIO sesuai platform yang digunakan
        printf("Setting TX enable pin %d to HIGH\n", modbus->u8txenpin);
    }

    // Menulis buffer ke port serial
    if (modbus->port != NULL) {
        for (uint8_t i = 0; i < modbus->u8BufferSize; i++) {
            fputc(modbus->au8Buffer[i], modbus->port);  // Menulis byte ke serial
        }
    }

    // Mengatur TX enable pin ke LOW setelah transmisi selesai
    if (modbus->u8txenpin > 1) {
        // Simulasikan `flush()` dengan penundaan untuk memastikan buffer dikirim sepenuhnya
        fflush(modbus->port);  // Flush output buffer untuk memastikan semua data dikirim
        volatile uint32_t u32overTimeCountDown = modbus->u32overTime;
        while (u32overTimeCountDown-- > 0);  // Menunggu waktu "over time" selesai
        printf("Setting TX enable pin %d to LOW\n", modbus->u8txenpin);
    }

    // Membersihkan input serial (mengganti `port->read()` di C++)
    while (modbus->port != NULL && fgetc(modbus->port) != EOF) {
        // Bersihkan data yang tersisa
    }

    // Mengatur ukuran buffer kembali ke 0
    modbus->u8BufferSize = 0;

    // Memperbarui time out
    modbus->u32timeOut = millis();

    // Menambah hitungan data keluar
    modbus->u16OutCnt++;
}

// Implementasi calcCRC untuk Modbus dalam C
uint16_t Modbus_calcCRC(Modbus* modbus, uint8_t u8length) {
    unsigned int temp, temp2, flag;
    temp = 0xFFFF;

    // Perulangan untuk setiap byte dalam buffer
    for (unsigned char i = 0; i < u8length; i++) {
        temp = temp ^ modbus->au8Buffer[i];  // XOR byte saat ini dengan nilai CRC

        // Perulangan untuk setiap bit dalam byte
        for (unsigned char j = 1; j <= 8; j++) {
            flag = temp & 0x0001;  // Memeriksa bit LSB
            temp >>= 1;            // Geser ke kanan satu bit
            if (flag) {
                temp ^= 0xA001;    // XOR dengan polinomial jika bit LSB adalah 1
            }
        }
    }

    // Tukar byte CRC agar sesuai dengan format Modbus (Little Endian)
    temp2 = temp >> 8;            // Ambil high byte dari CRC
    temp = (temp << 8) | temp2;   // Gabungkan dengan low byte
    temp &= 0xFFFF;               // Masking untuk memastikan CRC 16-bit

    return temp;  // Mengembalikan nilai CRC
}

// Implementasi validateRequest untuk Modbus dalam C
uint8_t Modbus_validateRequest(Modbus* modbus) {
    // Menggabungkan CRC dari dua byte terakhir dalam buffer
    uint16_t u16MsgCRC = ((modbus->au8Buffer[modbus->u8BufferSize - 2] << 8) |
                          modbus->au8Buffer[modbus->u8BufferSize - 1]);

    // Memeriksa apakah CRC yang dihitung cocok dengan CRC di dalam pesan
    if (modbus->calcCRC(modbus, modbus->u8BufferSize - 2) != u16MsgCRC) {
        modbus->u16errCnt++;
        return NO_REPLY;
    }

    // Memeriksa kode fungsi apakah didukung
    bool isSupported = false;
    for (uint8_t i = 0; i < sizeof(fctsupported); i++) {
        if (fctsupported[i] == modbus->au8Buffer[FUNC]) {
            isSupported = true;
            break;
        }
    }

    // Jika fungsi tidak didukung, kembalikan kode exception
    if (!isSupported) {
        modbus->u16errCnt++;
        return EXC_FUNC_CODE;
    }

    // Memeriksa alamat dan jumlah register
    uint16_t u16regs = 0;
    uint8_t u8regs;

    switch (modbus->au8Buffer[FUNC]) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUT:
        case MB_FC_WRITE_MULTIPLE_COILS:
            u16regs = ((modbus->au8Buffer[ADD_HI] << 8) | modbus->au8Buffer[ADD_LO]) / 16;
            u16regs += ((modbus->au8Buffer[NB_HI] << 8) | modbus->au8Buffer[NB_LO]) / 16;
            u8regs = (uint8_t)u16regs;
            if (u8regs > modbus->u8regsize) {
                return EXC_ADDR_RANGE;
            }
            break;

        case MB_FC_WRITE_COIL:
            u16regs = ((modbus->au8Buffer[ADD_HI] << 8) | modbus->au8Buffer[ADD_LO]) / 16;
            u8regs = (uint8_t)u16regs;
            if (u8regs > modbus->u8regsize) {
                return EXC_ADDR_RANGE;
            }
            break;

        case MB_FC_WRITE_REGISTER:
            u16regs = (modbus->au8Buffer[ADD_HI] << 8) | modbus->au8Buffer[ADD_LO];
            u8regs = (uint8_t)u16regs;
            if (u8regs > modbus->u8regsize) {
                return EXC_ADDR_RANGE;
            }
            break;

        case MB_FC_READ_REGISTERS:
        case MB_FC_READ_INPUT_REGISTER:
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            u16regs = (modbus->au8Buffer[ADD_HI] << 8) | modbus->au8Buffer[ADD_LO];
            u16regs += (modbus->au8Buffer[NB_HI] << 8) | modbus->au8Buffer[NB_LO];
            u8regs = (uint8_t)u16regs;
            if (u8regs > modbus->u8regsize) {
                return EXC_ADDR_RANGE;
            }
            break;

        default:
            // Jika kode fungsi tidak sesuai dengan yang diharapkan
            break;
    }

    // Jika semua pemeriksaan lulus, kembalikan nilai 0 (tidak ada exception)
    return 0;
}

// Implementasi validateAnswer untuk Modbus dalam C
uint8_t Modbus_validateAnswer(Modbus* modbus) {
    // Menggabungkan CRC dari dua byte terakhir dalam buffer
    uint16_t u16MsgCRC = ((modbus->au8Buffer[modbus->u8BufferSize - 2] << 8) |
                          modbus->au8Buffer[modbus->u8BufferSize - 1]);

    // Memeriksa apakah CRC yang dihitung cocok dengan CRC di dalam pesan
    if (modbus->calcCRC(modbus, modbus->u8BufferSize - 2) != u16MsgCRC) {
        modbus->u16errCnt++;
        return NO_REPLY;
    }

    // Memeriksa bit error di dalam kode fungsi (fungsi error ditandai dengan bit MSB yang di-set)
    if ((modbus->au8Buffer[FUNC] & 0x80) != 0) {
        modbus->u16errCnt++;
        return ERR_EXCEPTION;
    }

    // Memeriksa apakah kode fungsi didukung
    bool isSupported = false;
    for (uint8_t i = 0; i < sizeof(fctsupported); i++) {
        if (fctsupported[i] == modbus->au8Buffer[FUNC]) {
            isSupported = true;
            break;
        }
    }

    // Jika fungsi tidak didukung, kembalikan kode exception
    if (!isSupported) {
        modbus->u16errCnt++;
        return EXC_FUNC_CODE;
    }

    // Jika semua pemeriksaan lulus, kembalikan nilai 0 (tidak ada exception)
    return 0;
}

// Implementasi buildException untuk Modbus dalam C
void Modbus_buildException(Modbus* modbus, uint8_t u8exception) {
    // Mengambil kode fungsi asli dari buffer
    uint8_t u8func = modbus->au8Buffer[FUNC];

    // Membangun pesan exception
    modbus->au8Buffer[ID] = modbus->u8id;
    modbus->au8Buffer[FUNC] = u8func + 0x80;  // Menandai bahwa ini adalah exception dengan menambah 0x80
    modbus->au8Buffer[2] = u8exception;       // Kode exception
    modbus->u8BufferSize = EXCEPTION_SIZE;    // Ukuran buffer exception yang ditetapkan
}

// Implementasi get_FC1 untuk Modbus dalam C
void Modbus_get_FC1(Modbus* modbus) {
    uint8_t u8byte = 3;  // Offset untuk data dalam buffer
    uint8_t i;

    // Perulangan untuk memproses setiap byte data
    for (i = 0; i < modbus->au8Buffer[2]; i++) {
        if (i % 2) {
            // Jika `i` adalah ganjil, menggabungkan byte saat ini dengan low byte dari register
            modbus->au16regs[i / 2] = ((modbus->au8Buffer[i + u8byte] << 8) |
                                       (modbus->au16regs[i / 2] & 0x00FF));
        } else {
            // Jika `i` adalah genap, menggabungkan high byte dari register dengan byte saat ini
            modbus->au16regs[i / 2] = ((modbus->au16regs[i / 2] & 0xFF00) |
                                       modbus->au8Buffer[i + u8byte]);
        }
    }
}

// Implementasi get_FC3 untuk Modbus dalam C
void Modbus_get_FC3(Modbus* modbus) {
    uint8_t u8byte = 3;  // Offset untuk data dalam buffer
    uint8_t i;

    // Perulangan untuk memproses setiap register yang diterima
    for (i = 0; i < (modbus->au8Buffer[2] / 2); i++) {
        // Menggabungkan dua byte menjadi satu register 16-bit
        modbus->au16regs[i] = ((modbus->au8Buffer[u8byte] << 8) | modbus->au8Buffer[u8byte + 1]);
        u8byte += 2;  // Increment by 2 untuk mengakses register berikutnya
    }
}

// Implementasi process_FC1 untuk Modbus dalam C
int8_t Modbus_process_FC1(Modbus* modbus, uint16_t* regs, uint8_t u8size) {
    uint8_t u8currentRegister, u8currentBit, u8bytesno, u8bitsno;
    uint8_t u8CopyBufferSize;
    uint16_t u16currentCoil, u16coil;

    // Menghitung nilai awal coil dan jumlah coil berdasarkan buffer
    uint16_t u16StartCoil = (modbus->au8Buffer[ADD_HI] << 8) | modbus->au8Buffer[ADD_LO];
    uint16_t u16Coilno = (modbus->au8Buffer[NB_HI] << 8) | modbus->au8Buffer[NB_LO];

    // Menghitung jumlah byte yang diperlukan untuk menyimpan status coil
    u8bytesno = (uint8_t)(u16Coilno / 8);
    if (u16Coilno % 8 != 0) {
        u8bytesno++;
    }
    modbus->au8Buffer[ADD_HI] = u8bytesno;
    modbus->u8BufferSize = ADD_LO;

    // Inisialisasi nilai akhir buffer terakhir menjadi 0
    modbus->au8Buffer[modbus->u8BufferSize + u8bytesno - 1] = 0;

    // Inisialisasi bit index yang sedang diakses
    u8bitsno = 0;

    // Perulangan untuk setiap coil yang harus dibaca
    for (u16currentCoil = 0; u16currentCoil < u16Coilno; u16currentCoil++) {
        u16coil = u16StartCoil + u16currentCoil;
        u8currentRegister = (uint8_t)(u16coil / 16);
        u8currentBit = (uint8_t)(u16coil % 16);

        // Mengatur bit di buffer untuk status coil yang dibaca
        if ((regs[u8currentRegister] & (1 << u8currentBit)) != 0) {
            modbus->au8Buffer[modbus->u8BufferSize] |= (1 << u8bitsno);
        } else {
            modbus->au8Buffer[modbus->u8BufferSize] &= ~(1 << u8bitsno);
        }
        u8bitsno++;

        // Jika sudah mencapai 8 bit, pindah ke byte berikutnya
        if (u8bitsno > 7) {
            u8bitsno = 0;
            modbus->u8BufferSize++;
        }
    }

    // Jika jumlah coil tidak habis dibagi 8, tambahkan satu byte ke ukuran buffer
    if (u16Coilno % 8 != 0) {
        modbus->u8BufferSize++;
    }
    u8CopyBufferSize = modbus->u8BufferSize + 2;

    // Mengirim buffer ke port serial
    modbus->sendTxBuffer(modbus);

    return u8CopyBufferSize;
}

// Implementasi process_FC3 untuk Modbus dalam C
int8_t Modbus_process_FC3(Modbus* modbus, uint16_t* regs, uint8_t u8size) {
    uint16_t u16StartAdd = (modbus->au8Buffer[ADD_HI] << 8) | modbus->au8Buffer[ADD_LO];
    uint16_t u16regsno = (modbus->au8Buffer[NB_HI] << 8) | modbus->au8Buffer[NB_LO];
    uint8_t u8CopyBufferSize;
    uint16_t i;

    // Mengatur jumlah byte data yang akan dikirim
    modbus->au8Buffer[2] = u16regsno * 2;
    modbus->u8BufferSize = 3;

    // Mengisi buffer dengan nilai register
    for (i = u16StartAdd; i < u16StartAdd + u16regsno; i++) {
        modbus->au8Buffer[modbus->u8BufferSize] = (uint8_t)(regs[i] >> 8);  // High byte dari register
        modbus->u8BufferSize++;
        modbus->au8Buffer[modbus->u8BufferSize] = (uint8_t)(regs[i] & 0xFF); // Low byte dari register
        modbus->u8BufferSize++;
    }

    u8CopyBufferSize = modbus->u8BufferSize + 2;

    // Mengirim buffer ke port serial
    modbus->sendTxBuffer(modbus);

    return u8CopyBufferSize;
}

// Implementasi process_FC5 untuk Modbus dalam C
int8_t Modbus_process_FC5(Modbus* modbus, uint16_t* regs, uint8_t u8size) {
    uint8_t u8currentRegister, u8currentBit;
    uint8_t u8CopyBufferSize;

    // Menggabungkan dua byte untuk mendapatkan alamat coil
    uint16_t u16coil = (modbus->au8Buffer[ADD_HI] << 8) | modbus->au8Buffer[ADD_LO];
    u8currentRegister = (uint8_t)(u16coil / 16);  // Menghitung register saat ini berdasarkan alamat coil
    u8currentBit = (uint8_t)(u16coil % 16);       // Menghitung posisi bit dalam register

    // Mengatur nilai bit di register sesuai dengan nilai yang diterima
    if (modbus->au8Buffer[NB_HI] == 0xFF) {
        regs[u8currentRegister] |= (1 << u8currentBit);  // Menulis '1' pada bit
    } else {
        regs[u8currentRegister] &= ~(1 << u8currentBit); // Menulis '0' pada bit
    }

    // Mengatur ukuran buffer
    modbus->u8BufferSize = 6;
    u8CopyBufferSize = modbus->u8BufferSize + 2;

    // Mengirim buffer ke port serial
    modbus->sendTxBuffer(modbus);

    return u8CopyBufferSize;
}

// Implementasi process_FC6 untuk Modbus dalam C
int8_t Modbus_process_FC6(Modbus* modbus, uint16_t* regs, uint8_t u8size) {
    uint8_t u8add = (modbus->au8Buffer[ADD_HI] << 8) | modbus->au8Buffer[ADD_LO];
    uint8_t u8CopyBufferSize;
    uint16_t u16val = (modbus->au8Buffer[NB_HI] << 8) | modbus->au8Buffer[NB_LO];

    // Menulis nilai register
    regs[u8add] = u16val;

    // Mengatur ukuran buffer untuk respons
    modbus->u8BufferSize = RESPONSE_SIZE;

    u8CopyBufferSize = modbus->u8BufferSize + 2;

    // Mengirim buffer ke port serial
    modbus->sendTxBuffer(modbus);

    return u8CopyBufferSize;
}

// Implementasi process_FC15 untuk Modbus dalam C
int8_t Modbus_process_FC15(Modbus* modbus, uint16_t* regs, uint8_t u8size) {
    uint8_t u8currentRegister, u8currentBit, u8frameByte, u8bitsno;
    uint8_t u8CopyBufferSize;
    uint16_t u16currentCoil, u16coil;
    bool bTemp;

    // Menggabungkan dua byte untuk mendapatkan alamat awal coil
    uint16_t u16StartCoil = (modbus->au8Buffer[ADD_HI] << 8) | modbus->au8Buffer[ADD_LO];
    uint16_t u16Coilno = (modbus->au8Buffer[NB_HI] << 8) | modbus->au8Buffer[NB_LO];

    // Inisialisasi untuk memulai penulisan bit
    u8bitsno = 0;
    u8frameByte = 7;  // Posisi byte data pertama untuk coil

    // Perulangan untuk memproses setiap coil yang harus diperbarui
    for (u16currentCoil = 0; u16currentCoil < u16Coilno; u16currentCoil++) {
        u16coil = u16StartCoil + u16currentCoil;
        u8currentRegister = (uint8_t)(u16coil / 16);  // Menghitung register saat ini
        u8currentBit = (uint8_t)(u16coil % 16);       // Menghitung posisi bit dalam register

        // Membaca bit dari buffer dan menyimpannya dalam `bTemp`
        bTemp = (modbus->au8Buffer[u8frameByte] & (1 << u8bitsno)) != 0;

        // Menulis nilai bit ke register yang sesuai
        if (bTemp) {
            regs[u8currentRegister] |= (1 << u8currentBit);  // Menulis '1' pada bit
        } else {
            regs[u8currentRegister] &= ~(1 << u8currentBit); // Menulis '0' pada bit
        }

        // Mengatur bit index
        u8bitsno++;

        // Jika sudah mencapai 8 bit, pindah ke byte berikutnya
        if (u8bitsno > 7) {
            u8bitsno = 0;
            u8frameByte++;
        }
    }

    // Mengatur ukuran buffer respons
    modbus->u8BufferSize = 6;
    u8CopyBufferSize = modbus->u8BufferSize + 2;

    // Mengirim buffer ke port serial
    modbus->sendTxBuffer(modbus);

    return u8CopyBufferSize;
}

// Implementasi process_FC16 untuk Modbus dalam C
int8_t Modbus_process_FC16(Modbus* modbus, uint16_t* regs, uint8_t u8size) {
    uint16_t u16StartAdd = (modbus->au8Buffer[ADD_HI] << 8) | modbus->au8Buffer[ADD_LO];
    uint16_t u16regsno = (modbus->au8Buffer[NB_HI] << 8) | modbus->au8Buffer[NB_LO];
    uint8_t u8CopyBufferSize;
    uint8_t i;
    uint16_t temp;

    // Mengatur buffer respons untuk jumlah register yang di-update
    modbus->au8Buffer[NB_HI] = 0;
    modbus->au8Buffer[NB_LO] = (uint8_t)u16regsno;
    modbus->u8BufferSize = RESPONSE_SIZE;

    // Mengisi nilai register dengan data yang diterima dari buffer
    for (i = 0; i < u16regsno; i++) {
        temp = (modbus->au8Buffer[(BYTE_CNT + 1) + i * 2] << 8) |
               (modbus->au8Buffer[(BYTE_CNT + 2) + i * 2]);
        
        regs[u16StartAdd + i] = temp;
    }

    // Menghitung ukuran buffer untuk respons
    u8CopyBufferSize = modbus->u8BufferSize + 2;

    // Mengirim buffer ke port serial
    modbus->sendTxBuffer(modbus);

    return u8CopyBufferSize;
}
