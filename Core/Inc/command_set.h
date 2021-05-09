#ifndef MX25R6435F_COMMAND_HEAD
#define MX25R6435F_COMMAND_HEAD

// READ
// normal read
#define READ                   0x03 // address 1 line, dummy cycle 0, data 1 line MSB
#define FAST_READ              0x0B // address 1 line, dummy cycle 8, data 1 line MSB
#define TWO_READ               0xBB // address 2 line, dummy cycle 4, data 2 line MSB
#define D_READ                 0x3B // address 1 line, dummy cycle 8, data 2 line MSB
#define FOUR_READ              0xEB // address 4 line, dummy cycle 4, data 4 line MSB,alter byte 4 lines,
#define Q_READ                 0x6B // address 1 line, dummy cycle 8, data 4 line MSB

// program
#define PAGE_PROGRAM           0x02 // address 1 line, dummy cycle 0, data 1 line MSB
#define QUAL_PAGE_PROGRAM      0x38 // address 4 line, dummy cycle 0, data 4 line MSB
#define READ_SFDP              0x5A

// erase 
#define SECTOR_ERASE           0x20 // address 1 line, dummy cycle 0
#define BLOCK_ERASE_32K        0x52 // address 1 line, dummy cycle 0
#define BLOCK_ERASE_64K        0xD8 // address 1 line, dummy cycle 0
#define CHIP_ERASE             0x60

// register/setting
#define WRITE_ENABLE           0x06
#define WRITE_DISABLE          0x04
#define READ_STATUS_REG        0x05 
#define READ_CONF_STATUS_REG   0x15
#define WRITE_STATUS_REG       0x01
#define PGM_SUSPEND            0x75
#define PGM_RESUME             0x7A
#define DEEP_POWER             0xB9
#define SET_BURST_LENGTH       0xC0

// ID/RESET Command
#define READ_ID                0x9F 
#define READ_ELE_ID            0x90
#define ENTER_SECURE_OTP       0xB1
#define EXIT_SECURE_OTP        0xC1
#define READ_SECURE_REG        0x2B
#define WRITE_SECURE_REG       0x2F
#define NOP                    0x00
#define RESET_ENABLE           0x66
#define RESET_MEM              0x99

#endif