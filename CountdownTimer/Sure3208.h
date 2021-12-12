// Sure3208.h
//  Library for controlling the Sure 3208 LED Dot Matrix Display (with HT1632C led controller).
//   This code should be able to control up to 4 daisy-chained 3208 boards for purposes of
//   displaying date and time.  (The intention is to display only numbers and a few symbols).
//
//  **IMPT NOTE:  This is for V2 of the 3208 modules: 12V power, 10 pin header, No Dip Switch.
//
//  Author:     K. Sielski, for FRC Team 1507 - Warlocks (Lockport,NY)
//  Date:       12/15/2016
//

#ifndef SURE3208_H
#define SURE3208_H

#include <Arduino.h>


//** DEFINES **
#define HT1632C_DISP_MEM_SIZE   32    //32 bytes (Matrix is 8x32)

// HT1632 ID List
#define HT1632C_ID_READ         0x600
#define HT1632C_ID_WRITE        0x500
#define HT1632C_ID_CMD          0x400

//Commands are 12 bit:      (Actually 11 bits of valid data left shifted once for 12 bits)
//** NOTE ** We will Left shift the data in code!!!!!!!
//  III-CCCC-CCCC-X  
//   I = 3-bit ID Code
//   C = Command Code
//   X = Don't Care
#define HT1632C_CMD_SYS_DIS     0x00
#define HT1632C_CMD_SYS_EN      0x01
#define HT1632C_CMD_LED_OFF     0x02
#define HT1632C_CMD_LED_ON      0x03
#define HT1632C_CMD_BLINK_OFF   0x08
#define HT1632C_CMD_BLINK_ON    0x09
#define HT1632C_CMD_SLAVE_MODE  0x10
#define HT1632C_CMD_MASTER_MODE 0x18
#define HT1632C_CMD_EXT_CLK     0x1C
#define HT1632C_CMD_PWM_DUTY    0xA0

#define HT1632C_CMD_COM_N_8     0x20
#define HT1632C_CMD_COM_N_16    0x24
#define HT1632C_CMD_COM_P_8     0x28
#define HT1632C_CMD_COM_P_16    0x2C


//Sure3208 Class
class Sure3208_class
{
  private:
    uint8_t pinCS;       //Note: CS is Active LOW
    uint8_t num_boards;  //Num Sure3208 V2 boards
    uint8_t pinCK;
    uint8_t pinWR;
    uint8_t pinDATA;

    uint8_t *mem;        // Display buffer that is sent to 3208 boards
    uint8_t mem_size;    //mem size in bytes  

    void init(void);
    void select( uint8_t mask );
    void sendCommand( uint8_t cmd );
    void sendData( uint16_t data, uint8_t numbits );
    void sendDispMem(void);


  public:
    void begin(uint8_t numBoards, uint8_t CSpin, uint8_t CKpin, uint8_t WRpin, uint8_t DATApin);

    void clearMemory(void);

    void printString(char *string, int16_t offset);
    void printBuffer(uint8_t *buffer, int8_t size);
  
};

extern Sure3208_class Sure3208;


#endif  //SURE3208_H
