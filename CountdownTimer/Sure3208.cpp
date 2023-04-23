//

#include "Sure3208.h"
#include "kwsFont.h"


void Sure3208_class::begin(uint8_t numBoards, uint8_t CSpin, uint8_t CKpin, uint8_t WRpin, uint8_t DATApin)
{
    pinCS       = CSpin;
    num_boards  = numBoards;
    pinCK       = CKpin;
    pinWR       = WRpin;
    pinDATA     = DATApin;
    
    if( numBoards > 4) numBoards=4;  //Max 4 boards
    
    init();
}

void Sure3208_class::init(void)
{
  //Init Pins

  pinMode(pinCS,OUTPUT);
  digitalWrite(pinCS,HIGH);       //CS Active Low.  Init HIGH
  
  pinMode(pinCK,OUTPUT);
  digitalWrite(pinCK,LOW);
  
  pinMode(pinWR,OUTPUT);
  digitalWrite(pinWR,HIGH);       //Datasheet seems to want to start HIGH

  pinMode(pinDATA,OUTPUT);
  digitalWrite(pinDATA,LOW);

  //Init CS Shift Sequence
  select(0x00);

  //Init Memory.....
  mem_size = HT1632C_DISP_MEM_SIZE * num_boards;      // num bytes needed to contain all board display data
  mem = (uint8_t*)malloc( mem_size );
  clearMemory();

  //Small delay to allow 3208's to power up/init
  delay(500);  //500ms

  //Now Init 3208 boards (HT1632C chips)
  sendCommand( HT1632C_CMD_SYS_EN );
  sendCommand( HT1632C_CMD_LED_ON );
  sendCommand( HT1632C_CMD_MASTER_MODE );
  sendCommand( HT1632C_CMD_COM_N_8 );
  sendCommand( HT1632C_CMD_PWM_DUTY | 0x7 );

}

void Sure3208_class::select( uint8_t mask )
{
  uint8_t bitsel = 0x01<<(num_boards-1);//start at the right most board and work backwards
  for(uint8_t cs=0;cs<num_boards;cs++)
  {
    digitalWrite(pinCS, (mask & bitsel)?LOW:HIGH);    //Set low if mask bit set, else high   
    
    digitalWrite(pinCK,HIGH);
    digitalWrite(pinCK,LOW);
    
    bitsel >>= 1;                                         //Right Shift
  }
}

void Sure3208_class::sendCommand( uint8_t cmd )
{
  //Commands are 12 bits
  uint16_t data = (HT1632C_ID_CMD | cmd)<<1;   //Do Left Shift on cmd/data   
  select(0x0F);         //Select ALL modules
  sendData( data,12 );  //send 12 bits
  select(0x00);         //De-delect ALL modules
}



void Sure3208_class::sendData( uint16_t data, uint8_t numbits )
{
  uint16_t bitsel = 1<<(numbits-1);    //MSBit first
  for( int i=numbits;i>0;i--)
  {
    digitalWrite(pinWR,LOW);      //WR LOW
    if( data & bitsel )
      digitalWrite(pinDATA,HIGH); //Bit is High
    else
      digitalWrite(pinDATA,LOW);  //Bit is Low      
    digitalWrite(pinWR,HIGH);     //Return WR HIGH
    bitsel>>= 1;                  //Right shift
  }
  
}

void Sure3208_class::clearMemory(void)
{
  for(int i=0; i<mem_size;i++)
    mem[i] = 0;
}


//Send entire contents of mem buffer to 3208 boards....
void Sure3208_class::sendDispMem(void)
{

  uint16_t wrcmd = HT1632C_ID_WRITE>>1;     //Right shift once to get ID in correct spot. Addr=0.  
  uint8_t csmask = 0x01;
  uint8_t index = 0;
  
  for(int cs=0; cs<num_boards;cs++)
  {
    select(csmask);

    //Send Write Command
    sendData(wrcmd, 10);  //10 Bit Write command
    
    //Send Data for only this selected board
    for(int i=0;i<HT1632C_DISP_MEM_SIZE;i++)
      sendData(mem[index++],8);

    select(0x00);     //Deselect
    csmask <<= 1;     //Shift left
  }
}


void Sure3208_class::printString(const char *string, int16_t offset)
{
  uint16_t index=0;  //index into mem array
  uint8_t  *pfont;   //ptr to font data to transfer to array
  uint8_t  fsize;    //size of font to transfer

  //load mem with 0 if offset > 0 to shift string RIGHT
  if( offset > 0 )
    for(uint16_t i=0;i<offset;i++)
      mem[index++] = 0x00;


  //Now run through string and convert to raw font data
  while(*string!=0)
  {

    //Letter
    if(  (*string>='A') && (*string<='Z') )
    {
      pfont =  (uint8_t*)&fontLET[  (*string-'A') * FONT_LET_WIDTH  ];  //calculate offset into NUM font
      fsize = FONT_LET_WIDTH;
    }
    else
    if(  (*string>='1') && (*string<='9') )
    {
      pfont =  (uint8_t*)&fontLET[  (26 + *string-'1') * FONT_LET_WIDTH  ];  //calculate offset into NUM font
      fsize = FONT_LET_WIDTH;
    }
    else
    if( *string == '0'  )
    {
      pfont =  (uint8_t*)&fontLET[  (35 + *string-'0') * FONT_LET_WIDTH  ];  //calculate offset into NUM font
      fsize = FONT_LET_WIDTH;
    }
    else
    if( *string == ' '  )
    {
      pfont =  (uint8_t*)fontSP;
      fsize = sizeof(fontSP);
    }
    else
    if( *string == ':'  )
    {
      pfont =  (uint8_t*)&fontPUNC[COLON];
      fsize = 1;
    }
    else
    if( *string == '.'  )
    {
      pfont =  (uint8_t*)&fontPUNC[PERIOD];
      fsize = 1;
    }
    else
    if( *string == ','  )
    {
      pfont =  (uint8_t*)&fontPUNC[COMMA];
      fsize = 1;
    }
    else
    if( *string == '!'  )
    {
      pfont =  (uint8_t*)&fontPUNC[EXMARK];
      fsize = 1;
    }
    else
    if( *string == '$'  )
    {
      pfont = (uint8_t*)fontWAR;
      fsize = sizeof(fontWAR);
    }
    else
    {
      //Default-Error
      pfont = (uint8_t*)fontER;
      fsize = sizeof(fontER);
    }


      
    //move font into mem
    for(int i=0;i<fsize;i++)
    {

      if( offset<0)                               //check if offset is not negative 
        offset++;
      else if(index<mem_size)                     //check if there is room left in mem buffer
        mem[index++] = pgm_read_byte(&pfont[i]);  //must use pgm_read_byte here to get data from PROGMEM memory 
    }
      
    //Add space if offset is not negative and room in memory
    if( offset <0 )
      offset++;
    else if( index<mem_size )
      mem[index++] = 0;
    
    //Next Char
    string++; 
    
  }   //end while
 
  //Clear out any unused display mem
  while(index<mem_size)
    mem[index++] = 0;  //zero out rest of mem buffer
  
 
  //Now send mem to Display boards
  sendDispMem(); 
  
}

void Sure3208_class::printBuffer(uint8_t *buffer, int8_t size)
{

  //move buffer into mem
  for(int i=0;i<size;i++)
  {

    if(i<mem_size)                     //check if there is room left in mem buffer
      mem[i] = buffer[i];            

  }

  //Now send mem to Display boards
  sendDispMem(); 
}



//Hidden class Instantiation 
Sure3208_class Sure3208;
