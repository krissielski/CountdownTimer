//CountdownTimer
//SparkFun Arduino Redboard (Arduino Uno)
//
//  Author:     K. Sielski, for FRC Team 1507 - Warlocks (Lockport,NY)
//  Date:       12/15/2016
//

// ** INCLUDES **
//#include <Arduino.h>
#include "Sure3208.h"
#include "RTClib.h"
#include <Wire.h>


// ** SETTINGS **
#define USE_AS_CLOCK    0       //** Set to 1 for CLOCK operation (disable countdown timer)
#define WARLOCKS_EN     0       //** Set to 1 to enable scrolling Warlocks Banner/Display         

#define WARLOCKS_TIME   30      //Seconds between Banner/Scrolling effects

#define ADD_ONEHOUR     0       //Set to 1 for Daylight Savings (Add +1 hour)

// ** DEFINES **
#define PIN_LED   13
#define PIN_IRQ   3
#define PIN_TP    2

#define PIN_CS    7
#define PIN_CK    6
#define PIN_WR    4
#define PIN_DATA  5


//Globals
RTC_DS3231 rtc;

DateTime now;
DateTime then;

bool irq_flag = false;



//IRQ Handler
void IrqHandler(void)
{
  irq_flag = true;
}



void setup() {

    //Setup Pins
    pinMode(PIN_LED, OUTPUT);   //Debug LED Output
    pinMode(PIN_TP,  OUTPUT);   //Testpoint
    pinMode(PIN_IRQ, INPUT);    //IRQ Input
    digitalWrite(PIN_IRQ,HIGH); //Enable PU
    
    attachInterrupt (digitalPinToInterrupt(PIN_IRQ), IrqHandler, RISING);  // attach interrupt handler
  
    //Setup Serial Monitor
    Serial.begin(115200);     

    //Sure 3208 Board Setup
    Sure3208.begin(2,PIN_CS,PIN_CK,PIN_WR,PIN_DATA);  //num_boards/cs/ck/wr/data

    //Setup DS3231 DTC
    rtc.begin();
    rtc.enableOscillator(true,false,0);

    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    //THEN SETUP - this time/date to countdown to
    //               yr, mo, dy, hr, min, sec 
    then = DateTime( 20, 02, 22, 23, 59,  59 );


}



void loop() {

  bool ledtoggle=false;
  
  char dispstr[20];

  uint32_t next_disp_time = 30*1000;  //30 sec



  Serial.print("Countdown Timer FRC 1507\n");
 
    
  if (! rtc.isrunning())
  {
    Serial.println("RTC lost power. Set the time!");
  }


  //Enable Interrupts
  irq_flag = false;
  interrupts(); //Enable IRQs
  

  //Infinite Loop
  while(1)
  {


    //************************************************************************
    if( WARLOCKS_EN && ( millis() > next_disp_time)  )
    {
      RunEffects();
      next_disp_time += WARLOCKS_TIME * 1000;     
    }


    //************************************************************************
    if( irq_flag )
    {
      irq_flag = false;
      now = rtc.now();
      
//    printDate(now);
//    printSpan(then,now);
      
#if USE_AS_CLOCK==1
      //Default.  Display dd:hh:mm:ss
      uint8_t hour = now.hour();

#if ADD_ONEHOUR==1
      //Account for Daylight Savings
      hour += 1;
#endif  //Add one hour
      
      if( hour>12) hour=hour-12;
             
      dispstr[0]  = (hour<10) ? ' ': ('0'+ (hour / 10));  //If <10, ditch leading zero
      dispstr[1]  = '0'+ (hour % 10);
      dispstr[2]  = ':';
      
      dispstr[3]  = '0'+ (now.minute() / 10);
      dispstr[4]  = '0'+ (now.minute() % 10);
      dispstr[5]  = ':';
      
      dispstr[6]  = '0'+ (now.second() / 10 );
      dispstr[7]  = '0'+ (now.second() % 10 );

      dispstr[8]  = 0;  //Null String
        
      Sure3208.printString(dispstr,9);  
      
#else
            
      TimeSpan t = then-now;
      
      //Some checks on the results....
      if( t.totalseconds() < 0 ) 
      {
        //** We already passeed the date.  Shows zeros...
        Sure3208.printString("00:00:00:00",1);    
      }
      else if( t.days() > 99 )
      {
        //** Num days>99 - we don't have enough room for ddd:hh:mm:ss.  
        //So we have to do ddd:hh:mm
        uint16_t days = t.days();        
        uint8_t  hdays = days/100;
                 days  = days%100;
        uint8_t  tdays = days/10;
        uint8_t  odays = days%10;
        
        dispstr[0]  = '0'+ hdays;
        dispstr[1]  = '0'+ tdays;
        dispstr[2]  = '0'+ odays;
        dispstr[3]  = ':';
        
        dispstr[4]  = '0'+(t.hours() / 10 );
        dispstr[5]  = '0'+(t.hours() % 10 );
        dispstr[6]  = ':';
        
        dispstr[7]  = '0'+(t.minutes() / 10 );
        dispstr[8]  = '0'+(t.minutes() % 10 );
        dispstr[9]  = 0;  //Null String
          
        Sure3208.printString(dispstr,5);    
      }
      else
      {
        //Default.  Display dd:hh:mm:ss
        dispstr[0]  = '0'+ (t.days() / 10 );
        dispstr[1]  = '0'+ (t.days() % 10 );
        dispstr[2]  = ':';
        
        dispstr[3]  = '0'+(t.hours() / 10 );
        dispstr[4]  = '0'+(t.hours() % 10 );
        dispstr[5]  = ':';
        
        dispstr[6]  = '0'+(t.minutes() / 10 );
        dispstr[7]  = '0'+(t.minutes() % 10 );
        dispstr[8]  = ':';
    
        dispstr[9]  = '0'+(t.seconds() / 10 );
        dispstr[10] = '0'+(t.seconds() % 10 );
        dispstr[11] = 0;  //Null String
          
        Sure3208.printString(dispstr,1);   
      } 
  
#endif  
      
       digitalWrite(PIN_TP,ledtoggle?HIGH:LOW);
       
       //blink LED
      digitalWrite(PIN_LED,ledtoggle?HIGH:LOW);
      ledtoggle = !ledtoggle;  
     
    }
    // End if(irq_flag)
    //************************************************************************


    //Place all tasks here while waiting in infinite loop
    serialComm();             //Check for serial Communications



  }  //End Infinite Loop
}


//Serial Comm
void serialComm( void )
{
  static char rxbuffer[50];
  static uint8_t index = 0;
         char rxbyte;

  while( Serial.available() )
  {

    rxbyte = Serial.read();

    if( rxbyte=='?' )
    {
      printHelp();
      index=0;
      return;
    }
  
    if( index == 50 )
    {
      index=0;
      return;
    }
  
    if( rxbyte == '\r' ) return;
  
    if( rxbyte == '\n')
    {
      rxbuffer[index]   = 0;        //null string
      //Serial.println(rxbuffer);

      serialParse(rxbuffer,index);
  
      index = 0;                    //Reset Index
      return;
    }
  
    //Load rx data into buffer
    rxbuffer[index++] = rxbyte;

  }
}



void serialParse( char *string, uint8_t len )
{
  uint8_t y,m,d,hh,mm,ss; //temp working vars
  
  if(len==0) return;
  
    switch( string[0] )
    {

      case 'c':
          if( len == 1 ){ printDate(now); return; }      
        break;
      
      case 't':
          if( len == 1 ){ printDate(then); return; }      
        break;     
      
      case 's':
          if( len == 19 )
          {
            y  = conv2d(&string[2]);
            m  = conv2d(&string[5]);
            d  = conv2d(&string[8]);
            hh = conv2d(&string[11]);
            mm = conv2d(&string[14]);
            ss = conv2d(&string[17]);   
   
            rtc.adjust(DateTime(2000+(uint16_t)y, m, d, hh, mm, ss));
            
            return;
          }
          //Error
          Serial.println("S usage:  s yy mm dd hh mm ss");
      
        break;

      default:
        Serial.println("Unknown Command");
        break;
    }
}

void printDate(DateTime dt)
{
      Serial.print(dt.year(), DEC);
      Serial.print('/');
      Serial.print(dt.month(), DEC);
      Serial.print('/');
      Serial.print(dt.day(), DEC);
      Serial.print(" ");
  
      Serial.print(dt.hour(), DEC);
      Serial.print(':');
      Serial.print(dt.minute(), DEC);
      Serial.print(':');
      Serial.print(dt.second(), DEC);
      Serial.println();
}

void printSpan(DateTime a, DateTime b)
{
      TimeSpan t = a-b;
      Serial.print(t.days(), DEC);
      Serial.print(":");
      Serial.print(t.hours(), DEC);
      Serial.print(':');
      Serial.print(t.minutes(), DEC);
      Serial.print(':');
      Serial.print(t.seconds(), DEC);
      Serial.println();   
}

void printHelp(void)
{
      Serial.println();  
      Serial.print("  FRC 1507 Couuntdown Timer");
      Serial.println();  
      Serial.println();  
      Serial.println("Commands:");
      Serial.println("  c                       - Show current date/time");
      Serial.println("  s yy mm dd hh mm ss     - Set current date/time");
      Serial.println("  t                       - Show countdown date/time");   
      Serial.println();
}

static uint8_t conv2d(const char* p) {
    uint8_t v = 0;
    if ('0' <= *p && *p <= '9')
        v = *p - '0';
    return 10 * v + *++p - '0';
}


//Banner/scrolling effects
void RunEffects(void)
{
  int8_t offset = 64;

  while(1)
  {
    delay(75); 
    Sure3208.printString("X",offset); 
    offset--;
    if( offset==-75) return;

    serialComm();             //Check for any serial Communications
  }
}
