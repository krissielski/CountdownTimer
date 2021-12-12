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
#define CLOCK_EN        0       //** Set to 1 for CLOCK enabled
#define COUNTDOWN_EN    0       //** Set to 1 for COUNTDOWN 

#define WARLOCKS_EN     0       //** Set to 1 to enable scrolling Warlocks Banner/Display         

#define WARLOCKS_TIME   30      //Seconds between Banner/Scrolling effects



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

DateTime curr_time;
DateTime goal_time; //Countdown timer end time/date

bool irq_flag = false;

char message[128];



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

    //Goal setup - this time/date to countdown to
    //                    yr, mo, dy, hr, min, sec 
    goal_time = DateTime( 22, 03, 22, 23, 59,  59 );


}



void loop() {

  bool ledtoggle=false;
  
  char dispstr[20];

  uint32_t sm_timer   = 0;
  uint8_t  curr_state = 0;
  uint8_t  prev_state = 255;




  Serial.print("Countdown Timer FRC 1507\n");
 
    
  if (! rtc.isrunning())
  {
    Serial.println("RTC lost power. Set the time!");
  }


  ////// Test     //////

  // strcpy(message,"12345678 ABCDEFGH IJKLMNOP QRSTUVWXYZ .,!");
  // RunScroll(message);
  
  ////// End Test //////


  //Enable Interrupts
  irq_flag = false;
  interrupts(); //Enable IRQs
  

  //Infinite Loop
  while(1)
  {


    //************************************************************************
    if( irq_flag )
    {
      irq_flag = false;
      curr_time = rtc.now();
      
//    printDate(curr_time);
//    printSpan(goal_time,curr_time);
      
      //Did state machine change states?
      if( curr_state != prev_state )
      {
        prev_state = curr_state;
        sm_timer = 0;
        Serial.print("sm>");Serial.print(curr_state);Serial.println();    //SM Debug
      }

      //State Machine....
      switch ( curr_state )
      {
        case 0:
          RunScroll("HELLO!");
          curr_state++;
          break;
      
        case 1:
          DisplayCurrTime(curr_time);
          if( sm_timer == 15 )          
            curr_state++;
          break;

        case 2:
          DisplayCountdownTime( curr_time, goal_time );
          if( sm_timer == 15 )
            curr_state++;
          break;

        case 3:
          RunScroll("HI!");
          curr_state++;
          break;

        case 4:
          RunEffects();
          curr_state = 1;   //Go back to time!
          break; 


        default:
          //Should never get here
          curr_state = 0;   //Reset!
          break;
      }
      
      //Clean up....
      sm_timer++;         //Bump timer

      
       //blink LED
      digitalWrite(PIN_LED,ledtoggle?HIGH:LOW);
      ledtoggle = !ledtoggle;  
     
    }
    // End if(irq_flag)
    //************************************************************************


    //Place all tasks here while waiting in infinite loop
    serialComm();             //Check for serial Communications



  }  //End Infinite Loop
}   //End LOOP
//***************************************************************************




//***************************************************************************
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

      //Handle empty line
      if( index == 0 )                  //Check for empty line
      {
        //Empty line
        Serial.print("\n> ");
      }
      else
      {
        //A command arrived        
        rxbuffer[index]   = 0;          //null string
        serialParse(rxbuffer,index);    //Parse string
        index = 0;                      //Reset Index
      }
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
          if( len == 1 ){ printDate(curr_time); return; }      
        break;
      
      case 't':
          if( len == 1 ){ printDate(goal_time); return; }      
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
    Sure3208.printString("$",offset); 
    offset--;
    if( offset==-75) return;

    serialComm();             //Check for any serial Communications
  }
}





//Banner/scrolling effects
void RunScroll( char *string )
{
  int16_t offset = 64; //Start at Far Right

  int16_t endoffset =  (  strlen(string) * 6 ) * -1;    //6 = 5dots + 1 space
  

  while(1)
  {
    delay(75); 
    Sure3208.printString(string,offset); 
    offset--;
    if( offset == endoffset) break;

    serialComm();             //Check for any serial Communications
    
  }
}




//***************************************************************************
void DisplayCurrTime( DateTime curr_time )
{
  char dispstr[20];

  //Default.  Display dd:hh:mm:ss
  uint8_t hour = curr_time.hour();

  if( hour>12) hour=hour-12;
              
  dispstr[0]  = (hour<10) ? ' ': ('0'+ (hour / 10));  //If <10, ditch leading zero
  dispstr[1]  = '0'+ (hour % 10);
  dispstr[2]  = ':';

  dispstr[3]  = '0'+ (curr_time.minute() / 10);
  dispstr[4]  = '0'+ (curr_time.minute() % 10);
  dispstr[5]  = ':';

  dispstr[6]  = '0'+ (curr_time.second() / 10 );
  dispstr[7]  = '0'+ (curr_time.second() % 10 );

  dispstr[8]  = 0;  //Null String
        
  Sure3208.printString(dispstr,9);  

}


//***************************************************************************
void DisplayCountdownTime( DateTime curr_time, DateTime goal_time )
{

  char dispstr[20];          
  TimeSpan t = goal_time-curr_time;
  
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
}


