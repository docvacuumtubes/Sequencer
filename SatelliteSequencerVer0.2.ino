//  GNU General Public License 3.0
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, version 3 of the License.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You can review the GNU General Public License at:
//     see <https://www.gnu.org/licenses/>.
//    NOTE this includes COPYLEFT. 
//




//////////////////LIBRARIES TO INCLUDE/////////////////////////////////
// include the library for the Liquid Crystal display
#include <LiquidCrystal.h>    // standard lib for liquid crystal
#include <Arduino.h>          // basic Arduino functions
#include <avr/wdt.h>          // Allows us to have a watchdog timer


/////////////ARDUINO  OUTPUTS//actual board//////////////////////////
#define SENDINPUT     4 //  D4  receives the send input 
#define ALCOUTPUT     5 //  D5 (PWM) creates alc output via averaging 
                        //  NOTE: D5 is a PWM output @ 980 Hz
#define ONOUTPUT      6 //  D6  shows when TX is on and powered normally
#define DELAYOUTPUT   7 //  D7  shows during the time that delay is active
#define PWRLEVELINPUT A0 // A0 allows reading desired power level
#define DELAYINPUT    A1 // A1 allows reading desired delay before output power after SEND
#define RELAYOUTPUT   A2 // A2 can turn on the relay output
#define KILLOUTPUT    A3 // A3  quickly brings output to zero
#define MINIMUMDELAY  15 // Mininum of 15 msec delay if you're going to the trouble of using this!

////////////////////////STATES//////////////////////////////////
int   tx_state;
int   prev_state;

#define RX            0
#define TXDELAY       1
#define TXPOWER       2



/*******************************LOGIC ****************************************************

SETUP
Setup turns off the transmitter power by setting ALC to approx -4 VDC
Setup turns off the ON light
Setup turns off the DELAY light


*********************************************************************************************/


///////////////////////////STRUCTURES & VARIABLES ///////////////////////////////////////
unsigned long   current_milliseconds;         // the current number of milliseconds since SEND output
int             delay_milliseconds;           // the desired delay
int             powersetting;                 // setting 0-1023 where 0 = maximum ALC (minimum output) and 
                                            // 1023 = minimum ALC (maximum output power)
int             screencount=0;                // avoids flickering screen

LiquidCrystal lcd(8, 9, 10, 11, 12, 13);  // The GLG board -- sequencer, rotator controller, arduino winkeyer etc

//LiquidCrystal lcd(13,12,11,10,9,8);  // when using the GAINESVILLE VENTILATOR Ashhar 1.0 Board
// This means that D8goes to RS, D9 to en, D10-D13 are the 4 bit parallel port info
// to the lcd display, which is connected exactly like both the UF venntilator
// and the uBitx Raduino display.






/////////////////// SET UP ROUTINE (EXECUTED ONCE ON STARTUP ////////////////////////////////////////////
void setup() {

  int i;  // general purpose counter
  int j;  // another 
  
  pinMode(ALCOUTPUT,    OUTPUT);
  pinMode(ONOUTPUT,   OUTPUT);
  pinMode(DELAYOUTPUT, OUTPUT);
  pinMode(RELAYOUTPUT, OUTPUT);
  pinMode(KILLOUTPUT,  OUTPUT);
  pinMode(SENDINPUT, INPUT_PULLUP);  // Getting the SENDINPUT set up
    
  analogReference(DEFAULT); // sets 5V as the top of reference

  digitalWrite(RELAYOUTPUT, LOW); // turn OFF the relay output to de-energize the relay
  analogWrite(ALCOUTPUT,200);  // set the ALC output to maximum silence output (gets inverted to -4V)
  digitalWrite(KILLOUTPUT,HIGH);  // turn off the output quickly
  digitalWrite(ONOUTPUT,LOW);  // turn off that LED
  digitalWrite(DELAYOUTPUT,LOW); // turn off the delay LED
// set all the states to RECEIVE
  tx_state = RX;
  prev_state = RX;

  
  cli();
  wdt_reset();
  /*
    WDTCSR configuration:
    WDIE = 1: Interrupt Enable
    WDE = 1 :Reset Enable
    See table for time-out variations:
    WDP3 = 0 :For 1000ms Time-out
    WDP2 = 1 :For 1000ms Time-out
    WDP1 = 1 :For 1000ms Time-out
    WDP0 = 0 :For 1000ms Time-out
  */
  // Enter Watchdog Configuration mode:
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  // Set Watchdog settings:
  WDTCSR = (1 << WDIE) | (1 << WDE) | (0 << WDP3) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0);
  // trying to set it for 2 seconds
  sei();  // R-enable the interrups

  Serial.begin(115200);   // this sets up the USB to give us diagnostic info
                          // at baud rate 115200 (fast)

  lcd.begin(16, 2);       // initialize the lcd display

  

  lcd.setCursor(0,1);     // ;set cursor to 0th column, 1st row (2nd line on ours)
  Lcd_Clear_Line();
  lcd.setCursor(0,1);
  
  lcd.setCursor(0,0);     // set the cursor at 0th column, 0th row
  Lcd_Clear_Line();
  lcd.setCursor(0,0);
  lcd.print(F("GNU GPL v3.0"));
  MyDelay(2000);
  lcd.setCursor(0,0);     // set the cursor at 0th column, 0th row
  Lcd_Clear_Line();
  lcd.setCursor(0,0);
  lcd.print(F("KX4Z Sequencer"));
  
  MyDelay(2000);    // Delay so this can be READ  


  lcd.setCursor(0,0);     // set the cursor at 0th column, 0th row
  Lcd_Clear_Line();
  lcd.setCursor(0,0);
  lcd.print(F("Version 0.2"));
 
  MyDelay(2000);    // Delay so this can be READ 



  // Need to turn off all the current systems and measure the offset voltages...

  Serial.print("PowerLevel:  ");
  Serial.println(analogRead(A0));
  /*
  lcd.setCursor(0,1);
  Lcd_Clear_Line();
  lcd.setCursor(0,1);
  sprintf(buffer,"Send:  ");
  lcd.print(buffer);
  lcd.print(analogRead(A0) );
  */
  
  MyDelay(1500);
   wdt_reset();
  MyDelay(1500);
  wdt_reset();
 
  
 
}  //end of SETUP

////////////////////////////READ POwer LEVEL////////////////////////////////
// Reads the potentiometer to find desired power level
// Sets the PWM to that level
// Does not affect digitalWrite(KILLOUTPUT,HIGH);  // turn off the output quickly
void readpowerlevel() {
int pwmlevel;  // 0 = maximum output
// read the 0-5V level of the power level adjustment potentiometer
powersetting = analogRead(PWRLEVELINPUT);   // reads 0-1023 for 0 - 5 VDC
pwmlevel = (1023-powersetting)/5;   // Will develop 4 volts when power setting for 0%
                                    // Will develop 0 volts when power setting for 100%
analogWrite(ALCOUTPUT,pwmlevel);    // sets the PWM

}

//////////////////////////////READ DELAY SETTING///////////////////////////////////
void readdelaysetting() {
// read the 0-5V setting of the delay setting potentiometer
int delayinputsetting;  

delayinputsetting = analogRead(DELAYINPUT);  // reads 0-1023 for 0-250 mSec delay
delay_milliseconds = delayinputsetting + MINIMUMDELAY  ;  // int from 0 to about 1000 mSec
return;
}

///////////////////////READ SEND/////////////////////////////
void readsend()
{

int reading1;
int reading2;
char buffer[15];
reading1 = digitalRead(SENDINPUT);

MyDelay(10);
reading2 = digitalRead(SENDINPUT);
// A simpleminded "debounce"
if(reading1 == reading2) {
    if((reading1==LOW) &&(tx_state==RX)) {
    tx_state=TXDELAY;
    }
    if ((reading1==HIGH) &&(tx_state!=RX)) {
    tx_state=RX;
    }

}
return;
}


///////////////////////////LOOP ROUTINE ///////////////////////////////////////////
// Meat of the program: executed over and over again
void loop() {
  char  buffer[20];
  // put your main code here, to run repeatedly:
   wdt_reset();      // Reset the watchdog timer -- it it doesn't get reset within 2 seconds,
                    // it will restart the software


    screencount++;
    if(screencount==10){
      lcd.setCursor(0,0);     // set the cursor at 0th column, 0th row
      Lcd_Clear_Line();
      lcd.setCursor(0,0);
      sprintf(buffer,"PWR%3d DLY%3d", (powersetting*10)/102,delay_milliseconds);
      
      lcd.print(buffer);
      lcd.setCursor(0,1);
      Lcd_Clear_Line();
      lcd.setCursor(6,1);
      sprintf(buffer,"STATE:%2d",tx_state);
      lcd.print(buffer);
      
      screencount = 0;
    }
                    
  // Check the state and see if it has moved to TXDELAY;

 ////////////////READ EACH OF THE CONTROLS!!!!!/////////////////////////////  
  readsend();
  readdelaysetting();
  readpowerlevel();
///////////////////////////////////////////////////////////////////////////
  
  // Which of 3 states are we now in?
  if(tx_state==TXDELAY){
    // keep the transmitter turned down to zero for an amount of time commanded by the delay knob, withe DELAY LED lit
    
    analogWrite(ALCOUTPUT,200);  // set the ALC output to maximum silence output (gets inverted to -4V)
    digitalWrite(RELAYOUTPUT,HIGH);  // turn on the RELAY to control anything needed there
    digitalWrite(ONOUTPUT,LOW);  // turn off that LED
    digitalWrite(DELAYOUTPUT,HIGH); // turn ON the delay LED
    // reset the timer
    wdt_reset();
    lcd.setCursor(0,1);
    sprintf(buffer,"DLY");
    lcd.print(buffer);
    lcd.setCursor(6,1);
    sprintf(buffer,"STATE:%2d",tx_state);
    lcd.print(buffer);
    
    delay(delay_milliseconds);  // Approx 0 - 1023 msec
    wdt_reset();
        // Then turn the transmitter on the power level commanded by the power level knob and turn on the ON LED
    readpowerlevel();  // go read the desired power setting and put it there
    // The RELAY has already been turned ON; it only gets turned OFF if RX state occurs
    digitalWrite(KILLOUTPUT,LOW);  // stop denying any output power at all
    digitalWrite(DELAYOUTPUT,LOW);  // turn off the delay LED
    digitalWrite(ONOUTPUT, HIGH);   // turn ON the ON LED
    tx_state=TXPOWER;  // set the STATE to full power
        wdt_reset();
    lcd.setCursor(0,1);
    sprintf(buffer,"TX");
    lcd.print(buffer);
    lcd.setCursor(6,1);
    sprintf(buffer,"STATE:%2d",tx_state);
    lcd.print(buffer);
    }

    
  if(tx_state==RX) {
    // keep the transmitter turned down to zero
     analogWrite(ALCOUTPUT,200);  // set the ALC output to maximum silence output (gets inverted to -4V)
     digitalWrite(KILLOUTPUT,HIGH);  // turn off the output quickly
     digitalWrite(RELAYOUTPUT,LOW);   // Turn off the relay
     digitalWrite(DELAYOUTPUT,LOW);  // turn off the delay LED
     digitalWrite(ONOUTPUT, LOW);   // turn ON the ON LED
     lcd.setCursor(0,1);
     sprintf(buffer,"RX");
     lcd.print(buffer);
  }
  if(tx_state==TXPOWER) {
    // keep the power level at the amount commanded by the power level knob
    readpowerlevel();  // this reads and sets (check and see if blips it)
    digitalWrite(KILLOUTPUT,LOW);  // quit denying any output at all
    digitalWrite(DELAYOUTPUT,LOW);  // turn off the delay LED
    digitalWrite(ONOUTPUT, HIGH);   // turn ON the ON LED
    lcd.setCursor(0,1);
    sprintf(buffer,"TX");
    lcd.print(buffer);
  }

  // Now go and read the power level knob and the delay knob and store those values for use.
   
}  // END OF LOOP

//////////////////////////supporting subroutines //////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////


///////////////////////////CLEAR LINE////////////////////////////////////////////////
void Lcd_Clear_Line()
{
  lcd.print(F("                "));  // should be exactly 16 spaces
}
//////////////////////////PRINT TO THE SCREEN ///////////////////////////////////////
void  lcd_display(char *s1,char *s2, int dtime)
{
  // dtime= milliseconds to delay
  lcd.setCursor(0,0);     // set the cursor at 0th column, 0th row
  // make sure the strings are null terminated after the 16th character.  1st char = *s1
  *(s1+15) = 0;
  *(s2+15) = 0;
  lcd.print(s1);
  lcd.setCursor(0,1);     // set cursor to 0th column, 1st row (2nd line on ours)
  lcd.print(s2);
  // call the delay function, while handling the watchdog
  wdt_reset();      // Reset the watchdog timer -- it it doesn't get reset within 2 seconds,
                    // it will restart the software
  MyDelay(dtime);
  wdt_reset();      // Reset the watchdog timer -- it it doesn't get reset within 2 seconds,
                    // it will restart the software
}
//////////////////////MICROSECOND DELAY NON BLOCKING /////////////////////////////////
void MyMicroSecondsDelay(int udelay)
{
  int microseconds;
  unsigned long current_microseconds, next_microseconds;
  
  current_microseconds = micros();
  next_microseconds = current_microseconds + (unsigned long) udelay;
    while(micros()<next_microseconds);
  return;
  
}

/////////////////////////////MILLISECOND DELAY NON BLOCKING///////////////////
void MyDelay(int msec)
{
  
  unsigned long local_current_milliseconds, next_milliseconds, intermediate_milliseconds;
  int x;
  int thousands;
  wdt_reset();
  local_current_milliseconds = millis();
  next_milliseconds = local_current_milliseconds + (unsigned long) msec;
  thousands = msec/1000;            // integer division
  if(thousands>=1) {
      for(x=1;x<= thousands; x++){
          intermediate_milliseconds = local_current_milliseconds + (unsigned long) (x*1000);
          while (millis() < intermediate_milliseconds);
          wdt_reset();
          }
    }
  // now finish out the remainder, which should be less than 1000 milliseconds
   while (millis()<next_milliseconds);
   wdt_reset();
  return;
}
