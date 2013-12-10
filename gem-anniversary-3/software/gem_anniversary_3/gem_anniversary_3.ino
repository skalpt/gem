/* To do:
   =====
* Add auto-power off
* Move button into an interrupt so we can skip long screens
* Add serial sync function
* Add countdowns to important dates
* Add external time keeper
* Switch to ATTINY85
*/

#include <avr/sleep.h>
#include "Config.h"

//----------------------GRAPHIC-----------------------------//
//#include "data.c"
//==========================================================//

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

extern unsigned char HappyAnniversary[];

unsigned char fill_OLED=0x55;
unsigned char fill_string0[]="Dear Gem,";
unsigned char fill_string1[]="I am so lucky to";
unsigned char fill_string2[]="be able to share";
unsigned char fill_string3[]="my life with";
unsigned char fill_string4[]="you. It has been";
unsigned char fill_string5[]="an incredible";
unsigned char fill_string6[]="journey so far!";
unsigned char fill_string7[]="I | you. | Julz.";
extern unsigned char myFont[][8];

const int buttonPin = 2;
const int oledPowerPin = 3;

int buttonState = HIGH;
int curScreen = 0;
long lastPoll = 0;

// For debouncing:
int lastButtonState = HIGH;
long lastDebounceTime = 0;
long debounceDelay = 50;
boolean readyToSleep = false;

void setup() {
  
  pinMode(buttonPin, INPUT);
  pinMode(oledPowerPin, OUTPUT);
  
  i2c_init();

  SleepNow();
}

void loop() {
  
  int reading = digitalRead(buttonPin);
  
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
     
      if (buttonState == LOW) {
        switch (curScreen) {
          case 0:
            digitalWrite(oledPowerPin, 1);
            init_OLED();
            delay(10);
    
            AnniversaryPic();
            curScreen = 1;
            break;
          case 1:
            AnniversaryMsg();
            curScreen = 2;
            break;
          case 2:
            curScreen = 0;
            readyToSleep = true;
            break;
        }
      }
      
      if (buttonState == HIGH && readyToSleep) {
        readyToSleep = false;
        SleepNow();
      }
    }
  }
  
  lastButtonState = reading;
}

void AnniversaryPic() {

  clear_display();
  delay(50);

  sendcommand(0xa6);            //Set Normal Display

  sendcommand(0xae);		//display off
  sendcommand(0x20);            //Set Memory Addressing Mode
  sendcommand(0x00);            //Set Memory Addressing Mode ab Horizontal addressing mode

  clear_display();
  delay(50);

//==========================================================//
  for(int i=0;i<128*8;i++)     // show 128 * 64 picture
  {
    SendChar(HappyAnniversary[i]);
  }
//==========================================================//

  sendcommand(0xaf);
//  sendcommand(0xa7);          //Set Inverse Display
}

void AnniversaryMsg() {
  
  clear_display();
  delay(50);

  sendcommand(0x20);            //Set Memory Addressing Mode
  sendcommand(0x02);            //Set Memory Addressing Mode ab Page addressing mode(RESET)  

  sendcommand(0xa6);            //Set Normal Display (default)

  clear_display();
  delay(50);

  //---------------SHOW MESSAGE [LINES 1-8]-----------------//
  setXY(0,0);
  sendStr(fill_string0);
  setXY(1,0);
  sendStr(fill_string1);
  setXY(2,0);
  sendStr(fill_string2);
  setXY(3,0);
  sendStr(fill_string3);
  setXY(4,0);
  sendStr(fill_string4);
  setXY(5,0);
  sendStr(fill_string5);
  setXY(6,0);
  sendStr(fill_string6);
  setXY(7,0);
  sendStr(fill_string7);
//==========================================================//
}

void SleepNow() {

  clear_display();
  delay(50);

  sendcommand(0xa6);            //Set Normal Display

  sendcommand(0xae);		//display off
  sendcommand(0x20);            //Set Memory Addressing Mode
  sendcommand(0x00);            //Set Memory Addressing Mode ab Horizontal addressing mode

  clear_display();
  delay(50);

  digitalWrite(oledPowerPin, 0);

  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  attachInterrupt(0, wakeUpNow, LOW);

  sleep_mode();                        // System sleeps here
}

void wakeUpNow() {

  sleep_disable();
  detachInterrupt(0);
  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON
}

//==========================================================//
void print_a_char(unsigned char ascii=0)
{
  unsigned char i=0;
  for(i=0;i<8;i++)
  {
    SendChar(myFont[ascii-0x20][i]);
  }
}

//==========================================================//
void sendcommand(unsigned char com)
{
  i2c_OLED_send_cmd(com);  
}

//==========================================================//
void clear_display(void)
{
  unsigned char i,k;
  for(k=0;k<8;k++)
  {	
    setXY(k,0);    
    {
      for(i=0;i<128;i++)     //clear all COL
      {
        SendChar(0);         //clear all COL
        //delay(10);
      }
    }
  }
}

//==========================================================//
void SendChar(unsigned char data)
{
  i2c_OLED_send_byte(data);
}

//==========================================================//
void setXY(unsigned char row,unsigned char col)
{
  sendcommand(0xb0+row);                //set page address
  sendcommand(0x00+(8*col&0x0f));       //set low col address
  sendcommand(0x10+((8*col>>4)&0x0f));  //set high col address
}

//==========================================================//
void sendStr(unsigned char *string)
{
  unsigned char i=0;
  //setXY(0,0);    
  while(*string)
  {
    for(i=0;i<8;i++)
    {
      SendChar(myFont[*string-0x20][i]);

      // SendChar(*string);
      delay(10);
    }
    *string++;
  }
}

//==========================================================//
void init_OLED(void)
{
i2c_OLED_init();

}

void  i2c_OLED_init(void){
  i2c_OLED_send_cmd(0xae);    //display off
  i2c_OLED_send_cmd(0x2e);    //deactivate scrolling
  i2c_OLED_send_cmd(0xa4);          //SET All pixels OFF
//  i2c_OLED_send_cmd(0xa5);            //SET ALL pixels ON
  delay(50);
  i2c_OLED_send_cmd(0x20);            //Set Memory Addressing Mode
  i2c_OLED_send_cmd(0x02);            //Set Memory Addressing Mode to Page addressing mode(RESET)
//  i2c_OLED_send_cmd(0xa0);      //colum address 0 mapped to SEG0 (POR)*** wires at bottom
  i2c_OLED_send_cmd(0xa1);    //colum address 127 mapped to SEG0 (POR) ** wires at top of board
//  i2c_OLED_send_cmd(0xC0);            // Scan from Right to Left (POR)         *** wires at bottom
  i2c_OLED_send_cmd(0xC8);          // Scan from Left to Right               ** wires at top
  i2c_OLED_send_cmd(0xa6);            // Set WHITE chars on BLACK backround

//  i2c_OLED_send_cmd(0xa7);            // Set BLACK chars on WHITE backround
  i2c_OLED_send_cmd(0x81);            // 81 Setup CONTRAST CONTROL, following byte is the contrast Value
  i2c_OLED_send_cmd(0xff);            // af contrast value between 1 ( == dull) to 256 ( == bright)
  delay(20);
  i2c_OLED_send_cmd(0xaf);          //display on
  delay(20);
}


// ************************************************************************************************************
// I2C general functions
// ************************************************************************************************************

static uint32_t neutralizeTime = 0;
static int16_t  i2c_errors_count = 0;

#define I2C_SPEED 100000L     //100kHz normal mode, this value must be used for a genuine WMP

#define I2C_PULLUPS_ENABLE         PORTC |= 1<<4; PORTC |= 1<<5;   // PIN A4&A5 (SDA&SCL)
#define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);

void i2c_init(void) {
  #if defined(INTERNAL_I2C_PULLUPS)
    I2C_PULLUPS_ENABLE
  #else
    I2C_PULLUPS_DISABLE
  #endif
  TWSR = 0;                                    // no prescaler => prescaler = 1
  TWBR = ((F_CPU / I2C_SPEED) - 16) / 2;   // change the I2C clock rate
  TWCR = 1<<TWEN;                              // enable twi module, no interrupt
}

void i2c_rep_start(uint8_t address) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) ; // send REPEAT START condition
  waitTransmissionI2C();                       // wait until transmission completed
  TWDR = address;                              // send device address
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();                       // wail until transmission completed
}

void i2c_stop(void) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  //  while(TWCR & (1<<TWSTO));                // <- can produce a blocking state with some WMP clones
}

void i2c_write(uint8_t data ) {	
  TWDR = data;                                 // send data to the previously addressed device
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();
}

uint8_t i2c_read(uint8_t ack) {
  TWCR = (1<<TWINT) | (1<<TWEN) | (ack? (1<<TWEA) : 0);
  waitTransmissionI2C();
  uint8_t r = TWDR;
  if (!ack) i2c_stop();
  return r;
}

uint8_t i2c_readAck() {
  return i2c_read(1);
}

uint8_t i2c_readNak(void) {
  return i2c_read(0);
}

void waitTransmissionI2C() {
  uint16_t count = 255;
  while (!(TWCR & (1<<TWINT))) {
    count--;
    if (count==0) {              //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay
      i2c_errors_count++;
      break;
    }
  }
}

size_t i2c_read_to_buf(uint8_t add, void *buf, size_t size) {
  i2c_rep_start((add<<1) | 1);	// I2C read direction
  size_t bytes_read = 0;
  uint8_t *b = (uint8_t*)buf;
  while (size--) {
    /* acknowledge all but the final byte */
    *b++ = i2c_read(size > 0);
    /* TODO catch I2C errors here and abort */
    bytes_read++;
  }
  return bytes_read;
}

size_t i2c_read_reg_to_buf(uint8_t add, uint8_t reg, void *buf, size_t size) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  return i2c_read_to_buf(add, buf, size);
}

/* transform a series of bytes from big endian to little
   endian and vice versa. */
void swap_endianness(void *buf, size_t size) {
  /* we swap in-place, so we only have to
  * place _one_ element on a temporary tray
  */
  uint8_t tray;
  uint8_t *from;
  uint8_t *to;
  /* keep swapping until the pointers have assed each other */
  for (from = (uint8_t*)buf, to = &from[size-1]; from < to; from++, to--) {
    tray = *from;
    *from = *to;
    *to = tray;
  }
}

void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  i2c_write(val);        // value to write in register
  i2c_stop();
}


void i2c_OLED_send_cmd(uint8_t command) {
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate
  i2c_writeReg(OLED_address, 0x80, (uint8_t)command);
}

void i2c_OLED_send_byte(uint8_t val) {
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate
  i2c_writeReg(OLED_address, 0x40, (uint8_t)val);
}


