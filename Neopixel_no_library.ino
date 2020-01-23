#include <avr/io.h>

#include "TrinketHidCombo.h"  


#define neopixel PB0
#define encoderPinA PB3
#define encoderPinB PB2
#define encoderButton PB1 // active low since it shares build in LED, acts as a pull-down resistor
#define TRINKET_PINx PINB


#define LATCHSTATE PB4 //originally just 3, is this for a pin or something else?
int buttonState = HIGH, lastButtonState = HIGH;
long lastDebounceTime = 0, debounceDelay = 50;
int _position = 0, _positionExt = 0;
int8_t _oldState;

const int8_t encoderStates[] = {
  0, -1, 1, 0,
  1, 0, 0, -1,
  -1, 0, 0, 1,
  0, 1, -1, 0
};



// Change this to be at least as long as your pixel string (too long will work fine, just be a little slower)
#define PIXELS 16  // Number of pixels in the string


#define PIXEL_PORT  PORTB  // Port of the pin the pixels are connected to
#define PIXEL_DDR   DDRB // Port of the pin the pixels are connected to
#define PIXEL_BIT   neopixel   // Bit of the pin the pixels are connected to

// These are the timing constraints taken mostly from the WS2812 datasheets 
// These are chosen to be conservative and avoid problems rather than for maximum throughput 

#define T1H  900    // Width of a 1 bit in ns
#define T1L  600    // Width of a 1 bit in ns

#define T0H  400    // Width of a 0 bit in ns
#define T0L  900    // Width of a 0 bit in ns

// The reset gap can be 6000 ns, but depending on the LED strip it may have to be increased
// to values like 600000 ns. If it is too small, the pixels will show nothing most of the time.
#define RES 6000    // Width of the low gap between bits to cause a frame to latch

// Here are some convience defines for using nanoseconds specs to generate actual CPU delays

#define NS_PER_SEC (1000000000L)          // Note that this has to be SIGNED since we want to be able to check for negative values of derivatives

#define CYCLES_PER_SEC (F_CPU)

#define NS_PER_CYCLE ( NS_PER_SEC / CYCLES_PER_SEC )

#define NS_TO_CYCLES(n) ( (n) / NS_PER_CYCLE )

// Actually send a bit to the string. We must to drop to asm to enusre that the complier does
// not reorder things and make it so the delay happens in the wrong place.

inline void sendBit( bool bitVal ) {
  
    if (  bitVal ) {        // 0 bit
      
    asm volatile (
      "sbi %[port], %[bit] \n\t"        // Set the output bit
      ".rept %[onCycles] \n\t"                                // Execute NOPs to delay exactly the specified number of cycles
      "nop \n\t"
      ".endr \n\t"
      "cbi %[port], %[bit] \n\t"                              // Clear the output bit
      ".rept %[offCycles] \n\t"                               // Execute NOPs to delay exactly the specified number of cycles
      "nop \n\t"
      ".endr \n\t"
      ::
      [port]    "I" (_SFR_IO_ADDR(PIXEL_PORT)),
      [bit]   "I" (PIXEL_BIT),
      [onCycles]  "I" (NS_TO_CYCLES(T1H) - 2),    // 1-bit width less overhead  for the actual bit setting, note that this delay could be longer and everything would still work
      [offCycles]   "I" (NS_TO_CYCLES(T1L) - 2)     // Minimum interbit delay. Note that we probably don't need this at all since the loop overhead will be enough, but here for correctness

    );
                                  
    } else {          // 1 bit

    // **************************************************************************
    // This line is really the only tight goldilocks timing in the whole program!
    // **************************************************************************


    asm volatile (
      "sbi %[port], %[bit] \n\t"        // Set the output bit
      ".rept %[onCycles] \n\t"        // Now timing actually matters. The 0-bit must be long enough to be detected but not too long or it will be a 1-bit
      "nop \n\t"                                              // Execute NOPs to delay exactly the specified number of cycles
      ".endr \n\t"
      "cbi %[port], %[bit] \n\t"                              // Clear the output bit
      ".rept %[offCycles] \n\t"                               // Execute NOPs to delay exactly the specified number of cycles
      "nop \n\t"
      ".endr \n\t"
      ::
      [port]    "I" (_SFR_IO_ADDR(PIXEL_PORT)),
      [bit]   "I" (PIXEL_BIT),
      [onCycles]  "I" (NS_TO_CYCLES(T0H) - 2),
      [offCycles] "I" (NS_TO_CYCLES(T0L) - 2)

    );
      
    }
    

    
}  

  
inline void sendByte( unsigned char byte ) {
    
    for( unsigned char bit = 0 ; bit < 8 ; bit++ ) {
      
      sendBit( bitRead( byte , 7 ) );                // Neopixel wants bit in highest-to-lowest order
                                                     // so send highest bit (bit #7 in an 8-bit byte since they start at 0)
      byte <<= 1;                                    // and then shift left so bit 6 moves into 7, 5 moves into 6, etc
      
    }           
} 



inline void sendPixel( unsigned char r, unsigned char g , unsigned char b )  {  
  
  sendByte(g);          // Neopixel wants colors in green then red then blue order
  sendByte(r);
  sendByte(b);
  
}


// Just wait long enough without sending any bots to cause the pixels to latch and display the last sent frame

void show() {
  _delay_us( (RES / 1000UL) + 1);       // Round up since the delay must be _at_least_ this long (too short might not work, too long not a problem)
}




// Display a single color on the whole string

void showColor( unsigned char r , unsigned char g , unsigned char b ) {
  
  cli();  
  for( int p=0; p<PIXELS; p++ ) {
    sendPixel( r , g , b );
  }
  sei();
  show();
  
}





// RGB color, min 1, max 255
int rVal = 50; // 
int gVal = 50; // 
int bVal = 50; // 

int rDir = -1;
int gDir = -1;
int bDir = -1;

int brightness = 200; // maximum 255



void setup() {
    
 // bitSet( PIXEL_DDR , PIXEL_BIT ); //BitSet (writes a 1 to) a bit of a numeric variable. 

  DDRB |= (1 << neopixel); //replaces pinMode(PB0, OUTPUT);
  DDRB &= ~(1 << encoderPinA); //replaces pinMode(PB3, INPUT);
  DDRB &= ~(1 << encoderPinB); //replaces pinMode(PB2, INPUT);
  DDRB &= ~(1 << encoderButton); //replaces pinMode(PB1, INPUT); 
  //DDRB &= ~(1 << LATCHSTATE); //replaces pinMode(PB4, INPUT);
  

  PORTB |= (1 << neopixel); //replaces digitalWrite(PB0, HIGH);
  PORTB |= (1 << encoderPinA); //replaces digitalWrite(PB3, HIGH);
  PORTB |= (1 << encoderPinB); //replaces digitalWrite(PB2, HIGH);
  PORTB &= ~(1 << encoderButton); //replaces digitalWrite(PB1, LOW); // active low since it shares build in LED, acts as a pull-down resistor
  //PORTB &= ~(1 << LATCHSTATE); //replaces digitalWrite(PB4, LOW);

  _oldState = 3; 
  TrinketHidCombo.begin();
  
}


void loop() {


showColor(rVal, gVal, bVal);

  rVal = rVal + rDir;
  gVal = gVal + gDir;
  bVal = bVal + bDir;


//originally 255, 
   if (rVal >= brightness || rVal <= 0) {
    rDir = rDir * -1;
  }

  if (gVal >= brightness || gVal <= 0) {
    gDir = gDir * -1;
  }

  if (bVal >= brightness || bVal <= 0) {
    bDir = bDir * -1;
  }

  // sets pulsating speed
  delay(20);
  
  return;



 static int pos = 0;
  tick();
  int newPos = getPosition();
  if (pos != newPos) {
    if (newPos < pos) {
      TrinketHidCombo.pressMultimediaKey(MMKEY_VOL_DOWN);
    }
    else if (newPos > pos){
      TrinketHidCombo.pressMultimediaKey(MMKEY_VOL_UP);
    }
    pos = newPos;
  }
  int reading = digitalRead(encoderButton);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        TrinketHidCombo.pressMultimediaKey(MMKEY_MUTE);
      } else if (buttonState == HIGH){

      }
    } 
  }
  lastButtonState = reading;
  TrinketHidCombo.poll();
  
}




int  getPosition() {
  return _positionExt;
}

void setPosition(int newPosition) {
  _position = ((newPosition<<2) | (_position & 0x03));
  _positionExt = newPosition;
}

void tick(void) {
  int sig1 = digitalRead(encoderPinA);
  int sig2 = digitalRead(encoderPinB);
  int8_t thisState = sig1 | (sig2 << 1);
  if (_oldState != thisState) {
    _position += encoderStates[thisState | (_oldState<<2)];
    if (thisState == LATCHSTATE)
      _positionExt = _position >> 2;
    _oldState = thisState;
  }
} 


