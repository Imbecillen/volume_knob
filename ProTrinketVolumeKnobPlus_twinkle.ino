

#include <ProTrinketHidCombo.h>

#define PIN_ENCODER_A      3
#define PIN_ENCODER_B      5
#define TRINKET_PINx       PIND
#define PIN_ENCODER_SWITCH 6 

static uint8_t enc_prev_pos   = 0;
static uint8_t enc_flags      = 0;
static char    sw_was_pressed = 0;


// NEOPIXEL_____________________


const int duration = 2000; //number of loops to run each animation for

#define NUMBEROFPIXELS 16 //Number of LEDs on the strip
#define PIXELPIN 9 //Pin where WS281X pixels are connected

#include <Adafruit_NeoPixel.h>
#include <NeoPixelPainter.h>


Adafruit_NeoPixel neopixels = Adafruit_NeoPixel(NUMBEROFPIXELS, PIXELPIN, NEO_GRB + NEO_KHZ800);

NeoPixelPainterCanvas pixelcanvas = NeoPixelPainterCanvas(&neopixels); //create canvas, linked to the neopixels (must be created before the brush)
NeoPixelPainterBrush pixelbrush = NeoPixelPainterBrush(&pixelcanvas); //crete brush, linked to the canvas to paint to


void setup()
{
  //Neopixel safe startup
  delay(3000);
  
  // set pins as input with internal pull-up resistors enabled
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_ENCODER_SWITCH, INPUT_PULLUP);
  
  TrinketHidCombo.begin(); // start the USB device engine and enumerate

  digitalWrite(PIN_ENCODER_SWITCH, HIGH);

  // get an initial reading on the encoder pins
  if (digitalRead(PIN_ENCODER_A) == LOW) {
    enc_prev_pos |= (1 << 0);
  }
  if (digitalRead(PIN_ENCODER_B) == LOW) {
    enc_prev_pos |= (1 << 1);
  }


// NEOPIXEL_____________________

randomSeed(analogRead(0)); //new random seed 
  pinMode(PIXELPIN, OUTPUT);

  neopixels.begin();

  Serial.begin(115200);
  Serial.println(" ");
  Serial.println("NeoPixel Painter Demo");

  //check if ram allocation of brushes and canvases was successful (painting will not work if unsuccessful, program should still run though)
  //this check is optional but helps to check if something does not work, especially on low ram chips like the Arduino Uno
  if (pixelcanvas.isvalid() == false) Serial.println("canvas allocation problem");
  else  Serial.println("canvas allocation ok");


  if (pixelbrush.isvalid() == false) Serial.println("brush allocation problem");
  else  Serial.println("brush allocation ok");

  
}

unsigned long loopcounter; //count the loops, switch to next animation after a while
bool initialized = false; //initialize the canvas & brushes in each loop when zero

void loop()
{
  int8_t enc_action = 0; // 1 or -1 if moved, sign is direction

  // note: for better performance, the code will use
  // direct port access techniques
  // http://www.arduino.cc/en/Reference/PortManipulation
  uint8_t enc_cur_pos = 0;
  // read in the encoder state first
  if (bit_is_clear(TRINKET_PINx, PIN_ENCODER_A)) {
    enc_cur_pos |= (1 << 0);
  }
  if (bit_is_clear(TRINKET_PINx, PIN_ENCODER_B)) {
    enc_cur_pos |= (1 << 1);
  }

  // if any rotation at all
  if (enc_cur_pos != enc_prev_pos)
  {
    if (enc_prev_pos == 0x00)
    {
      // this is the first edge
      if (enc_cur_pos == 0x01) {
        enc_flags |= (1 << 0);
      }
      else if (enc_cur_pos == 0x02) {
        enc_flags |= (1 << 1);
      }
    }

    if (enc_cur_pos == 0x03)
    {
      // this is when the encoder is in the middle of a "step"
      enc_flags |= (1 << 4);
    }
    else if (enc_cur_pos == 0x00)
    {
      // this is the final edge
      if (enc_prev_pos == 0x02) {
        enc_flags |= (1 << 2);
      }
      else if (enc_prev_pos == 0x01) {
        enc_flags |= (1 << 3);
      }

      // check the first and last edge
      // or maybe one edge is missing, if missing then require the middle state
      // this will reject bounces and false movements
      if (bit_is_set(enc_flags, 0) && (bit_is_set(enc_flags, 2) || bit_is_set(enc_flags, 4))) {
        enc_action = 1;
      }
      else if (bit_is_set(enc_flags, 2) && (bit_is_set(enc_flags, 0) || bit_is_set(enc_flags, 4))) {
        enc_action = 1;
      }
      else if (bit_is_set(enc_flags, 1) && (bit_is_set(enc_flags, 3) || bit_is_set(enc_flags, 4))) {
        enc_action = -1;
      }
      else if (bit_is_set(enc_flags, 3) && (bit_is_set(enc_flags, 1) || bit_is_set(enc_flags, 4))) {
        enc_action = -1;
      }

      enc_flags = 0; // reset for next time
    }
  }

  enc_prev_pos = enc_cur_pos;

  if (enc_action > 0) {
    TrinketHidCombo.pressMultimediaKey(MMKEY_VOL_UP);  // Clockwise, send multimedia volume up
  }
  else if (enc_action < 0) {
    TrinketHidCombo.pressMultimediaKey(MMKEY_VOL_DOWN); // Counterclockwise, is multimedia volume down
  }

  // remember that the switch is active low 
  if (bit_is_clear(TRINKET_PINx, PIN_ENCODER_SWITCH)) 
  {
    if (sw_was_pressed == 0) // only on initial press, so the keystroke is not repeated while the button is held down
    {
      TrinketHidCombo.pressMultimediaKey(MMKEY_PLAYPAUSE); // Encoder pushed down, toggle mute or not
      delay(5); // debounce delay
    }
    sw_was_pressed = 1;
  }
  else
  {
    if (sw_was_pressed != 0) {
      delay(5); // debounce delay
    }
    sw_was_pressed = 0;
  }

  TrinketHidCombo.poll(); // check if USB needs anything done, do every 10 ms or so



// NEOPIXEL_____________________


//---------------------
  //TWINKLY STARS
  //---------------------
  //brush set to random positions and painting a fading star
  for(loopcounter = 0; loopcounter<duration; loopcounter++) 
  {

    HSV brushcolor;

    if (initialized == false)
    {
      initialized = true;
      pixelbrush.setSpeed(0); //do not move automatically
      pixelbrush.setFadein(true); //fade in 
      pixelbrush.setFadeout(true); //and fade out
    }


    if (rand() % 100 == 0) //at a random interval, move the brush to paint a new pixel (brush only paints a new pixel once)
    {
      brushcolor.h = rand();
      brushcolor.s = random(80); //set low saturation, almost white
      brushcolor.v = random(200) + 40; //set random brightness
      pixelbrush.setColor(brushcolor);
      pixelbrush.moveTo(random(NUMBEROFPIXELS)); //move the brush to a new, random pixel
      pixelbrush.setFadeSpeed(random(5) + 10); //set random fade speed, minimum of 5
    }

    //add a background color by setting all pixels to a color (instead of clearing all pixels):
    int i;
    for (i = 0; i < NUMBEROFPIXELS; i++)
    {
      neopixels.setPixelColor(i, 1, 0, 6); //color in RGB: dark blue
    }


    pixelbrush.paint(); //paint the brush to the canvas 
    pixelcanvas.transfer(); //transfer (add) the canvas to the neopixels

    neopixels.show();
  }


  initialized = false; //reset the variable before moving to the next loop



}




