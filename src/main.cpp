#include <Arduino.h>
/****************************************
Example Sound Level Sketch for the 
Adafruit Microphone Amplifier
****************************************/
#include <Adafruit_NeoPixel.h>
#include <FastLED.h>
#define PIN 6
#define N_PIXELS  27
#define BRIGHTNESS 255   // 0-255, higher number is brighter.
#define LED_TYPE WS2812B
#define MIC_PIN   A4  // Microphone is attached to this analog pin
#define TOP       (N_PIXELS + 2) // Allow dot to go slightly off scale
#define N_PIXELS_HALF (N_PIXELS/2)
#define COLOR_ORDER GRB  // Try mixing up the letters (RGB, GBR, BRG, etc) for a whole new world of color combinations
#define LEFT 1
#define RIGHT 0

CRGB leds[N_PIXELS];

float
  greenOffset = 30,
  blueOffset = 150;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_PIXELS, PIN, NEO_GRB + NEO_KHZ800);


// Vu meter 4
const uint32_t Red = strip.Color(255, 0, 0);
const uint32_t Yellow = strip.Color(255, 255, 0);
const uint32_t Green = strip.Color(0, 255, 0);
const uint32_t Blue = strip.Color(0, 0, 255);
const uint32_t White = strip.Color(255, 255, 255);
const uint32_t Dark = strip.Color(0, 0, 0);
uint32_t colors[] = {Red, Yellow, Green, Blue, White};

// constants used here to set pin numbers:
const int buttonPin = 3;     // the number of the pushbutton pin

// Variables will change:

int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait, uint8_t dir) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      auto index = i;
      if (dir == LEFT) index = strip.numPixels() - i - 1;
      strip.setPixelColor(index, c);
      strip.show();
      delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

void vu(int height) {
   // Color pixels based on rainbow gradient
   for(auto i=0; i<N_PIXELS_HALF; i++) {
    if(i >= height) {              
      strip.setPixelColor(N_PIXELS_HALF-i-1,   0,   0, 0);
      strip.setPixelColor(N_PIXELS_HALF+i,   0,   0, 0);
    }
    else {
      uint32_t color = Wheel(map(i,0,N_PIXELS_HALF-1,(int)greenOffset, (int)blueOffset));
      strip.setPixelColor(N_PIXELS_HALF-i-1,color);
      strip.setPixelColor(N_PIXELS_HALF+i,color);
    }
   }
   strip.show();
}

void swipe() {
  for (auto i = 0; i < sizeof(colors)/sizeof(*colors); i++) {
    colorWipe(colors[i], 25, RIGHT);
  }
  for (auto i = 0; i < sizeof(colors)/sizeof(*colors); i++) {
    colorWipe(colors[i], 25, LEFT);
  }
}

const int sampleWindow = 100; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;

void setup() 
{
   Serial.begin(115200);
   delay( 2000 ); // power-up safety delay
   pinMode(buttonPin, INPUT);  
   //initialize the buttonPin as output
   digitalWrite(buttonPin, HIGH); 
   FastLED.addLeds<WS2812B, PIN, COLOR_ORDER>(leds, N_PIXELS).setCorrection( TypicalLEDStrip );
   FastLED.setBrightness(  BRIGHTNESS );
   LEDS.addLeds<LED_TYPE, PIN, COLOR_ORDER>(leds, N_PIXELS); 
   strip.begin();
   strip.show(); // Initialize all pixels to 'off'
}


void loop() 
{
   unsigned long startMillis= millis();  // Start of sample window
   unsigned int peakToPeak = 0;   // peak-to-peak level

   unsigned int signalMax = 0;
   unsigned int signalMin = 1024;

   // collect data for 50 mS
   while (millis() - startMillis < sampleWindow)
   {
      sample = analogRead(A4);
      if (sample < 1024)  // toss out spurious readings
      {
         if (sample > signalMax)
         {
            signalMax = sample;  // save just the max levels
         }
         else if (sample < signalMin)
         {
            signalMin = sample;  // save just the min levels
         }
      }
   }
   peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
   double volts = (peakToPeak * 5.0) / 1024;  // convert to volts

   Serial.println(volts);

   auto max = 3;
   auto min = 0;
   auto height = (int) ( N_PIXELS / (max - min) * volts);

  // read the pushbutton input pin:
  buttonState = digitalRead(buttonPin);
  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      // if the current state is HIGH then the button
      // wend from off to on:
      buttonPushCounter++;
      Serial.println("on");
      if(buttonPushCounter>=4) buttonPushCounter=1;
      Serial.print("number of button pushes:  ");
      Serial.println(buttonPushCounter);
      for(uint16_t i=0; i<strip.numPixels(); i++) strip.setPixelColor(i, Dark);
      strip.show(); // Initialize all pixels to 'off'
    } 
    else {
      // if the current state is LOW then the button
      // wend from on to off:
      Serial.println("off"); 
    }
  }
  // save the current state as the last state, 
  // for next time through the loop
  
  lastButtonState = buttonState;

  switch (buttonPushCounter) {  
  case 1:
  {
    vu(height);
    break;
  }
  case 2:
  {
    swipe();
    break;
  }
  default:
  {
    rainbow(10);
  }
  }
}