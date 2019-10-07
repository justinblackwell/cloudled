#include <MPU6050.h>
#include <Wire.h>
#include <FastLED.h>

FASTLED_USING_NAMESPACE

#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define NUM_LEDS    18
#define NUM_STRIPS 8
#define LEDS_PER_STRIP 18 
#define BRIGHTNESS          10
#define FRAMES_PER_SECOND  120
#define MIN_LIT 0

CRGB leds[NUM_STRIPS][NUM_LEDS];

uint8_t gHue = 0; // rotating "base color" used by many of the patterns
fract8 prob = 50; // prob of glitter 
MPU6050 mpu; // gyro device
int coords[8] = {
  9,9,9,9,9,9,9,9 // for "tilting" leds
};
uint8_t ledsToLight = NUM_LEDS;

void setup() {
  
  delay(3000); 
  Serial.begin(115200);
  
  // init all strips on their respective data pins  
  FastLED.addLeds<LED_TYPE,10, COLOR_ORDER>(leds[0], NUM_LEDS);
  FastLED.addLeds<LED_TYPE, 8, COLOR_ORDER>(leds[1], NUM_LEDS);
  FastLED.addLeds<LED_TYPE, 4, COLOR_ORDER>(leds[2], NUM_LEDS);
  FastLED.addLeds<LED_TYPE, 9, COLOR_ORDER>(leds[3], NUM_LEDS);
  FastLED.addLeds<LED_TYPE, 7, COLOR_ORDER>(leds[4], NUM_LEDS);
  FastLED.addLeds<LED_TYPE, 3, COLOR_ORDER>(leds[5], NUM_LEDS);
  FastLED.addLeds<LED_TYPE, 6, COLOR_ORDER>(leds[6], NUM_LEDS);
  FastLED.addLeds<LED_TYPE, 5, COLOR_ORDER>(leds[7], NUM_LEDS);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);

  // power mgmt
  set_max_power_in_volts_and_milliamps(5, 500);

  Serial.println("Initialize MPU6050");

  // attempt x times before recovering
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
}

void loop() {

  for(int x = 0; x < NUM_STRIPS ; x++){
    
//    FastLED.clear();

//    rainbow(x);
//    sinelon(x);
//    bpm(x);
//    juggle(x);
//    fadeToBlackBy( leds[x], NUM_LEDS, 20); addGlitter(60, x);
//    fadeToBlackBy( leds[x], NUM_LEDS, 20); addGlitter(prob, x);
//    rainbowWithGlitter(x);
//    confetti(x);

    ledsToLight = coords[x];
  
    fadeToBlackBy( leds[x], NUM_LEDS, 50);

//    fill_solid(leds[x], NUM_LEDS, CRGB::LightSkyBlue);
//    leds[x][coords[x]] = CRGB::Red;
//    bpm(x);
    flame(x, 8, coords[x]);
//      fill_rainbow(leds[x], coords[x], gHue, 7);
//    fill_gradient_RGB(leds[x], 0, CRGB::LightSkyBlue, coords[x], CRGB::Azure);
//      fill_gradient_RGB(leds[x], coords[x], CRGB::DarkBlue, NUM_LEDS, CRGB::Orange);
//    addGlitter(map(coords[x], 0, NUM_LEDS, 15, 95), x);
//    confetti(x);


//    fill_rainbow(leds[x], x+1, gHue, 7); // test pattern - identify strips
  }
  
  EVERY_N_MILLISECONDS( 50 ) { gHue+= 1; } // slowly cycle the "base color" through the rainbow
  EVERY_N_MILLISECONDS( 100 ) { calculateCoords(); } // calculate tilt angles
  
  FastLED.show();
  FastLED.delay(1000/FRAMES_PER_SECOND);

//  Serial.print("max_brightness_for_power_mW"); Serial.println( calculate_max_brightness_for_power_mW(leds[0], NUM_LEDS, 5, 500));
}


void calculateCoords(){
  Vector normAccel = mpu.readNormalizeAccel(); // Read normalized values 
  
//  Vector normAccel = mpu.readNormalizeGyro();
//  // Calculate Pitch, Roll and Yaw
//  pitch = pitch + norm.YAxis * timeStep;
//  roll = roll + norm.XAxis * timeStep;
//  yaw = yaw + norm.ZAxis * timeStep;

  // Calculate Pitch & Roll
  coords[0] = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
  coords[2] = (atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  
  coords[4] = -1 * coords[0];
  coords[6] = -1 * coords[2];
  
  coords[1] = (coords[0] + coords[2]) / 2;
  coords[5] = (coords[4] + coords[6]) / 2;
  
  coords[3] = (coords[2] + coords[4]) / 2;
  coords[7] = (coords[0] + coords[6]) / 2;

  for(uint8_t x = 0 ; x < NUM_STRIPS ; x++){
     coords[x] = map(coords[x], -90, 90, MIN_LIT, NUM_LEDS);
  }
}

//void ease() {
//
//  static uint8_t easeOutVal = 0;
//  static uint8_t easeInVal  = 0;
//  static uint8_t lerpVal    = 0;
//
//  easeOutVal = ease8InOutCubic(easeInVal);                     // Start with easeInVal at 0 and then go to 255 for the full easing.
//  easeInVal++;
//
//  lerpVal = lerp8by8(0, NUM_LEDS, easeOutVal);                // Map it to the number of LED's you have.
//
//  leds[lerpVal] = CRGB::Red;
//  fadeToBlackBy(leds, NUM_LEDS, 16);                          // 8 bit, 1 = slow fade, 255 = fast fade
//  
//} // ease()


void sinelon(int strip)
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds[strip], NUM_LEDS, 20);
  int pos = beatsin16( 13, 0, NUM_LEDS-1 );
  leds[strip][pos] += CHSV( gHue, 255, 192);
}

void rainbow(int strip) 
{
  fill_rainbow( leds[strip], NUM_LEDS, gHue, 7);
}

void rainbowWithGlitter(int strip) 
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow(strip);
  addGlitter(80, strip);
}

void addGlitter( fract8 chanceOfGlitter, int strip) 
{
  if( random8() < chanceOfGlitter) {
    leds[strip][ random16(ledsToLight) ] += CRGB::White;
  }
}

void confetti(int strip) 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds[strip], NUM_LEDS, 10);
  int pos = random16(ledsToLight);
  leds[strip][pos] += CHSV( gHue + random8(64), 200, 255);
}

void flame(uint8_t strip, uint8_t startPosition, uint8_t flameHeight){
  uint8_t beat = cubicwave8(32);
  CRGBPalette16 palette = HeatColors_p;
  for( int i = startPosition; i <= flameHeight && i < NUM_LEDS; i++) { 
    leds[strip][i] = ColorFromPalette(palette, i*10, beat+(i*10));
//    leds[strip][max(startPosition - i,0)] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
}

void bpm(int strip)
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 20;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[strip][i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
    
  }
}

void juggle(int strip) {
  // eight colored dots, weaving in and out of sync with each other
   fadeToBlackBy( leds[strip], NUM_LEDS, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[strip][beatsin16( i+7, 0, NUM_LEDS-1 )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}
