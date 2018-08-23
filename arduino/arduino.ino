// Libraries required
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "RunningMedian.h"
#include "FastLED.h" // Library required for addressable LEDs
#include <avr/interrupt.h> // Libraries for sleep mode
#include <avr/power.h> // Libraries for sleep mode
#include <avr/sleep.h> // Libraries for sleep mode
#include <avr/io.h> // Libraries for sleep mode

// Definitions
#define NUM_LEDS 72  // How many leds in your strip?
#define HIGH_BRIGHTNESS 50  // Set LEDS brightness
#define LOW_BRIGHTNESS 10  // Set LEDS brightness
#define DATA_PIN 10
#define MOSFET_GATE 11
#define MPU_POWER_PIN 9
#define PushB1  2 // Pin for push button 1  
#define Button_1_On  (!digitalRead(PushB1))
#define FRAMES_PER_SECOND  120
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))
#define SET_IDLE_MILLISECONDS 20000 // how many seconds at idle before moving to Fake_sleep?
#define SET_FAKE_SLEEP_MILLISECONDS 60000 // how many seconds at Fake_sleep before moving to Sleep?
#define SET_MAX_SPIRIT_LEVEL_MILLISECONDS 20000 // how many seconds at Fake_sleep before moving to Sleep?
#define SET_MAX_PLAY_SECONDS 60 // how many milliseconds vertical before moving to Sleep? (in case Stecchino is forgotten vertical)
#define START_PLAY_TRANSITION 500 //duration of animation when game starts (ms)
#define GAME_OVER_TRANSITION 1000 //duration of game over animation (ms)
#define SLEEP_TRANSITION 2000 //duration of animation to sleep (ms)
#define LOW_VCC 2700  // lower vcc value when checking battery level
#define HIGH_VCC 3350  // higher vcc value when checking battery level


// Variables used in getAngle() routine
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
enum AccelStatus {
    ACCEL_UNKNOWN,
    ACCEL_STRAIGHT,
    ACCEL_FALLEN
};
AccelStatus accel_status = ACCEL_UNKNOWN;
RunningMedian a_forwardRollingSample = RunningMedian(5);
RunningMedian a_sidewayRollingSample = RunningMedian(5);
RunningMedian a_verticalRollingSample = RunningMedian(5);
#define ACCELEROMETER_ORIENTATION 2     // 0, 1 or 2 to set the angle of the joystick
int a_forward_offset = 0,a_sideway_offset = 0,a_vertical_offset = 0;
enum POSITION_STECCHINO { NONE, POSITION_1,POSITION_2,POSITION_3,POSITION_4, POSITION_5, POSITION_6, COUNT };  // Used to detect position of buttons relative to Stecchino and user
const char *position_stecchino[COUNT] = { "None", "POSITION_1", "POSITION_2", "POSITION_3", "POSITION_4", "POSITION_5", "POSITION_6" };
// POSITION_1: Stecchino horizontal with buttons up (idle)
// POSITION_2: Stecchino horizontal with buttons down (force sleep)
// POSITION_3: Stecchino horizontal with long edge down (spirit level)
// POSITION_4: Stecchino horizontal with short edge down (opposite to spirit level)
// POSITION_5: Stecchino vertical with PCB up (normal game position = straight)
// POSITION_6: Stecchino vertical with PCB down (easy game position = straight)
uint8_t orientation = NONE;
float num_leds_per_second = 5; // how many LEDs are turned on every second when playing?

unsigned long start_time = 0, current_time = 0, elapsed_time = 0, record_time = 0, previous_record_time = 0;
int i = 0, Vcc = 0;
bool Ready_4_Change = false;

CRGB leds[NUM_LEDS];  // Define the array of leds

long readVcc();
void checkBatteryLed(int vcc);
void alloff();
void rainbow();
void confetti();
void sinelon();
void bpm();
void juggle();
void redGlitter();
float getAngle();
void sleepNow();
void pinInterrupt();

void setup() {

  Serial.begin(115200);
  while (!Serial)
      ;

  pinMode(PushB1,INPUT);
  digitalWrite(PushB1, HIGH);  // Configure built-in pullup resitor for push button 1

  pinMode(MOSFET_GATE,OUTPUT);
  digitalWrite(MOSFET_GATE, HIGH);


  pinMode(MPU_POWER_PIN,OUTPUT);
  digitalWrite(MPU_POWER_PIN, HIGH);

  delay(500);
  
  // LEDs strip
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);  
  FastLED.setBrightness(LOW_BRIGHTNESS);  
  
  // MPU
  Serial.print("Init MPU...");
  Wire.begin();
  accelgyro.initialize();  
  Serial.println("done");
  
  Vcc = int(readVcc());
  Serial.print("VCC=");
  Serial.print(Vcc);
  Serial.println("mV");
  checkBatteryLed(Vcc);
  delay(2000);
  alloff();
  
  start_time = millis();
}

// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = {confetti, sinelon, juggle, bpm, rainbow};
const char* SimplePatternNames[] = {"confetti","sinelon", "juggle", "bpm", "rainbow" };
uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns
uint8_t gFrameCount = 0; // Inc by 1 for each Frame of Transition, New/Changed connection(s) pattern


void setLedsOn(int count, int record) {
    for (int i = NUM_LEDS-1; i >= 0; i--) {
      if (i<=NUM_LEDS-count) {leds[i]=CRGB::Black;}
      //else {leds[i]=CRGB::Green;}
      else {leds[i]=CHSV(gHue, 255, 255);}
      if (i == NUM_LEDS-record) {leds[i]=CRGB::Red;}
    }

  // send the 'leds' array out to the actual LED strip
  FastLED.show();  
  // insert a delay to keep the framerate modest
  FastLED.delay(1000/FRAMES_PER_SECOND);       
  // do some periodic updates
  EVERY_N_MILLISECONDS(20) { gHue++; } // slowly cycle the "base color" through the rainbow
}

enum LedState {
    LED_GOING_TO_SLEEP,
    LED_IDLE,
    LED_START_PLAY,
    LED_WAHOO,
    LED_SPIRIT_LEVEL,
    LED_GAME_OVER,
    LED_EASY,
    LED_HARD,
    LED_OFF
};

void setLedPattern(LedState pattern)
{
  if (pattern == LED_GOING_TO_SLEEP) {
    digitalWrite(MOSFET_GATE, HIGH);
    FastLED.setBrightness(LOW_BRIGHTNESS); 
    for (int i = NUM_LEDS-1; i >= 0; i--) {
      leds[i] = CRGB::Blue;
    }
  }
  
  if (pattern == LED_IDLE) {
    digitalWrite(MOSFET_GATE, HIGH);
    FastLED.setBrightness(HIGH_BRIGHTNESS); 
    gPatterns[gCurrentPatternNumber]();
    //confetti();
    //rainbow();
  }
      
  if (pattern == LED_START_PLAY) {
    digitalWrite(MOSFET_GATE, HIGH);
    FastLED.setBrightness(LOW_BRIGHTNESS); 
    for (int i = NUM_LEDS-1; i >= 0; i--) {
      leds[i] = CRGB::Green;
    }
  }
   
   if (pattern == LED_WAHOO) {
    digitalWrite(MOSFET_GATE, HIGH);
    FastLED.setBrightness(HIGH_BRIGHTNESS); 
    redGlitter();
  } 
   
   if (pattern == LED_SPIRIT_LEVEL) {
    digitalWrite(MOSFET_GATE, HIGH);
    FastLED.setBrightness(HIGH_BRIGHTNESS); 
    sinelon();
  } 
    
  if (pattern == LED_GAME_OVER) {
    digitalWrite(MOSFET_GATE, HIGH);
    FastLED.setBrightness(LOW_BRIGHTNESS); 
    for (int i = NUM_LEDS-1; i >= 0; i--) {
      leds[i] = CRGB::Red;
    }
  }
  
  if (pattern == LED_EASY) {
    digitalWrite(MOSFET_GATE, HIGH);
    FastLED.setBrightness(HIGH_BRIGHTNESS); 
    for (int i = NUM_LEDS-1; i >= 0; i--) {
      leds[i] = CRGB::Green;
    }
  }

  if (pattern == LED_HARD) {
    digitalWrite(MOSFET_GATE, HIGH);
    FastLED.setBrightness(HIGH_BRIGHTNESS); 
    for (int i = NUM_LEDS-1; i >= 0; i--) {
      leds[i] = CRGB::White;
    }
  }
  
  if (pattern == LED_OFF) {
    for (int i = NUM_LEDS-1; i >= 0; i--) {
      leds[i].nscale8(230);
    }
    digitalWrite(MOSFET_GATE,LOW);
  }

  // send the 'leds' array out to the actual LED strip
  FastLED.show();  
  // insert a delay to keep the framerate modest
  FastLED.delay(1000/FRAMES_PER_SECOND);       
  // do some periodic updates
  EVERY_N_MILLISECONDS(20) { gHue++; } // slowly cycle the "base color" through the rainbow
}

void SPIRIT_LEVEL_LED(float angle) {
    digitalWrite(MOSFET_GATE, HIGH);
    FastLED.setBrightness(HIGH_BRIGHTNESS);
    int int_angle = int(angle);
    //int pos_led = map(int_angle,-90,90,1,NUM_LEDS);
    //int pos_led = map(int_angle,45,-45,1,NUM_LEDS);
    int pos_led = map(int_angle,-45,45,1,NUM_LEDS);
    int couleur_led = map(pos_led,0,NUM_LEDS,0,255);
    for (int i = NUM_LEDS-1; i >= 0; i--) {
      if (i == pos_led) {leds[i] = CHSV(couleur_led, 255, 255);}
      //if (i == pos_led) {leds[i] = CRGB::Blue;}
      else {leds[i] = CRGB::Black;}
    }
    // send the 'leds' array out to the actual LED strip
    FastLED.show();  
    // insert a delay to keep the framerate modest
    FastLED.delay(1000/FRAMES_PER_SECOND);  
    // do some periodic updates
  EVERY_N_MILLISECONDS(20) { gHue++; } // slowly cycle the "base color" through the rainbow 
}

void checkBatteryLed(int vcc) {  // bargraph showing battery level
    digitalWrite(MOSFET_GATE, HIGH);
    FastLED.setBrightness(LOW_BRIGHTNESS);
    if (vcc<LOW_VCC) {vcc = LOW_VCC;}
    if (vcc>HIGH_VCC) {vcc = HIGH_VCC;}
    int pos_led = map(vcc,LOW_VCC, HIGH_VCC,1,NUM_LEDS);
    //Serial.println(vcc);
    //Serial.println(LOW_VCC);
    //Serial.println(HIGH_VCC);
    //Serial.println(pos_led);

    
    for (int i = NUM_LEDS-1-1; i >= 0; i--) {
      if (i<=pos_led) {
        if (i<=5) {leds[i] = CRGB::Red;}
        else if (i>5 && i<=15) {leds[i] = CRGB::Orange;}
        else {leds[i] = CRGB::Green;}
        }
      else {leds[i] = CRGB::Black;}
    }
    // send the 'leds' array out to the actual LED strip
    FastLED.show();  
    // insert a delay to keep the framerate modest
    FastLED.delay(1000/FRAMES_PER_SECOND);  
    // do some periodic updates
    EVERY_N_MILLISECONDS(20)
    {
        // slowly cycle the "base color" through the rainbow 
        gHue++;
    }
}

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE(gPatterns);
  Serial.print("gCurrentPatternNumber ");
  Serial.println(gCurrentPatternNumber);
  delay(1000);
}

void rainbow() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow(leds, NUM_LEDS, gHue, 7);
}

void confetti() 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy(leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV(gHue + random8(64), 200, 255);
}

void sinelon()
{
    Serial.println("sinelon");
    delay(1000);
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy(leds, NUM_LEDS, 20);
  int pos = beatsin16(13,0,NUM_LEDS);
  Serial.print("pos");
  Serial.println(pos);
  delay(1000);
  leds[pos] += CHSV(gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8(BeatsPerMinute, 64, 255);
  for(int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy(leds, NUM_LEDS, 20);
  byte dothue = 0;
  for(int i = 0; i < 8; i++) {
    leds[beatsin16(i+7,0,NUM_LEDS)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

void redGlitter() {
  gFrameCount += 1;
  if (gFrameCount % 4 == 1) { // Slow down frame rate
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CHSV(HUE_RED, 0, random8() < 60 ? random8() : random8(64));
    }
  }
}
  
void alloff() {
  for (int i = NUM_LEDS-1; i >= 0; i--) {
    leds[i] = CRGB::Black;
    delay(10);
    FastLED.show();
  }
}

enum {
    Check_Battery,
    Wake_Up_Transition,
    Idle,
    Start_Play_Transition,
    Play,
    Wahoo,
    Game_Over_Transition,
    Spirit_Level,
    Magic_Wand,
    Sleep_Transition,
    Fake_Sleep
} state = Idle;

void loop() {
    //Serial.print("loop: "); Serial.println(state);
    float angle_2_horizon = getAngle();
    //Serial.print("angle: "); Serial.println(angle_2_horizon);
    switch (state) {

    case Check_Battery:
        Vcc = int(readVcc());
        Serial.print("VCC=");
        Serial.print(Vcc);
        Serial.println("mV");
        checkBatteryLed(Vcc);
        delay(2000);
        alloff();
        state = Idle;
        break;  
    
    case Wake_Up_Transition:
        break;
    
    case Idle:
        if (Button_1_On && Ready_4_Change) { 
            Serial.println("NEXT");
            delay(1000);
            //delay(20); // debouncing
            Ready_4_Change = false;
            nextPattern();  // change light patterns when button is pressed
            start_time = millis();  // restart counter to enjoy new pattern longer
        }
        if (!Button_1_On)
        {
            Ready_4_Change = true;
        }
        if (millis()-start_time>SET_IDLE_MILLISECONDS)
        {
            Serial.println("leave idle");
            delay(1000);
            state = Fake_Sleep;start_time = millis();
        }
        if (accel_status == ACCEL_STRAIGHT)
        {
            Serial.println("straight");
            delay(1000);
            state = Start_Play_Transition;start_time = millis();
        }
        if (orientation == POSITION_3)
        {
            Serial.println("spirit");
            delay(1000);
            state = Spirit_Level;start_time = millis();
        }
        if (orientation == POSITION_2)
        {
            Serial.println("pos2");
            delay(1000);
            state = Sleep_Transition;start_time = millis();
        }
        else {setLedPattern(LED_IDLE);}
        break;
    
    case Start_Play_Transition:
        alloff();
        state = Play;
        start_time = millis();
        previous_record_time = record_time;
        break;
    
    case Play:
        elapsed_time = (millis()-start_time)/1000;
        if (elapsed_time > record_time) 
        {
            // User has beaten record
            record_time = elapsed_time;
        }
        if (elapsed_time > previous_record_time && elapsed_time <= previous_record_time+1 && previous_record_time != 0) 
        {
            setLedPattern(LED_WAHOO);
        }
        if (elapsed_time > SET_MAX_PLAY_SECONDS) 
        {
            // Timeout, go to sleep
            state = Sleep_Transition;
            start_time = millis();
        }
        if (num_leds_per_second*int(elapsed_time) >= NUM_LEDS) 
        {
            // All LEDs are lit
            setLedPattern(LED_WAHOO);
        }
        else 
        {
            // Turn on the appropriate number of LEDs
            const int elapsed_time_x10 = (millis()-start_time)/100;
            const int leds_on = int(num_leds_per_second*elapsed_time_x10/10);
            setLedsOn(leds_on, int(num_leds_per_second*record_time));
        } 
        if (accel_status == ACCEL_FALLEN)
        {
            state = Game_Over_Transition;start_time = millis();
        }
        break;
    
    case Wahoo:
        break;
    
    case Game_Over_Transition:
        if (millis()-start_time>GAME_OVER_TRANSITION) {state = Idle;start_time = millis();}
        else {setLedPattern(LED_GAME_OVER);}
        break;

    case Spirit_Level:
        if (orientation == POSITION_1 || orientation == POSITION_2) {state = Idle;start_time = millis();}
        if (millis()-start_time>SET_MAX_SPIRIT_LEVEL_MILLISECONDS) {state = Fake_Sleep;start_time = millis();}
        SPIRIT_LEVEL_LED(angle_2_horizon);
        Serial.print("Angle to horizon:");
        Serial.println(angle_2_horizon);
        break;
    
    case Magic_Wand:
        break;
    
    case Fake_Sleep:
        if (millis()-start_time>SET_FAKE_SLEEP_MILLISECONDS) {state = Sleep_Transition;start_time = millis();}
        if (abs(angle_2_horizon)>15) {state = Idle;start_time = millis();}
        if (Button_1_On) {state = Idle;Ready_4_Change = false;start_time = millis();}
        else setLedPattern(LED_OFF);
        break; 
  
    case Sleep_Transition:
        if (millis()-start_time>SLEEP_TRANSITION) {sleepNow();}
        else setLedPattern(LED_GOING_TO_SLEEP);
        break;
    }
    FastLED.show();
}

float getAngle() {
  // Reads acceleration from MPU6050 to evaluate current state.
  // Tunables: 
  // Output values: accel_status = fallen or straight
  //                orientation = POSITION_1 to POSITION_6
  //                angle_2_horizon
  // Get accelerometer readings
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float angle_2_horizon = 0;

  // Convert to expected orientation - includes unit conversion to "cents of g" for MPU range set to 2g
  int a_forward = (ACCELEROMETER_ORIENTATION == 0?ax:(ACCELEROMETER_ORIENTATION == 1?ay:az))/164;
  int a_sideway = (ACCELEROMETER_ORIENTATION == 0?ay:(ACCELEROMETER_ORIENTATION == 1?az:ax))/164;
  int a_vertical = (ACCELEROMETER_ORIENTATION == 0?az:(ACCELEROMETER_ORIENTATION == 1?ax:ay))/164;
  
  // Update rolling average for smoothing
  a_forwardRollingSample.add(a_forward);
  a_sidewayRollingSample.add(a_sideway);
  a_verticalRollingSample.add(a_vertical);

  // Get median
  int a_forwardRollingSampleMedian = a_forwardRollingSample.getMedian()-a_forward_offset;
  int a_sidewayRollingSampleMedian = a_sidewayRollingSample.getMedian()-a_sideway_offset;
  int a_verticalRollingSampleMedian = a_verticalRollingSample.getMedian()-a_vertical_offset; 
  
  // Evaluate current state based on smoothed accelarations
  accel_status = ACCEL_UNKNOWN;
  if (abs(a_sidewayRollingSampleMedian) > abs(a_verticalRollingSampleMedian) ||
      abs(a_forwardRollingSampleMedian) > abs(a_verticalRollingSampleMedian))
  {
      accel_status = ACCEL_FALLEN;
  }
  if (abs(a_sidewayRollingSampleMedian) < abs(a_verticalRollingSampleMedian) &&
      abs(a_forwardRollingSampleMedian) < abs(a_verticalRollingSampleMedian))
  {
      accel_status = ACCEL_STRAIGHT;
  }

  if (a_verticalRollingSampleMedian >= 80 && abs(a_forwardRollingSampleMedian) <= 25 && abs(a_sidewayRollingSampleMedian) <= 25 && orientation != POSITION_6) {
    // Vertical with PCB down
    orientation = POSITION_6;
  } else if (a_forwardRollingSampleMedian >= 80 && abs(a_verticalRollingSampleMedian) <= 25 && abs(a_sidewayRollingSampleMedian) <= 25 && orientation != POSITION_2) {
    // Horizontal with buttons down
    orientation = POSITION_2;
  } else if (a_verticalRollingSampleMedian <= -80 && abs(a_forwardRollingSampleMedian) <= 25 && abs(a_sidewayRollingSampleMedian) <= 25 && orientation != POSITION_5) {
    // Vertical with PCB up
    orientation = POSITION_5;
  } else if (a_forwardRollingSampleMedian <= -80 && abs(a_verticalRollingSampleMedian) <= 25 && abs(a_sidewayRollingSampleMedian) <= 25 && orientation != POSITION_1) {
    // Horizontal with buttons up
    orientation = POSITION_1;
  } else if (a_sidewayRollingSampleMedian >= 80 && abs(a_verticalRollingSampleMedian) <= 25 && abs(a_forwardRollingSampleMedian) <= 25 && orientation != POSITION_3) {
    // Horizontal with long edge down
    orientation = POSITION_3;
  } else if (a_sidewayRollingSampleMedian <= -80 && abs(a_verticalRollingSampleMedian) <= 25 && abs(a_forwardRollingSampleMedian) <= 25 && orientation != POSITION_4) {
    // Horizontal with short edge down
    orientation = POSITION_4;
    if (num_leds_per_second < 5)
    {
        num_leds_per_second = 5;
        Serial.println("Easy mode");
        setLedPattern(LED_EASY);
        delay(2000);
    }
    else
    {
        num_leds_per_second = 2;
        Serial.println("Normal mode");
        setLedPattern(LED_HARD);
        delay(2000);
    }
  }

  angle_2_horizon = atan2(float(a_verticalRollingSampleMedian), float(max(abs(a_sidewayRollingSampleMedian), abs(a_forwardRollingSampleMedian))))*180/PI;

  /*
  // for debugging
  Serial.print("a_forward:");
  Serial.print(a_forwardRollingSampleMedian);
  Serial.print(" a_sideway:");
  Serial.print(a_sidewayRollingSampleMedian);
  Serial.print(" a_vertical:");
  Serial.print(a_verticalRollingSampleMedian);
  Serial.print(" - ");
  Serial.print(position_stecchino[orientation]);
  Serial.print(" - ");
  Serial.print(accel_status);
  Serial.print(" - ");
  Serial.println(angle_2_horizon);
  */
  
   return angle_2_horizon;
 }


void sleepNow(void)
{
    // Set pin 2 as interrupt and attach handler:
    attachInterrupt(0, pinInterrupt, LOW);
    delay(100);
    //
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    //
    // Set sleep enable (SE) bit:
    sleep_enable();
    //
    // Put the device to sleep:
    digitalWrite(MOSFET_GATE,LOW);   // turn LEDs off to indicate sleep
    digitalWrite(MPU_POWER_PIN,LOW);   // turn MPU off
    sleep_mode();
    //
    // Upon waking up, sketch continues from this point.
    sleep_disable();
    digitalWrite(MOSFET_GATE, HIGH);   // turn LED on to indicate awake 
    digitalWrite(MPU_POWER_PIN, HIGH);   // turn MPU off
    delay(100);

    // clear running median buffer
    a_forwardRollingSample.clear();
    a_sidewayRollingSample.clear();
    a_verticalRollingSample.clear();
    
    // retore MPU connection
    Wire.begin();
    accelgyro.initialize();  
    Serial.println("Restarting MPU... ");
    
    state = Check_Battery;
    start_time = millis();
}

void pinInterrupt(void)
{
    detachInterrupt(0);
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 =.1*1023*1000
  return result; // Vcc in millivolts
}

