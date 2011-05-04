/*
 * Mercury Retrograde
 * Version 002
 *
 * Copyright (C) 2009 Sean Voisen <http://voisen.org>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "mma7260q.h" // Accelerometer library
 
#define DEBUG false // Set to true for debugging, false for off

#if DEBUG
#define DEBUG_BAUD 9600
#endif

#define QUIRKS_MODE false // Set to true for quirks, false for off
 
#define MAX_MILLIS 0xFFFFFFFF

/******************************************
 * Delay Defines
 *****************************************/
#define MULTIPLEX_HIGH_DELAY 800 // microseconds (how long to keep nixie ON)
#define MULTIPLEX_LOW_DELAY 500 // microseconds (must be < 1000 to avoid missing interrupt!!)
#define FLASH_DELAY 500 // milliseconds
#define SCROLL_SET_DELAY 300 // milliseconds
#define BEEP_DELAY 500 // milliseconds
#define ALARM_SHOW_DELAY 2000 // milliseconds

/******************************************
 * Digital I/O Pin Defines
 *****************************************/
#define VAL0_PIN 13 // LSB of numeric output
#define VAL1_PIN 12
#define VAL2_PIN 11
#define VAL3_PIN 10 // MSB of numeric output
#define EXTERNAL_POWER_PIN 9 // Monitor power supply
#define PIEZO_PIN 8 // The buzzer
#define SECS_1_PIN 7 // Seconds, 1's
#define SECS_10_PIN 6 // Seconds, 10's
#define MINS_1_PIN 5 // Minutes, 1's
#define MINS_10_PIN 4 // Minutes, 10's
#define HOUR_1_PIN 3 // Hours, 1's
#define HOUR_10_PIN 2 // Hours, 10's
#define ALARM_INDICATOR_PIN 0 // LED for alarm on/off

/******************************************
 * Mode Defines
 *****************************************/
#define RUN 0
#define SET_TIME 1
#define SET_ALARM 2
#define CATHODE_POISON 3

/******************************************
 * Accelerometer Defines
 *****************************************/
#define MAX_SCROLL_VELOCITY 31000
#define MIN_SCROLL_VELOCITY -31000
#define MAX_ANGLE 75
#define DEFAULT_X_ZERO_ANGLE 0 // 17
#define DEFAULT_Y_ZERO_ANGLE 0
#define ANGLE_THRESHOLD 10
#define ANGLE_EXPONENT 2.395415 // log(MAX_SCROLL_VELOCITY) / log(MAX_ANGLE)

#define X_PIN 0
#define Y_PIN 1
#define Z_PIN 2

#define MIN_X 0
#define MAX_X 582
#define MIN_Y 108
#define MAX_Y 562
#define MIN_Z 22
#define MAX_Z 678

#define X_SHAKE_THRESHOLD 25
#define Y_SHAKE_THRESHOLD 25
#define Z_SHAKE_THRESHOLD 25

#define Z_SHAKE_MAX 500
#define Z_SHAKE_MIN 300
#define Y_SHAKE_MAX 500
#define Y_SHAKE_MIN 300
#define X_SHAKE_MAX 500
#define X_SHAKE_MIN 300

#define SHAKE_PHASE_1 0
#define SHAKE_PHASE_2 1
#define SHAKE_PHASE_3 2
#define SHAKE_PHASE_4 3

#define SHAKE_TIMEOUT 500

#define POISON_DURATION 3000

/******************************************
 * Music-playing Defines
 *****************************************/
// Defines for the frequency of the notes (.5 x freq of mid C)
#define AN    220     // 440 Hz
#define AS    233     // 466 Hz
#define BN    247     // 493 Hz
#define CN    261     // 523 Hz
#define CS    277     // 554 Hz
#define DN    294     // 588 Hz
#define DS    311     // 622 Hz
#define EN    330     // 658 Hz
#define FN    349     // 698 Hz
#define FS    370     // 740 Hz
#define GN    392     // 784 Hz
#define GS    415     // 830 Hz
#define REST  0       // Silence

// Defines for the duration of the notes (in ms)
#define WH    1024
#define H      512
#define DQ     448
#define Q      256
#define QT     170
#define DE     192
#define E      128
#define ET      85

/******************************************
 * ALARM DEFINES
 *****************************************/
#define BEEP_MAX 50 // Desired beeps * 2

/******************************************
 * QUIRKS MODE DEFINES
 *****************************************/
#if QUIRKS_MODE
#define QUIRK_DELAY_FACTOR 10000
#define QUIRK_OFFSET_CHANCE 25 // Percent
#endif

/******************************************
 * Timekeeping Variables
 *****************************************/
unsigned long currentMillis = 0;
unsigned long lastMillis = 0;
unsigned long tt = 0;
int seconds = 0;
int minutes = 0;
int hours = 12;
unsigned int hourDivisor = 24; // 24 for military time, 12 for standard

/******************************************
 * Setting Variables
 *****************************************/
unsigned int mode = RUN;
unsigned long lastFlashMillis = 0;
boolean flashOn = true;
int scrollVelocity = 0; // Used to scroll through time during setting, ms/execution
long st = 0;
int scrollSeconds = 0;
int scrollMinutes = 0;
int scrollHours = 0;
unsigned long lastScrollMillis = 0;

/******************************************
 * Alarm Variables
 *****************************************/
int alarmSeconds = 0;
int alarmMinutes = 0;
int alarmHours = 12;
boolean alarmEnabled = false;
boolean alarmOn = false;
boolean beepOn = false;
int beepMax = 0;
int beepCount = 0;
unsigned long lastBeep = 0;

/******************************************
 * Misc Variables
 *****************************************/
int lastPowerRead = LOW;

/******************************************
 * Quirks Mode Variables
 *****************************************/
#if QUIRKS_MODE
unsigned long lastQuirkMillis = 0;
int quirkVal = 0;
#endif

/******************************************
 * Accelerometer
 *****************************************/
Mma7260q accelerometer = Mma7260q(X_PIN, Y_PIN, Z_PIN);
int zShakePhase = SHAKE_PHASE_1;
int yShakePhase = SHAKE_PHASE_1;
int xShakePhase = SHAKE_PHASE_1;
unsigned long lastShakeMillis = 0;

/******************************************
 * Tunes
 *****************************************/
#define WESTMINSTER 1
int westminsterNotes[] = {CN*2, EN*2, DN*2, GN, REST, GN, DN*2, EN*2, CN*2};
int westminsterDurations[] = {H, H, H, WH, H, H, H, H, WH};
int westminsterNumNotes = 9;

#define TONE_UP 2
int toneUpNotes[] = {AN*2, CS*2, EN*2};
int toneUpDurations[] = {QT, QT, QT};
int toneUpNumNotes = 3;

#define TONE_UP2 3
int toneUp2Notes[] = {DN, FS, AN*2};
int toneUp2Durations[] = {QT, QT, QT};
int toneUp2NumNotes = 3;

/******************************************
 * Function Declarations
 *****************************************/
void outputValue( unsigned int );
void outputTime( int, int, int );
void flashNixie( unsigned int, unsigned int );
unsigned long updateTime();
void blankDisplay();
boolean updateFlash();
void scrollTime( int &, int &, int & );
void updateScrollVelocity( int );
void setDisplayPinsMode( boolean );
void checkExternalPower();
void playTune( int, boolean );
void playMelody( int[], int, int[], boolean );
void outputTone( int, int );
boolean detectShake( int );
void setAlarmEnabled( boolean );
void startAlarm( int );
void stopAlarm();
void handleAlarm();

/**
 * Called once on startup.
 */
void setup()
{
#if DEBUG
Serial.begin( DEBUG_BAUD );
Serial.println( "Mercury Retrograde" );
Serial.println( "Version 001" );
#endif

#if QUIRKS_MODE
  // Seed random number generator
  randomSeed( analogRead( 5 ) );
#endif
  
  // Start monitoring for external power
  pinMode( EXTERNAL_POWER_PIN, INPUT );
  setDisplayPinsMode( INPUT );
  checkExternalPower();
  
  accelerometer.autoZeroCalibration( max(0, analogRead( X_PIN ) - X_SHAKE_THRESHOLD - 10), max(0, analogRead( Y_PIN ) - Y_SHAKE_THRESHOLD - 10), max(0, analogRead( Z_PIN ) - Z_SHAKE_THRESHOLD - 10) );
  accelerometer.autoZeroCalibration( min(1023, analogRead( X_PIN ) + X_SHAKE_THRESHOLD + 10), min(1023, analogRead( Y_PIN ) + Y_SHAKE_THRESHOLD + 10), min(1023, analogRead( Z_PIN ) + Z_SHAKE_THRESHOLD + 10) );
  
  // Setup accelerometer
  //accelerometer.autoZeroCalibration(MIN_X,MIN_Y,MIN_Z);
  //accelerometer.autoZeroCalibration(MAX_X,MAX_Y,MAX_Z);
  
  // Setup alarm
  pinMode( PIEZO_PIN, OUTPUT );

  // Turn off all display outputs initially
  if( lastPowerRead == HIGH )
    blankDisplay();
}

/**
 * Called repeatedly.
 */
void loop()
{ 
  checkExternalPower(); // Always check
  
  accelerometer.autoZeroCalibration(analogRead( X_PIN ), analogRead( Y_PIN ), analogRead( Z_PIN ));
  
  switch( mode )
  {
    case RUN:
      updateTime(); // ALWAYS call this!
      
      // Go no further if power is off
      if( lastPowerRead == LOW )
        return;
        
      outputTime( hours, minutes, seconds );
        
      // Check for input to switch modes
      if( detectShake( X_PIN ) ) {
        if( alarmOn ) {
          stopAlarm();
        }
        else {
          mode = SET_TIME;
          scrollHours = hours;
          scrollMinutes = minutes;
          scrollSeconds = seconds;
          lastScrollMillis = millis();
          playTune( TONE_UP2, false );
          return;
        }
      }
      
      if( detectShake( Y_PIN ) ) {
        if( alarmOn ) {
          stopAlarm();
        }
        else {
          mode = SET_ALARM;
          scrollHours = alarmHours;
          scrollMinutes = alarmMinutes;
          scrollSeconds = alarmSeconds;
          lastScrollMillis = millis();
          playTune( TONE_UP, false );
          return;
        }
      }
      
      if( detectShake( Z_PIN ) ) {
        if( alarmOn ) {
          stopAlarm();
        }
        else {
          setAlarmEnabled( !alarmEnabled );
        }
      }
      
      // ALARM handling below here
      if( !alarmEnabled )
        return;
        
      if( !alarmOn && hours == alarmHours && minutes == alarmMinutes && seconds == alarmSeconds ) {
        // Sound the alarm
        startAlarm( BEEP_MAX );
      }
      else if( alarmOn ) {
        handleAlarm();
      }
      break;
      
    case SET_TIME:
      // Check for input to switch modes
      if( detectShake( X_PIN ) ) {
        mode = RUN;
        playTune( TONE_UP2, true );
        return;
      }
      
      updateScrollVelocity( X_PIN );          
      
      // Only flash when not scrolling through time
      if( scrollVelocity != 0 ) {
        scrollTime( scrollHours, scrollMinutes, scrollSeconds ); // Use this to update time instead, clock is no longer running
        outputTime( scrollHours, scrollMinutes, scrollSeconds );
        lastScrollMillis = millis();
      }
      else {
        updateFlash() ? outputTime( scrollHours, scrollMinutes, scrollSeconds ) : blankDisplay();
        if( millis() - lastScrollMillis >= SCROLL_SET_DELAY ) {
          hours = scrollHours;
          minutes = scrollMinutes;
          seconds = scrollSeconds;
        }
      }
      break;
      
    case SET_ALARM:
      updateTime(); // Keep the clock running
      
      // Check for input to switch modes
      if( detectShake( Y_PIN ) ) {
        mode = RUN;
        digitalWrite( ALARM_INDICATOR_PIN, alarmEnabled );
        playTune( TONE_UP, true );
        unsigned long modeSwitchMillis = updateTime();
        // Show alarm time for a few seconds before switching over
        while( updateTime() - modeSwitchMillis < ALARM_SHOW_DELAY ) {
          outputTime( alarmHours, alarmMinutes, alarmSeconds );
        }
        return;
      }
      
      updateScrollVelocity( Y_PIN );
      
      // Always flash alarm indicator
      digitalWrite( ALARM_INDICATOR_PIN, updateFlash() );
      
      // Only flash numbers when not scrolling
      if( scrollVelocity != 0 ) {
        scrollTime( scrollHours, scrollMinutes, scrollSeconds ); // Scroll through alarm setting
        outputTime( scrollHours, scrollMinutes, scrollSeconds );
        lastScrollMillis = millis();
      }
      else {
        // Flash was already updated with alarm indicator
        flashOn ? outputTime( scrollHours, scrollMinutes, scrollSeconds ) : blankDisplay();
        if( millis() - lastScrollMillis >= SCROLL_SET_DELAY ) {
          alarmHours = scrollHours;
          alarmMinutes = scrollMinutes;
          alarmSeconds = scrollSeconds;
        }
      }
      break;
      
    case CATHODE_POISON:
      unsigned long startPoison = millis();
      randomSeed( analogRead( 5 ) );
      long h1, h0, m1, m0, s1, s0;
      while( millis() - startPoison < POISON_DURATION ) {
        h1 = random(10);
        h0 = random(10);
        m1 = random(10);
        m0 = random(10);
        s1 = random(10);
        s0 = random(10);
        updateTime(); // Keep the clock running
        unsigned long startPoisonCycle = millis();
        while( millis() - startPoisonCycle < 200 ) {
          flashNixie( h1, HOUR_10_PIN );
          flashNixie( h0, HOUR_1_PIN );
          flashNixie( m1, MINS_10_PIN );
          flashNixie( m0, MINS_1_PIN );
          flashNixie( s1, SECS_10_PIN );
          flashNixie( s0, SECS_1_PIN );
        }
      }
      mode = RUN;
      break;
  }
}

/**
 * Outputs a value to the 74141 BCD encoder.
 */
void outputValue( unsigned int value )
{
  digitalWrite( VAL0_PIN, value & 0x01 ); // LSB
  digitalWrite( VAL1_PIN, (value & 0x02) >> 1 );
  digitalWrite( VAL2_PIN, (value & 0x04) >> 2 );
  digitalWrite( VAL3_PIN, (value & 0x08) >> 3 ); // MSB
}

/**
 * Outputs time to all nixies.
 * Call this cyclically.
 */
void outputTime( int outHours, int outMinutes, int outSeconds )
{
  flashNixie( outHours / 10, HOUR_10_PIN );
  flashNixie( outHours % 10, HOUR_1_PIN );
  flashNixie( outMinutes / 10, MINS_10_PIN );
  flashNixie( outMinutes % 10, MINS_1_PIN );
  flashNixie( outSeconds / 10, SECS_10_PIN );
  flashNixie( outSeconds % 10, SECS_1_PIN );
}

/**
 * Flashes nixie on/off for multiplexing.
 */
void flashNixie( unsigned int value, unsigned int pin )
{
  outputValue( value );
  digitalWrite( pin, HIGH );
  delayMicroseconds( MULTIPLEX_HIGH_DELAY );
  digitalWrite( pin, LOW );
  delayMicroseconds( MULTIPLEX_LOW_DELAY );
}

/**
 * Updates the internal time.
 */
unsigned long updateTime()
{
  cli(); // Disable interrupts
  currentMillis = millis();
  sei(); // Enable interrupts
  
  // Check for overflow!
  if( currentMillis < lastMillis ) {
    tt += MAX_MILLIS - lastMillis + currentMillis;
  }
  else {
    tt += currentMillis - lastMillis;
    
#if QUIRKS_MODE
    if( lastPowerRead == HIGH ) {
      quirkVal = min(tt, (currentMillis - lastQuirkMillis)/QUIRK_DELAY_FACTOR);
      tt -= random(100) < QUIRK_OFFSET_CHANCE ? quirkVal : 0;
      if( tt == 0 && currentMillis != 0 ) {
        setAlarmEnabled( true );
        startAlarm( BEEP_MAX );
        lastQuirkMillis = currentMillis;
      }
    }
#endif

  }
    
  seconds += tt / 1000;
  tt = tt % 1000;
  minutes += seconds / 60;
  seconds = seconds % 60;
  hours += minutes / 60;
  minutes = minutes % 60;
  hours = hours % hourDivisor;
  
  lastMillis = currentMillis;
  
  if( seconds == 13 ) {
    mode = CATHODE_POISON;
  }
  
  return currentMillis;
}

/**
 * Turn off display on all nixies.
 */
void blankDisplay()
{
  digitalWrite( SECS_1_PIN, LOW );
  digitalWrite( SECS_10_PIN, LOW );
  digitalWrite( MINS_1_PIN, LOW );
  digitalWrite( MINS_10_PIN, LOW );
  digitalWrite( HOUR_1_PIN, LOW );
  digitalWrite( HOUR_10_PIN, LOW );
}

/**
 * Scroll up or down through time depending on current scroll velocity
 */
void scrollTime( int &scrollHours, int &scrollMinutes, int &scrollSeconds )
{ 
  st += scrollVelocity;
  scrollSeconds += st / 1000;
  st = st % 1000;
  scrollMinutes += scrollSeconds / 60;
  scrollSeconds = scrollSeconds % 60;
  scrollHours += scrollMinutes / 60;
  scrollMinutes = scrollMinutes % 60;
  scrollHours = scrollHours % hourDivisor;
  
  if( scrollSeconds < 0 ) {
    scrollSeconds = 60 + scrollSeconds;
    scrollMinutes -= 1;
  }
    
  if( scrollMinutes < 0 ) {
    scrollMinutes = 60 + scrollMinutes;
    scrollHours -= 1;
  }
    
  if( scrollHours < 0 )
    scrollHours = hourDivisor + scrollHours;
}

/**
 * Updates the flashing on/off of numeric display during set mode.
 */
boolean updateFlash()
{
  if( millis() - lastFlashMillis >= FLASH_DELAY ) {
    flashOn = !flashOn;
    lastFlashMillis = millis();
  }
  return flashOn;
}

/**
 * Performs exponential growth or shrinkage of scroll velocity
 * depending on static tilt of accelerometer.
 */
void updateScrollVelocity( int pin )
{
  int sign, zero = 0;
  float rho, phi, theta = 0;
  float* angle;
  
  accelerometer.readTilt( &rho, &phi, &theta );
  
  switch( pin ) {
    case X_PIN:
      angle = &rho;
      zero = DEFAULT_X_ZERO_ANGLE;
      break;
      
    case Y_PIN:
      angle = &phi;
      zero = DEFAULT_Y_ZERO_ANGLE;
      break;
  }
  
  *angle = constrain(*angle, zero - ANGLE_THRESHOLD - MAX_ANGLE, zero + ANGLE_THRESHOLD + MAX_ANGLE);
  
  if( *angle <= zero - ANGLE_THRESHOLD ) {
    *angle = abs(*angle - (zero - ANGLE_THRESHOLD));
    sign = 1;
  }
  else if( *angle >= zero + ANGLE_THRESHOLD ) {
    *angle = *angle - (zero + ANGLE_THRESHOLD);
    sign = -1;
  }
  else {
    rho = 0;
    sign = 0;
  }
  
  scrollVelocity = pow(*angle, ANGLE_EXPONENT) * sign;
}

/**
 * Sets pin mode for pins controlling display (INPUT or OUTPUT)
 */
void setDisplayPinsMode( byte mode )
{
  // Setup all the pins
  pinMode( VAL0_PIN, mode );
  pinMode( VAL1_PIN, mode );
  pinMode( VAL2_PIN, mode );
  pinMode( VAL3_PIN, mode );
  pinMode( SECS_1_PIN, mode );
  pinMode( SECS_10_PIN, mode );
  pinMode( MINS_1_PIN, mode );
  pinMode( MINS_10_PIN, mode );
  pinMode( HOUR_1_PIN, mode );
  pinMode( HOUR_10_PIN, mode );
  
#if !DEBUG
  // Alarm pin uses RX so only do this if not using Serial
  pinMode( ALARM_INDICATOR_PIN, mode );
#endif

}

/**
 * Checks for external power. Sets display pins to high impedence on power
 * outage.
 */
void checkExternalPower()
{
  int read = digitalRead( EXTERNAL_POWER_PIN );
  
  if( read != lastPowerRead ) {
    if( read == LOW ) {
      // Power went out
      stopAlarm(); // In case it is running
      setDisplayPinsMode( INPUT ); // high impedence prevents extra backup power loss
      mode = RUN;
      
#if QUIRKS_MODE
      lastQuirkMillis = millis();
#endif

    }
    else {
      // Power back on
      setDisplayPinsMode( OUTPUT );
    }
    lastPowerRead = read;
  }
}

void playTune( int tune, boolean backwards )
{
  switch( tune ) {
    case WESTMINSTER:
      playMelody( westminsterNotes, westminsterNumNotes, westminsterDurations, backwards );
      break;
      
    case TONE_UP:
      playMelody( toneUpNotes, toneUpNumNotes, toneUpDurations, backwards );
      break;
      
    case TONE_UP2:
      playMelody( toneUp2Notes, toneUp2NumNotes, toneUp2Durations, backwards );
      break;
  }
}

void playMelody( int notes[], int numNotes, int durations[], boolean backwards )
{
  int i = backwards ? numNotes - 1 : 0;
  
  while( backwards ? i >= 0 : i < numNotes ) {
    if( notes[i] == REST ) {
      delay( durations[i] );
    }
    else {
      outputTone( notes[i], durations[i] );
    }
    i = backwards ? i-1 : i+1;
  }
}

/**
 * Mostly cargo-culted from http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1241248988
 */
void outputTone( int pitch, int duration )
{ 
  int delayPeriod;
  long cycles, i;

  delayPeriod = (500000 / pitch) - 7; // calc 1/2 period in us -7 for overhead
  cycles = ((long)pitch * (long)duration) / 1000; // calc. number of cycles for loop

  for( i=0; i <= cycles; i++ ) { // play note for duration ms
    digitalWrite( PIEZO_PIN, HIGH );
    delayMicroseconds( delayPeriod );
    digitalWrite( PIEZO_PIN, LOW );
    delayMicroseconds( delayPeriod - 1 ); // - 1 to make up for digitalWrite overhead
  }
}

boolean detectShake( int pin )
{
  int read = analogRead( pin );
  unsigned long curMillis = millis();
  
  int* phasePtr;
  int max, min;
  int xmax, ymax, zmax;
  int xmin, ymin, zmin;
  
  accelerometer.getMaxValues( &xmax, &ymax, &zmax );
  accelerometer.getMinValues( &xmin, &ymin, &zmin );
  
  switch( pin ) {
    case Z_PIN:
      phasePtr = &zShakePhase;
      max = zmax - Z_SHAKE_THRESHOLD;
      min = zmin + Z_SHAKE_THRESHOLD;
      break;
      
    case Y_PIN:
      phasePtr = &yShakePhase;
      max = ymax - Y_SHAKE_THRESHOLD;
      min = ymin + Y_SHAKE_THRESHOLD;
      break;
      
    case X_PIN:
      phasePtr = &xShakePhase;
      max = xmax - X_SHAKE_THRESHOLD;
      min = xmin + X_SHAKE_THRESHOLD;
      break;
  }
  
#if QUIRKS_MODE
  if( read <= min ) {
    lastQuirkMillis = curMillis;
  }
#endif
  
  if( curMillis - lastShakeMillis > SHAKE_TIMEOUT ) {
     *phasePtr = SHAKE_PHASE_1;
  }

  switch( *phasePtr ) {
    case SHAKE_PHASE_1:
      if( read <= min ) {
        *phasePtr = SHAKE_PHASE_2;
        lastShakeMillis = curMillis;
      }
      return false;
        
    case SHAKE_PHASE_2:
      if( read >= max && curMillis - lastShakeMillis <= SHAKE_TIMEOUT ) {
        *phasePtr = SHAKE_PHASE_3;
        lastShakeMillis = curMillis;
      }
      return false;
    
    case SHAKE_PHASE_3:
      if( read <= min && curMillis - lastShakeMillis <= SHAKE_TIMEOUT ) {
        *phasePtr = SHAKE_PHASE_4;
        lastShakeMillis = curMillis;
      }
      return false;
        
    case SHAKE_PHASE_4:
      if( read >= max && curMillis - lastShakeMillis <= SHAKE_TIMEOUT ) {
        *phasePtr = SHAKE_PHASE_1;
        return true;
      }
      return false;
  }
}

void setAlarmEnabled( boolean enable )
{
  alarmEnabled = enable;
  digitalWrite( ALARM_INDICATOR_PIN, enable );
  
  // Beep twice
  for( int i = 0; i < 2; i++ ) {
    digitalWrite( PIEZO_PIN, HIGH );
    delay( 100 );
    digitalWrite( PIEZO_PIN, LOW );
    delay( 100 );
  }
}

void startAlarm( int count )
{
  beepMax = count;
  beepCount = 0;
  beepOn = true;
  alarmOn = true;
  digitalWrite( PIEZO_PIN, HIGH );
  lastBeep = millis();
}

void stopAlarm()
{
  digitalWrite( PIEZO_PIN, LOW );
  beepOn = false;
  alarmOn = false;
  beepCount = 0;
  beepMax = 0;
  
  // Beep twice
  for( int i = 0; i < 2; i++ ) {
    delay( 100 );
    digitalWrite( PIEZO_PIN, HIGH );
    delay( 100 );
    digitalWrite( PIEZO_PIN, LOW );
  }
}

void handleAlarm()
{
  if( millis() - lastBeep >= BEEP_DELAY ) {
    if( beepOn ) {
      digitalWrite( PIEZO_PIN, LOW );
      beepOn = false;
    }
    else {
      digitalWrite( PIEZO_PIN, HIGH );
      beepOn = true;
    }
    if( ++beepCount >= beepMax ) {
      stopAlarm();
      return;
    }
    lastBeep = millis();
  }
}
