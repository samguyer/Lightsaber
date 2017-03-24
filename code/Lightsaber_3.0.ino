#include <adel.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "DFRobotDFPlayerMini.h"
#include <Adafruit_LSM9DS0.h>

// -- Saber state --------------------------

bool g_saber_on = false;

// -- On/Off Button -------------------------------

#define BUTTON_PIN 4

// -- Mode control
//    Determines what the rotary encoder does

#define MODE_BUTTON_PIN 8
#define BRIGHTNESS_A_PIN 5
#define BRIGHTNESS_B_PIN 6

enum Mode { NO_MODE, BRIGHTNESS_MODE, VOLUME_MODE };

Mode g_Mode;

// -- Blade --------------------------------

#define NUM_SEGMENTS 16

// -- PCA9685 driver
//    Called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define TOTAL_LENGTH 1200
#define SEGMENT_LENGTH (TOTAL_LENGTH / NUM_SEGMENTS)

int g_cur_length = 0;

#define TIME_TO_ACTIVATE 600
#define TIME_TO_DEACTIVATE 800

#define MAX_BRIGHTNESS 2400
#define MIN_BRIGHTNESS 500

int g_cur_brightness = 1400;

// -- Gyroscope ----------------------------

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

typedef enum { NONE, UP, DOWN, LEFT, RIGHT } Direction;

// -- Sounds -------------------------------

DFRobotDFPlayerMini myDFPlayer;

int g_volume = 18;
#define MAX_VOLUME 28
#define MIN_VOLUME 1

#define BUSY_BUTTON 3

#define SABER_ON_SOUND 36
#define SABER_OFF_SOUND 34

#define SWING_MIN_SOUND 49
#define SWING_MAX_SOUND 64

#define CLASH_MIN_SOUND 17
#define CLASH_MAX_SOUND 24

// ----------------------------------------------------------------

adel waitbutton(int pin)
{
   abegin:
   while (1) {
      await (digitalRead(pin) == HIGH);
      adelay (50);
      if (digitalRead(pin) == HIGH) {
         await (digitalRead(pin) == LOW);
         afinish;
      }
   }
   aend;
}

// -- Render the saber
//    Length is a number between 0 and TOTAL_LENGTH
void render()
{  
  int segments_on = g_cur_length / SEGMENT_LENGTH;
  int partial = g_cur_length - segments_on * SEGMENT_LENGTH;

  for (int i = 0; i < NUM_SEGMENTS; i++) {
    if (i < segments_on) {
      pwm.setPWM(i, 0, g_cur_brightness );
    }
    if (i == segments_on) {
      int level = map(partial/2, 0, SEGMENT_LENGTH, 0, g_cur_brightness);
      pwm.setPWM(i, 0, level );
    }
    if (i > segments_on) {
      pwm.setPWM(i, 0, 0 );
    }
  }
}

// ----------------------------------------------------------------
//  Rotary encoder
//  Function depends on the mode

adel waitTurn(int pinA, int pinB)
{
  abegin:
  while (1) {
    await( digitalRead(pinA) == HIGH );
    await( digitalRead(pinA) == LOW  );
    if ( digitalRead(pinA) == LOW) {
      await( digitalRead(pinB) == HIGH );
      await( digitalRead(pinB) == LOW  );
      if ( digitalRead(pinB) == LOW && digitalRead(pinA) == LOW)
        afinish;
    }
  }
  aend;
}

adel adjustments()
{
  int delta = 0;
  abegin:
  while (1) {
    delta = 0;
    auntil( waitTurn(BRIGHTNESS_A_PIN, BRIGHTNESS_B_PIN), 
            waitTurn(BRIGHTNESS_B_PIN, BRIGHTNESS_A_PIN) ) {
      delta = 1;
    } else {
      delta = -1;
    }
    
    if (delta != 0) {
        if (g_Mode == BRIGHTNESS_MODE) {
          g_cur_brightness += 50 * delta;
          if (g_cur_brightness > MAX_BRIGHTNESS) g_cur_brightness = MAX_BRIGHTNESS;
          if (g_cur_brightness < MIN_BRIGHTNESS) g_cur_brightness = MIN_BRIGHTNESS;
          render();
        }
        if (g_Mode == VOLUME_MODE) {
          g_volume += delta;
          if (g_volume > MAX_VOLUME) g_volume = MAX_VOLUME;
          if (g_volume < MIN_VOLUME) g_volume = MIN_VOLUME;
          myDFPlayer.volume(g_volume);  //Set volume value (0~30)
        }
    }
  }
  aend;
}

adel mode()
{
  abegin:
  while (1) {
    andthen( waitbutton(MODE_BUTTON_PIN) );
    if (g_Mode == VOLUME_MODE)          g_Mode = NO_MODE;
    else if (g_Mode == BRIGHTNESS_MODE) g_Mode = VOLUME_MODE;
    else if (g_Mode == NO_MODE)         g_Mode = BRIGHTNESS_MODE;
  }
  aend;
}

// ----------------------------------------------------------------
//  Sounds
//  Based on the motion

Adafruit_LSM9DS0::lsm9ds0Vector_t prev_accelData;
Adafruit_LSM9DS0::lsm9ds0Vector_t cur_accelData;

float accelDelta()
{
  float dx = cur_accelData.x - prev_accelData.x;
  float dy = cur_accelData.y - prev_accelData.y;
  float dz = cur_accelData.z - prev_accelData.z;
  return dx * dx + dy * dy + dz * dz;
}

bool getClash()
{
  bool clash = false;
  prev_accelData = cur_accelData;
  cur_accelData.x = lsm.accelData.x / 1000.0;
  cur_accelData.y = lsm.accelData.y / 1000.0;
  cur_accelData.z = lsm.accelData.z / 1000.0;
  float dx = cur_accelData.x - prev_accelData.x;
  float dy = cur_accelData.y - prev_accelData.y;
  float dz = cur_accelData.z - prev_accelData.z;
  float d = dx * dx + dy * dy + dz * dz;
  // float d = accelDelta();
  // -- Threshold for clash
  // Serial.println(d);
  if (d > 2200.0) clash = true;
  return clash;
}


Direction getSwing()
{
  Direction swing = NONE;
  
  float gyroY = ((float) lsm.gyroData.y) / 4000.0;
  float gyroZ = ((float) lsm.gyroData.z) / 4000.0;

  float mag = sqrt(gyroY * gyroY + gyroZ * gyroZ);

  if (mag >= 4.0) {
    if (abs(gyroY) > abs(gyroZ)) {
      // -- Mostly up-down
      if (gyroY < 0.0) 
        swing = UP;
      else 
        swing = DOWN;
    } else {
      // -- Mostly left-right
      if (gyroZ < 0.0) 
        swing = RIGHT;
      else 
        swing = LEFT;
    }
  }

  return swing;
}

adel swing()
{
  int sound;
  abegin:
    if (g_saber_on) {
      if (getSwing() != NONE) {
        // -- Play a random swing
        sound = SWING_MIN_SOUND + random(SWING_MAX_SOUND - SWING_MIN_SOUND);
        myDFPlayer.play(sound);
        // -- Wait for the sound to finish before starting a new one
        // await( digitalRead(BUSY_BUTTON) == HIGH );
        adelay(200);
      } 
    }
  aend;
}

adel clash()
{
  int sound;
  abegin:
    if (g_saber_on) {
      if (getClash()) {
        // -- Play a random clash
        sound = CLASH_MIN_SOUND + random(CLASH_MAX_SOUND - CLASH_MIN_SOUND);
        myDFPlayer.play(sound);
        // await( digitalRead(BUSY_BUTTON) == HIGH );
        adelay(100);
      }
    }
 
  aend;
}

// ----------------------------------------------------------------
//  Saber on and off 
//  Controls the scrolling effect

adel saber_on()
{
  abegin:

  aramp( TIME_TO_ACTIVATE, g_cur_length, 0, TOTAL_LENGTH ) {
    render();
    adelay(10);
  }

  g_cur_length = TOTAL_LENGTH;
  g_saber_on = true;
  render();

  aend;
}

adel saber_off()
{
  abegin:

  aramp( TIME_TO_ACTIVATE, g_cur_length, TOTAL_LENGTH, 0 ) {
    render();
    adelay(10);
  }

  g_cur_length = 0;
  g_saber_on = false;
  render();

  aend;
}

adel onoff()
{
  abegin:

  while (1) {
    andthen( waitbutton(BUTTON_PIN) );
    myDFPlayer.play(SABER_ON_SOUND);
    adelay(200);
    andthen( saber_on() );
    andthen( waitbutton(BUTTON_PIN) );
    myDFPlayer.play(SABER_OFF_SOUND);
    adelay(500);
    andthen( saber_off() );
  }
  
  aend;
}

// -----------------------------------------------
//    Set up
// -----------------------------------------------

void setup()
{
  delay(500);
  Serial.begin (9600);

  // -- Set up the DFPlayer
  delay(200);
  myDFPlayer.begin(Serial); // mp3_set_serial (Serial);  //set Serial for DFPlayer-mini mp3 module
  myDFPlayer.volume(g_volume);  //Set volume value (0~30)
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
  
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, LOW);

  pinMode(BRIGHTNESS_A_PIN, INPUT);
  digitalWrite(BRIGHTNESS_A_PIN, HIGH);                // Turn on internal pullup resistor
  pinMode(BRIGHTNESS_B_PIN, INPUT);
  digitalWrite(BRIGHTNESS_B_PIN, HIGH);                // Turn on internal pullup resistor

  // -- Set up the gyroscope
  lsm.begin();
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  
  // -- Set up the PCA9685 PWM module  
  pwm.begin();
  pwm.setPWMFreq(1600);  // This is the maximum PWM frequency

  g_cur_length = 0;
  render();

  g_Mode = NO_MODE;
}

// -----------------------------------------------
//   Main loop
// -----------------------------------------------

void loop()
{
  lsm.read();
  arepeat( onoff() );
  aevery(20, clash() );
  aevery(20, swing() );
  arepeat( adjustments() );
  arepeat( mode() );
}


